import importlib.util
import math
from typing import Callable, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node

Point3 = Tuple[float, float, float]


class Nav6DOptimizeTraj(Node):
    """A* -> pruning -> optimize_traj(minimum-snap) integrated pipeline for nav6d."""

    def __init__(self) -> None:
        super().__init__('nav_6d_optimize_traj')

        self.declare_parameter('input_topic', '/nav6d/planner/path')
        self.declare_parameter('pruned_topic', '/planning/pruned_path')
        self.declare_parameter('reference_topic', '/trajectory/reference')
        self.declare_parameter('state_topic', '/trajectory/state')
        self.declare_parameter('pose_topic', '/space_cobot/pose')

        self.declare_parameter('min_point_distance', 0.15)
        self.declare_parameter('rdp_epsilon', 0.25)

        # optimize_traj tuning knobs
        self.declare_parameter('v_ref', 1.2)
        self.declare_parameter('a_ref', 0.8)
        self.declare_parameter('max_velocity', 2.0)
        self.declare_parameter('max_acceleration', 1.5)
        self.declare_parameter('time_scale', 1.15)
        self.declare_parameter('corner_time_gain', 0.35)
        self.declare_parameter('sample_dt', 0.08)
        self.declare_parameter('min_segment_time', 0.08)

        # External solver adapter (user-provided minimum snap file)
        self.declare_parameter('external_solver_module', 'minimum_snap')
        self.declare_parameter('external_solver_function', 'optimize_traj')

        self.min_point_distance = self.get_parameter('min_point_distance').value
        self.rdp_epsilon = self.get_parameter('rdp_epsilon').value

        self.v_ref = self.get_parameter('v_ref').value
        self.a_ref = self.get_parameter('a_ref').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.max_acceleration = self.get_parameter('max_acceleration').value
        self.time_scale = self.get_parameter('time_scale').value
        self.corner_time_gain = self.get_parameter('corner_time_gain').value
        self.sample_dt = self.get_parameter('sample_dt').value
        self.min_segment_time = self.get_parameter('min_segment_time').value

        self.external_solver: Optional[Callable[..., List[Point3]]] = self.resolve_external_solver(
            self.get_parameter('external_solver_module').value,
            self.get_parameter('external_solver_function').value,
        )

        input_topic = self.get_parameter('input_topic').value
        pruned_topic = self.get_parameter('pruned_topic').value
        reference_topic = self.get_parameter('reference_topic').value
        state_topic = self.get_parameter('state_topic').value
        pose_topic = self.get_parameter('pose_topic').value

        self.pruned_pub = self.create_publisher(Path, pruned_topic, 10)
        self.reference_pub = self.create_publisher(Path, reference_topic, 10)
        self.state_pub = self.create_publisher(PoseStamped, state_topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)

        self.create_subscription(Path, input_topic, self.path_callback, 10)

        self.reference_path: List[PoseStamped] = []
        self.ref_idx = 0
        self.timer = self.create_timer(0.05, self.publish_state_step)

        self.get_logger().info(
            f'nav_6d optimize_traj ready: {input_topic} -> {pruned_topic} -> {reference_topic}; '
            f'external_solver={self.external_solver is not None}'
        )

    def resolve_external_solver(self, module_name: str, function_name: str) -> Optional[Callable[..., List[Point3]]]:
        spec = importlib.util.find_spec(module_name)
        if spec is None:
            return None
        module = importlib.import_module(module_name)
        func = getattr(module, function_name, None)
        if callable(func):
            self.get_logger().info(f'Using external solver: {module_name}.{function_name}')
            return func
        return None

    def path_callback(self, msg: Path) -> None:
        if len(msg.poses) < 2:
            return

        raw = msg.poses
        pruned = self.rdp(self.remove_duplicates(raw, self.min_point_distance), self.rdp_epsilon)

        pruned_msg = Path()
        pruned_msg.header = msg.header
        pruned_msg.poses = pruned
        self.pruned_pub.publish(pruned_msg)

        points = [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in pruned]
        segment_times = self.allocate_segment_times(points)

        if self.external_solver is not None:
            samples = self.external_solver(points=points, segment_times=segment_times, dt=self.sample_dt)
        else:
            tangents = self.compute_tangents(points)
            samples = self.sample_hermite(points, tangents, segment_times, self.sample_dt)

        ref_msg = Path()
        ref_msg.header = msg.header
        ref_msg.poses = [self.point_to_pose(msg.header.frame_id, p) for p in samples]
        self.reference_pub.publish(ref_msg)

        self.reference_path = ref_msg.poses
        self.ref_idx = 0

        self.get_logger().info(
            f'A*: {len(raw)} -> prune: {len(pruned)} -> reference: {len(ref_msg.poses)}'
        )

    def publish_state_step(self) -> None:
        if not self.reference_path or self.ref_idx >= len(self.reference_path):
            return
        src = self.reference_path[self.ref_idx]
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = src.header.frame_id if src.header.frame_id else 'map'
        out.pose = src.pose
        self.state_pub.publish(out)
        self.pose_pub.publish(out)
        self.ref_idx += 1

    def allocate_segment_times(self, points: List[Point3]) -> List[float]:
        times: List[float] = []
        v_eff = min(max(self.v_ref, 1e-3), max(self.max_velocity, 1e-3))
        a_eff = min(max(self.a_ref, 1e-3), max(self.max_acceleration, 1e-3))
        for i in range(len(points) - 1):
            dist = self.euclidean(points[i], points[i + 1])
            base_t = max(dist / v_eff, math.sqrt(dist / a_eff))
            corner_penalty = 1.0 + self.corner_time_gain * self.corner_factor(points, i)
            t = max(base_t * self.time_scale * corner_penalty, self.min_segment_time, self.sample_dt)
            times.append(t)
        return times

    def corner_factor(self, points: List[Point3], i: int) -> float:
        if i <= 0 or i >= len(points) - 2:
            return 0.0
        p0, p1, p2 = points[i - 1], points[i], points[i + 1]
        v1 = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
        v2 = (p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2])
        n1 = self.norm(v1)
        n2 = self.norm(v2)
        if n1 < 1e-6 or n2 < 1e-6:
            return 0.0
        cosang = max(-1.0, min(1.0, self.dot(v1, v2) / (n1 * n2)))
        return (1.0 - cosang) * 0.5

    def sample_hermite(
        self, points: List[Point3], tangents: List[Point3], segment_times: List[float], dt: float
    ) -> List[Point3]:
        out: List[Point3] = []
        for i, t_total in enumerate(segment_times):
            p0, p1 = points[i], points[i + 1]
            m0, m1 = tangents[i], tangents[i + 1]
            steps = max(2, int(math.ceil(t_total / dt)))
            for k in range(steps):
                s = k / float(steps)
                out.append(self.hermite(p0, p1, m0, m1, s))
        out.append(points[-1])
        return out

    def compute_tangents(self, points: List[Point3]) -> List[Point3]:
        tangents: List[Point3] = []
        n = len(points)
        for i in range(n):
            if i == 0:
                tangents.append((points[1][0] - points[0][0], points[1][1] - points[0][1], points[1][2] - points[0][2]))
            elif i == n - 1:
                tangents.append((points[-1][0] - points[-2][0], points[-1][1] - points[-2][1], points[-1][2] - points[-2][2]))
            else:
                tangents.append(
                    (
                        0.5 * (points[i + 1][0] - points[i - 1][0]),
                        0.5 * (points[i + 1][1] - points[i - 1][1]),
                        0.5 * (points[i + 1][2] - points[i - 1][2]),
                    )
                )
        return tangents

    @staticmethod
    def point_to_pose(frame_id: str, p: Point3) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = frame_id if frame_id else 'map'
        msg.pose.position.x = p[0]
        msg.pose.position.y = p[1]
        msg.pose.position.z = p[2]
        msg.pose.orientation.w = 1.0
        return msg

    @staticmethod
    def remove_duplicates(poses: List[PoseStamped], min_dist: float) -> List[PoseStamped]:
        if len(poses) <= 1:
            return poses
        filtered = [poses[0]]
        for pose in poses[1:]:
            if Nav6DOptimizeTraj.pose_distance(filtered[-1], pose) >= min_dist:
                filtered.append(pose)
        if filtered[-1] is not poses[-1]:
            filtered.append(poses[-1])
        return filtered

    def rdp(self, poses: List[PoseStamped], epsilon: float) -> List[PoseStamped]:
        if len(poses) < 3:
            return poses
        start = poses[0]
        end = poses[-1]
        max_dist = -1.0
        idx = -1
        for i in range(1, len(poses) - 1):
            d = self.point_line_distance(poses[i], start, end)
            if d > max_dist:
                max_dist = d
                idx = i
        if max_dist > epsilon:
            left = self.rdp(poses[: idx + 1], epsilon)
            right = self.rdp(poses[idx:], epsilon)
            return left[:-1] + right
        return [start, end]

    @staticmethod
    def point_line_distance(point: PoseStamped, start: PoseStamped, end: PoseStamped) -> float:
        px, py, pz = point.pose.position.x, point.pose.position.y, point.pose.position.z
        sx, sy, sz = start.pose.position.x, start.pose.position.y, start.pose.position.z
        ex, ey, ez = end.pose.position.x, end.pose.position.y, end.pose.position.z
        vx, vy, vz = ex - sx, ey - sy, ez - sz
        wx, wy, wz = px - sx, py - sy, pz - sz
        vv = vx * vx + vy * vy + vz * vz
        if vv < 1e-9:
            return math.sqrt(wx * wx + wy * wy + wz * wz)
        t = max(0.0, min(1.0, (wx * vx + wy * vy + wz * vz) / vv))
        cx, cy, cz = sx + t * vx, sy + t * vy, sz + t * vz
        dx, dy, dz = px - cx, py - cy, pz - cz
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    @staticmethod
    def pose_distance(a: PoseStamped, b: PoseStamped) -> float:
        dx = a.pose.position.x - b.pose.position.x
        dy = a.pose.position.y - b.pose.position.y
        dz = a.pose.position.z - b.pose.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    @staticmethod
    def hermite(p0: Point3, p1: Point3, m0: Point3, m1: Point3, s: float) -> Point3:
        h00 = 2 * s**3 - 3 * s**2 + 1
        h10 = s**3 - 2 * s**2 + s
        h01 = -2 * s**3 + 3 * s**2
        h11 = s**3 - s**2
        return (
            h00 * p0[0] + h10 * m0[0] + h01 * p1[0] + h11 * m1[0],
            h00 * p0[1] + h10 * m0[1] + h01 * p1[1] + h11 * m1[1],
            h00 * p0[2] + h10 * m0[2] + h01 * p1[2] + h11 * m1[2],
        )

    @staticmethod
    def euclidean(a: Point3, b: Point3) -> float:
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)

    @staticmethod
    def dot(a: Point3, b: Point3) -> float:
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

    @staticmethod
    def norm(a: Point3) -> float:
        return math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)


def main() -> None:
    rclpy.init()
    node = Nav6DOptimizeTraj()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
