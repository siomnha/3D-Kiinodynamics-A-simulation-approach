import math
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node


class TrajectoryAdapter(Node):
    """Generate smooth time-parameterized references from pruned waypoints."""

    def __init__(self) -> None:
        super().__init__('trajectory_adapter')

        self.declare_parameter('input_topic', '/planning/pruned_path')
        self.declare_parameter('fallback_input_topic', '/nav6d/planner/path')
        self.declare_parameter('output_topic', '/trajectory/reference')

        # Trajectory strategy switch:
        # - optimize_traj: slower, safer allocation (recommended for nav6d + pruning)
        # - preview: faster preview for quick RViz checks
        self.declare_parameter('traj_mode', 'optimize_traj')

        # Design parameters to tune
        self.declare_parameter('v_ref', 1.2)
        self.declare_parameter('a_ref', 0.8)
        self.declare_parameter('max_velocity', 2.0)
        self.declare_parameter('max_acceleration', 1.5)
        self.declare_parameter('min_segment_time', 0.08)
        self.declare_parameter('time_scale', 1.15)
        self.declare_parameter('corner_time_gain', 0.35)
        self.declare_parameter('sample_dt', 0.1)

        self.traj_mode = self.get_parameter('traj_mode').get_parameter_value().string_value
        self.v_ref = self.get_parameter('v_ref').get_parameter_value().double_value
        self.a_ref = self.get_parameter('a_ref').get_parameter_value().double_value
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
        self.max_acceleration = self.get_parameter('max_acceleration').get_parameter_value().double_value
        self.min_segment_time = self.get_parameter('min_segment_time').get_parameter_value().double_value
        self.time_scale = self.get_parameter('time_scale').get_parameter_value().double_value
        self.corner_time_gain = self.get_parameter('corner_time_gain').get_parameter_value().double_value
        self.sample_dt = self.get_parameter('sample_dt').get_parameter_value().double_value

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        fallback_input = self.get_parameter('fallback_input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.path_pub = self.create_publisher(Path, output_topic, 10)
        self.create_subscription(Path, input_topic, self.path_callback, 10)
        self.create_subscription(Path, fallback_input, self.fallback_callback, 10)

        self.received_pruned = False

        self.get_logger().info(
            'Trajectory adapter started. '
            f'Primary={input_topic}, fallback={fallback_input}, out={output_topic}, '
            f'mode={self.traj_mode}, v_ref={self.v_ref:.2f}, a_ref={self.a_ref:.2f}, '
            f'v_max={self.max_velocity:.2f}, a_max={self.max_acceleration:.2f}'
        )

    def fallback_callback(self, msg: Path) -> None:
        if self.received_pruned:
            return
        self.path_callback(msg)

    def path_callback(self, msg: Path) -> None:
        self.received_pruned = True
        if len(msg.poses) < 2:
            return

        points = [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in msg.poses]
        segment_times = self.allocate_segment_times(points)
        tangents = self.compute_tangents(points)

        sampled = self.sample_trajectory(points, tangents, segment_times, self.sample_dt)

        out = Path()
        out.header = msg.header
        out.poses = []
        for x, y, z in sampled:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            ps.pose.orientation.w = 1.0
            out.poses.append(ps)

        self.path_pub.publish(out)
        self.get_logger().info(
            f'Reference trajectory: {len(msg.poses)} waypoints -> {len(out.poses)} samples, '
            f'segment_time_sum={sum(segment_times):.2f}s'
        )

    def allocate_segment_times(self, pts: List[Tuple[float, float, float]]) -> List[float]:
        times: List[float] = []

        v_eff = min(max(self.v_ref, 1e-3), max(self.max_velocity, 1e-3))
        a_eff = min(max(self.a_ref, 1e-3), max(self.max_acceleration, 1e-3))

        for i in range(len(pts) - 1):
            dist = self.euclidean(pts[i], pts[i + 1])
            base_t = max(dist / v_eff, math.sqrt(dist / a_eff))

            if self.traj_mode == 'optimize_traj':
                corner_penalty = 1.0 + self.corner_time_gain * self.corner_factor(pts, i)
                t = base_t * max(self.time_scale, 0.1) * corner_penalty
            else:
                # preview mode keeps lower latency for quick checks
                t = base_t * max(self.time_scale * 0.75, 0.05)

            times.append(max(t, self.min_segment_time, self.sample_dt))

        return times

    def corner_factor(self, pts: List[Tuple[float, float, float]], segment_index: int) -> float:
        """Return normalized corner sharpness [0, 1] around the segment endpoint."""
        if segment_index <= 0 or segment_index >= len(pts) - 2:
            return 0.0

        p_prev = pts[segment_index - 1]
        p_curr = pts[segment_index]
        p_next = pts[segment_index + 1]

        v1 = (p_curr[0] - p_prev[0], p_curr[1] - p_prev[1], p_curr[2] - p_prev[2])
        v2 = (p_next[0] - p_curr[0], p_next[1] - p_curr[1], p_next[2] - p_curr[2])

        n1 = self.norm(v1)
        n2 = self.norm(v2)
        if n1 < 1e-6 or n2 < 1e-6:
            return 0.0

        cosang = max(-1.0, min(1.0, self.dot(v1, v2) / (n1 * n2)))
        # straight line => factor ~0, sharp corner => factor ~1
        return (1.0 - cosang) * 0.5

    def compute_tangents(self, pts: List[Tuple[float, float, float]]) -> List[Tuple[float, float, float]]:
        tangents: List[Tuple[float, float, float]] = []
        n = len(pts)
        for i in range(n):
            if i == 0:
                dx = pts[1][0] - pts[0][0]
                dy = pts[1][1] - pts[0][1]
                dz = pts[1][2] - pts[0][2]
            elif i == n - 1:
                dx = pts[-1][0] - pts[-2][0]
                dy = pts[-1][1] - pts[-2][1]
                dz = pts[-1][2] - pts[-2][2]
            else:
                dx = (pts[i + 1][0] - pts[i - 1][0]) * 0.5
                dy = (pts[i + 1][1] - pts[i - 1][1]) * 0.5
                dz = (pts[i + 1][2] - pts[i - 1][2]) * 0.5
            tangents.append((dx, dy, dz))
        return tangents

    def sample_trajectory(
        self,
        pts: List[Tuple[float, float, float]],
        tangents: List[Tuple[float, float, float]],
        segment_times: List[float],
        dt: float,
    ) -> List[Tuple[float, float, float]]:
        sampled: List[Tuple[float, float, float]] = []
        for i, t_total in enumerate(segment_times):
            p0, p1 = pts[i], pts[i + 1]
            m0, m1 = tangents[i], tangents[i + 1]
            steps = max(2, int(math.ceil(t_total / dt)))

            for step in range(steps):
                s = step / float(steps)
                sampled.append(self.hermite(p0, p1, m0, m1, s))

        sampled.append(pts[-1])
        return sampled

    @staticmethod
    def hermite(
        p0: Tuple[float, float, float],
        p1: Tuple[float, float, float],
        m0: Tuple[float, float, float],
        m1: Tuple[float, float, float],
        s: float,
    ) -> Tuple[float, float, float]:
        h00 = 2 * s**3 - 3 * s**2 + 1
        h10 = s**3 - 2 * s**2 + s
        h01 = -2 * s**3 + 3 * s**2
        h11 = s**3 - s**2

        x = h00 * p0[0] + h10 * m0[0] + h01 * p1[0] + h11 * m1[0]
        y = h00 * p0[1] + h10 * m0[1] + h01 * p1[1] + h11 * m1[1]
        z = h00 * p0[2] + h10 * m0[2] + h01 * p1[2] + h11 * m1[2]
        return (x, y, z)

    @staticmethod
    def euclidean(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        dz = a[2] - b[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    @staticmethod
    def dot(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

    @staticmethod
    def norm(a: Tuple[float, float, float]) -> float:
        return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def main() -> None:
    rclpy.init()
    node = TrajectoryAdapter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
