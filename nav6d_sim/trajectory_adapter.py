import math
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node


class TrajectoryAdapter(Node):
    """Generate a smooth time-parameterized reference from pruned waypoints."""

    def __init__(self) -> None:
        super().__init__('trajectory_adapter')

        self.declare_parameter('input_topic', '/planning/pruned_path')
        self.declare_parameter('fallback_input_topic', '/nav6d/planner/path')
        self.declare_parameter('output_topic', '/trajectory/reference')
        self.declare_parameter('v_ref', 1.2)
        self.declare_parameter('a_ref', 0.8)
        self.declare_parameter('sample_dt', 0.1)

        self.v_ref = self.get_parameter('v_ref').get_parameter_value().double_value
        self.a_ref = self.get_parameter('a_ref').get_parameter_value().double_value
        self.sample_dt = self.get_parameter('sample_dt').get_parameter_value().double_value

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        fallback_input = self.get_parameter('fallback_input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.path_pub = self.create_publisher(Path, output_topic, 10)
        self.create_subscription(Path, input_topic, self.path_callback, 10)
        self.create_subscription(Path, fallback_input, self.fallback_callback, 10)

        self.received_pruned = False

        self.get_logger().info(
            f'Trajectory adapter started. Primary={input_topic}, fallback={fallback_input}, out={output_topic}'
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
            f'Reference trajectory: {len(msg.poses)} waypoints -> {len(out.poses)} sampled points.'
        )

    def allocate_segment_times(self, pts: List[Tuple[float, float, float]]) -> List[float]:
        times: List[float] = []
        for i in range(len(pts) - 1):
            dist = self.euclidean(pts[i], pts[i + 1])
            t = max(dist / max(self.v_ref, 1e-3), math.sqrt(dist / max(self.a_ref, 1e-3)))
            times.append(max(t, self.sample_dt))
        return times

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


def main() -> None:
    rclpy.init()
    node = TrajectoryAdapter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
