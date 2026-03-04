import math
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node


class TrajectoryModeGenerator(Node):
    """Generate reference trajectories with configurable modes for RViz validation."""

    def __init__(self) -> None:
        super().__init__('trajectory_mode_generator')

        self.declare_parameter('mode', 'circle')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_topic', '/trajectory/reference')
        self.declare_parameter('samples', 200)
        self.declare_parameter('center', [0.0, 0.0, 1.5])
        self.declare_parameter('line_end', [5.0, 0.0, 1.5])
        self.declare_parameter('radius', 3.0)
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('height_delta', 2.0)
        self.declare_parameter('amplitude_y', 2.0)

        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.samples = int(self.get_parameter('samples').get_parameter_value().integer_value)
        self.center = tuple(self.get_parameter('center').get_parameter_value().double_array_value)
        self.line_end = tuple(self.get_parameter('line_end').get_parameter_value().double_array_value)
        self.radius = self.get_parameter('radius').get_parameter_value().double_value
        self.speed = max(0.1, self.get_parameter('speed').get_parameter_value().double_value)
        self.height_delta = self.get_parameter('height_delta').get_parameter_value().double_value
        self.amplitude_y = self.get_parameter('amplitude_y').get_parameter_value().double_value

        topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        self.path_pub = self.create_publisher(Path, topic, 10)
        self.timer = self.create_timer(1.0, self.publish_once)

        self.get_logger().info(f'Trajectory mode generator mode={self.mode}, topic={topic}')

    def publish_once(self) -> None:
        path = self.build_path()
        self.path_pub.publish(path)

    def build_path(self) -> Path:
        mode = self.mode.lower()
        points = {
            'hover': self.mode_hover,
            'line': self.mode_line,
            'circle': self.mode_circle,
            'figure8': self.mode_figure8,
            'helix': self.mode_helix,
        }.get(mode, self.mode_circle)()

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id
        path.poses = [self.point_to_pose(p) for p in points]
        return path

    def mode_hover(self) -> List[Tuple[float, float, float]]:
        return [self.center for _ in range(max(2, self.samples // 10))]

    def mode_line(self) -> List[Tuple[float, float, float]]:
        sx, sy, sz = self.center
        ex, ey, ez = self.line_end
        pts = []
        for i in range(self.samples):
            t = i / max(1, self.samples - 1)
            pts.append((sx + t * (ex - sx), sy + t * (ey - sy), sz + t * (ez - sz)))
        return pts

    def mode_circle(self) -> List[Tuple[float, float, float]]:
        cx, cy, cz = self.center
        pts = []
        for i in range(self.samples):
            th = 2.0 * math.pi * i / max(1, self.samples - 1)
            pts.append((cx + self.radius * math.cos(th), cy + self.radius * math.sin(th), cz))
        return pts

    def mode_figure8(self) -> List[Tuple[float, float, float]]:
        cx, cy, cz = self.center
        pts = []
        for i in range(self.samples):
            th = 2.0 * math.pi * i / max(1, self.samples - 1)
            x = cx + self.radius * math.sin(th)
            y = cy + self.amplitude_y * math.sin(th) * math.cos(th)
            pts.append((x, y, cz))
        return pts

    def mode_helix(self) -> List[Tuple[float, float, float]]:
        cx, cy, cz = self.center
        pts = []
        for i in range(self.samples):
            t = i / max(1, self.samples - 1)
            th = 2.0 * math.pi * t * self.speed
            z = cz + t * self.height_delta
            pts.append((cx + self.radius * math.cos(th), cy + self.radius * math.sin(th), z))
        return pts

    def point_to_pose(self, point: Tuple[float, float, float]) -> PoseStamped:
        x, y, z = point
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.w = 1.0
        return ps


def main() -> None:
    rclpy.init()
    node = TrajectoryModeGenerator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
