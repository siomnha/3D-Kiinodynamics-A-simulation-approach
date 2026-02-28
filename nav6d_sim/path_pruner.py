import math
from typing import List

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node


class PathPruner(Node):
    """Reduce noisy local waypoints before trajectory generation."""

    def __init__(self) -> None:
        super().__init__('path_pruner')

        self.declare_parameter('input_topic', '/nav6d/planner/path')
        self.declare_parameter('output_topic', '/planning/pruned_path')
        self.declare_parameter('min_point_distance', 0.15)
        self.declare_parameter('rdp_epsilon', 0.25)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.min_point_distance = self.get_parameter('min_point_distance').get_parameter_value().double_value
        self.rdp_epsilon = self.get_parameter('rdp_epsilon').get_parameter_value().double_value

        self.path_pub = self.create_publisher(Path, output_topic, 10)
        self.create_subscription(Path, input_topic, self.path_callback, 10)

        self.get_logger().info(
            f'Path pruner started. {input_topic} -> {output_topic}, '
            f'min_dist={self.min_point_distance:.2f}, epsilon={self.rdp_epsilon:.2f}'
        )

    def path_callback(self, msg: Path) -> None:
        if not msg.poses:
            return

        poses = self.remove_duplicates(msg.poses, self.min_point_distance)
        poses = self.rdp(poses, self.rdp_epsilon)

        out = Path()
        out.header = msg.header
        out.poses = poses
        self.path_pub.publish(out)

        self.get_logger().info(
            f'Pruned path: {len(msg.poses)} -> {len(out.poses)} points.'
        )

    def remove_duplicates(self, poses: List[PoseStamped], min_dist: float) -> List[PoseStamped]:
        if len(poses) <= 1:
            return poses

        filtered = [poses[0]]
        for pose in poses[1:]:
            if self.distance(filtered[-1], pose) >= min_dist:
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
        index = -1

        for i in range(1, len(poses) - 1):
            d = self.point_line_distance(poses[i], start, end)
            if d > max_dist:
                max_dist = d
                index = i

        if max_dist > epsilon:
            left = self.rdp(poses[: index + 1], epsilon)
            right = self.rdp(poses[index:], epsilon)
            return left[:-1] + right

        return [start, end]

    @staticmethod
    def distance(a: PoseStamped, b: PoseStamped) -> float:
        dx = a.pose.position.x - b.pose.position.x
        dy = a.pose.position.y - b.pose.position.y
        dz = a.pose.position.z - b.pose.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

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


def main() -> None:
    rclpy.init()
    node = PathPruner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
