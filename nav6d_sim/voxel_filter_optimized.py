import time
from typing import List

import numpy as np
import open3d as o3d
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


class VoxelFilterOptimized(Node):
    """Downsample incoming point clouds before OctoMap ingestion."""

    def __init__(self) -> None:
        super().__init__('voxel_filter')

        self.declare_parameter('input_topic', '/fused_cloud')
        self.declare_parameter('output_topic', '/filtered_cloud')
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('max_points', 20000)
        self.declare_parameter('enable_statistical_filter', False)
        self.declare_parameter('drop_old_messages', True)
        self.declare_parameter('max_message_age_s', 0.1)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.max_points = int(self.get_parameter('max_points').value)
        self.enable_stat_filter = bool(self.get_parameter('enable_statistical_filter').value)
        self.drop_old = bool(self.get_parameter('drop_old_messages').value)
        self.max_message_age_s = float(self.get_parameter('max_message_age_s').value)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_callback,
            qos_profile,
        )
        self.publisher = self.create_publisher(
            PointCloud2,
            self.output_topic,
            qos_profile,
        )

        self.callback_count = 0
        self.dropped_count = 0
        self.last_log_time = time.time()

        self.get_logger().info(
            f'VoxelFilterOptimized ready: {self.input_topic} -> {self.output_topic}, '
            f'voxel_size={self.voxel_size:.3f}m, max_points={self.max_points}, '
            f'drop_old_messages={self.drop_old}'
        )

    def cloud_callback(self, msg: PointCloud2) -> None:
        if self.drop_old:
            current_time = self.get_clock().now()
            msg_time = rclpy.time.Time.from_msg(msg.header.stamp)
            message_age = (current_time - msg_time).nanoseconds / 1e9
            if message_age > self.max_message_age_s:
                self.dropped_count += 1
                if self.dropped_count % 50 == 0:
                    self.get_logger().warn(
                        f'Dropped {self.dropped_count} stale point clouds older than '
                        f'{self.max_message_age_s:.3f}s'
                    )
                return

        try:
            start = time.time()

            points_list: List[List[float]] = []
            for count, point in enumerate(pc2.read_points(msg, skip_nans=True), start=1):
                points_list.append([point[0], point[1], point[2]])
                if count >= self.max_points:
                    break

            if not points_list:
                return

            points = np.asarray(points_list, dtype=np.float32)
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(points)

            filtered = cloud.voxel_down_sample(self.voxel_size)

            if self.enable_stat_filter and len(filtered.points) > 10:
                filtered, _ = filtered.remove_statistical_outlier(nb_neighbors=10, std_ratio=2.0)

            filtered_points = np.asarray(filtered.points, dtype=np.float32)
            if filtered_points.size == 0:
                return

            header = Header()
            header.frame_id = msg.header.frame_id
            header.stamp = self.get_clock().now().to_msg()

            cloud_msg = pc2.create_cloud_xyz32(header, filtered_points)
            self.publisher.publish(cloud_msg)

            self.callback_count += 1
            elapsed_ms = (time.time() - start) * 1000.0
            now = time.time()
            if now - self.last_log_time > 2.0:
                self.get_logger().info(
                    f'Processed {self.callback_count} clouds | '
                    f'latency={elapsed_ms:.2f}ms | '
                    f'points {len(points)} -> {len(filtered_points)}'
                )
                self.last_log_time = now
        except Exception as exc:
            self.get_logger().error(f'Voxel filter processing error: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoxelFilterOptimized()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
