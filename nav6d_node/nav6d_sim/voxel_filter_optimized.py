import time
from typing import List

import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


class VoxelFilterOptimized(Node):
    """Downsample incoming point clouds before OctoMap ingestion using NumPy only."""

    def __init__(self) -> None:
        super().__init__('voxel_filter')

        self.declare_parameter('input_topic', '/fused_cloud')
        self.declare_parameter('output_topic', '/filtered_cloud')
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('max_points', 20000)
        self.declare_parameter('drop_old_messages', True)
        self.declare_parameter('max_message_age_s', 0.1)
        self.declare_parameter('min_z', -np.inf)
        self.declare_parameter('max_z', np.inf)
        self.declare_parameter('min_x', -np.inf)
        self.declare_parameter('max_x', np.inf)
        self.declare_parameter('min_y', -np.inf)
        self.declare_parameter('max_y', np.inf)
        self.declare_parameter('min_points_per_voxel', 1)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.max_points = int(self.get_parameter('max_points').value)
        self.drop_old = bool(self.get_parameter('drop_old_messages').value)
        self.max_message_age_s = float(self.get_parameter('max_message_age_s').value)

        self.min_z = float(self.get_parameter('min_z').value)
        self.max_z = float(self.get_parameter('max_z').value)
        self.min_x = float(self.get_parameter('min_x').value)
        self.max_x = float(self.get_parameter('max_x').value)
        self.min_y = float(self.get_parameter('min_y').value)
        self.max_y = float(self.get_parameter('max_y').value)
        self.min_points_per_voxel = int(self.get_parameter('min_points_per_voxel').value)

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
            f'min_points_per_voxel={self.min_points_per_voxel}'
        )

    def cloud_callback(self, msg: PointCloud2) -> None:
        if self.drop_old:
            current_time = self.get_clock().now()
            msg_time = rclpy.time.Time.from_msg(msg.header.stamp)
            message_age = (current_time - msg_time).nanoseconds / 1e9
            if message_age > self.max_message_age_s:
                self.dropped_count += 1
                return

        try:
            start = time.time()

            points_list: List[List[float]] = []
            for count, point in enumerate(pc2.read_points(msg, skip_nans=True), start=1):
                x, y, z = float(point[0]), float(point[1]), float(point[2])

                if not (self.min_x <= x <= self.max_x):
                    continue
                if not (self.min_y <= y <= self.max_y):
                    continue
                if not (self.min_z <= z <= self.max_z):
                    continue

                points_list.append([x, y, z])
                if len(points_list) >= self.max_points:
                    break

            if not points_list:
                return

            points = np.asarray(points_list, dtype=np.float32)

            voxel_indices = np.floor(points / self.voxel_size).astype(np.int32)
            unique_voxels, inverse_indices, counts = np.unique(
                voxel_indices,
                axis=0,
                return_inverse=True,
                return_counts=True,
            )

            if self.min_points_per_voxel > 1:
                valid_voxel_mask = counts >= self.min_points_per_voxel
                valid_point_mask = valid_voxel_mask[inverse_indices]
                points = points[valid_point_mask]
                voxel_indices = voxel_indices[valid_point_mask]

                if points.shape[0] == 0:
                    return

                unique_voxels, inverse_indices = np.unique(
                    voxel_indices,
                    axis=0,
                    return_inverse=True,
                )

            sums = np.zeros((unique_voxels.shape[0], 3), dtype=np.float64)
            np.add.at(sums, inverse_indices, points)
            voxel_counts = np.bincount(inverse_indices)
            filtered_points = (sums / voxel_counts[:, None]).astype(np.float32)

            if filtered_points.shape[0] == 0:
                return

            header = Header()
            header.frame_id = msg.header.frame_id
            header.stamp = self.get_clock().now().to_msg()

            cloud_msg = pc2.create_cloud_xyz32(header, filtered_points.tolist())
            self.publisher.publish(cloud_msg)

            self.callback_count += 1
            elapsed_ms = (time.time() - start) * 1000.0
            now = time.time()
            if now - self.last_log_time > 2.0:
                self.get_logger().info(
                    f'Processed {self.callback_count} clouds | '
                    f'latency={elapsed_ms:.2f}ms | '
                    f'points {len(points_list)} -> {len(filtered_points)}'
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
