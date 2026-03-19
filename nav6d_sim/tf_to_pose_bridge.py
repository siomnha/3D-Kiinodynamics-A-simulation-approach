from typing import Optional

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node


class TfToPoseBridge(Node):
    """Bridge TF transform (parent->child) to PoseStamped topic for planner/mission nodes."""

    def __init__(self) -> None:
        super().__init__('tf_to_pose_bridge')

        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('pose_topic', '/space_cobot/pose')
        self.declare_parameter('publish_period_s', 0.05)  # match nav_6d_optimize_traj timer
        self.declare_parameter('lookup_timeout_s', 0.05)

        self.parent_frame = str(self.get_parameter('parent_frame').value)
        self.child_frame = str(self.get_parameter('child_frame').value)
        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.publish_period_s = float(self.get_parameter('publish_period_s').value)
        self.lookup_timeout_s = float(self.get_parameter('lookup_timeout_s').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)

        self.timer = self.create_timer(max(0.01, self.publish_period_s), self.timer_cb)
        self.warn_throttle: Optional[rclpy.time.Time] = None

        self.get_logger().info(
            f'tf_to_pose_bridge started: {self.parent_frame}->{self.child_frame} -> {self.pose_topic} '
            f'@ {1.0/max(0.01, self.publish_period_s):.1f} Hz'
        )

    def timer_cb(self) -> None:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.child_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.lookup_timeout_s),
            )
        except Exception:
            now = self.get_clock().now()
            if self.warn_throttle is None or (now - self.warn_throttle).nanoseconds > int(2e9):
                self.get_logger().warn(
                    f'No TF available yet for {self.parent_frame}->{self.child_frame}; waiting...'
                )
                self.warn_throttle = now
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.parent_frame
        msg.pose.position.x = tf.transform.translation.x
        msg.pose.position.y = tf.transform.translation.y
        msg.pose.position.z = tf.transform.translation.z
        msg.pose.orientation = tf.transform.rotation
        self.pose_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = TfToPoseBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
