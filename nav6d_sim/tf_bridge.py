import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
from geometry_msgs.msg import TransformStamped


class TFBridge(Node):

    def __init__(self):
        super().__init__('tf_bridge')

        self.br = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(
            PoseStamped,
            '/space_cobot/pose',
            self.pose_callback,
            10)

    def pose_callback(self, msg):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        t.transform.rotation = msg.pose.orientation

        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = TFBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

