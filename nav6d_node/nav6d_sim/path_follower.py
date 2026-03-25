import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class PathFollower(Node):

    def __init__(self):
        super().__init__('path_follower')

        self.path = []
        self.index = 0

        # Subscribe to planned path
        self.create_subscription(
            Path,
            '/nav6d/planner/path',
            self.path_callback,
            10
        )

        # Publish simulated pose
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/space_cobot/pose',
            10
        )

        # Timer for movement
        self.timer = self.create_timer(0.1, self.move_step)

        self.get_logger().info("Path follower started.")

    def path_callback(self, msg):
        if len(msg.poses) == 0:
            return

        self.path = msg.poses
        self.index = 0
        self.get_logger().info(f"New path received with {len(self.path)} points.")

    def move_step(self):
        if not self.path:
            return

        if self.index >= len(self.path):
            return

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.pose = self.path[self.index].pose

        self.pose_pub.publish(pose_msg)

        self.index += 1


def main():
    rclpy.init()
    node = PathFollower()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

