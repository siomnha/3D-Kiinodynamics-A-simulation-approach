import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import math


class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')

        self.waypoints = []
        self.current_index = 0
        self.current_pose = None

        self.create_subscription(
             Path,
             '/waypoints',
             self.waypoints_callback,
             10
        )

        # Subscribe to robot pose
        self.create_subscription(
            PoseStamped,
            '/space_cobot/pose',
            self.pose_callback,
            10
        )

        # Publish goals to Nav6D
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/nav6d/goal',
            10
        )

        # Timer to check arrival
        self.timer = self.create_timer(0.5, self.check_progress)

        self.get_logger().info("Mission manager started.")

    def waypoints_callback(self, msg):
        if len(msg.poses) == 0:
            return

        self.waypoints = [pose.pose for pose in msg.poses]
        self.current_index = 0
        self.get_logger().info(f"Received {len(self.waypoints)} waypoints.")
        self.send_next_goal()

    def pose_callback(self, msg):
        self.current_pose = msg.pose

    def send_next_goal(self):
        if self.current_index >= len(self.waypoints):
            self.get_logger().info("Mission complete.")
            return

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose = self.waypoints[self.current_index]

        self.goal_pub.publish(goal)

        self.get_logger().info(f"Sending waypoint {self.current_index}")

    def check_progress(self):
        if self.current_pose is None:
            return

        if self.current_index >= len(self.waypoints):
            return

        wp = self.waypoints[self.current_index]

        dx = wp.position.x - self.current_pose.position.x
        dy = wp.position.y - self.current_pose.position.y
        dz = wp.position.z - self.current_pose.position.z

        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        # Threshold distance to consider waypoint reached
        if dist < 0.5:
            self.get_logger().info(f"Reached waypoint {self.current_index}")
            self.current_index += 1
            self.send_next_goal()


def main():
    rclpy.init()
    node = MissionManager()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

