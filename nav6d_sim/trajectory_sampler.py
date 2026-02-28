import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node


class TrajectorySampler(Node):
    """Sample trajectory reference for RViz and downstream control integration."""

    def __init__(self) -> None:
        super().__init__('trajectory_sampler')

        self.declare_parameter('input_topic', '/trajectory/reference')
        self.declare_parameter('pose_topic', '/space_cobot/pose')
        self.declare_parameter('state_topic', '/trajectory/state')
        self.declare_parameter('timer_period', 0.05)
        self.declare_parameter('loop', False)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        state_topic = self.get_parameter('state_topic').get_parameter_value().string_value
        period = self.get_parameter('timer_period').get_parameter_value().double_value
        self.loop = self.get_parameter('loop').get_parameter_value().bool_value

        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        self.state_pub = self.create_publisher(PoseStamped, state_topic, 10)

        self.path = []
        self.index = 0

        self.create_subscription(Path, input_topic, self.path_callback, 10)
        self.timer = self.create_timer(period, self.step)

        self.get_logger().info(
            f'Trajectory sampler started. {input_topic} -> ({pose_topic}, {state_topic})'
        )

    def path_callback(self, msg: Path) -> None:
        if not msg.poses:
            return

        self.path = msg.poses
        self.index = 0
        self.get_logger().info(f'Loaded trajectory with {len(self.path)} points.')

    def step(self) -> None:
        if not self.path:
            return

        if self.index >= len(self.path):
            if self.loop:
                self.index = 0
            else:
                return

        src = self.path[self.index]

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = src.header.frame_id if src.header.frame_id else 'map'
        pose.pose = src.pose

        self.pose_pub.publish(pose)
        self.state_pub.publish(pose)
        self.index += 1


def main() -> None:
    rclpy.init()
    node = TrajectorySampler()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
