import math
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node


class ArdupilotMavrosSetpointBridge(Node):
    """Bridge planned trajectory topics to MAVROS local position setpoints."""

    def __init__(self) -> None:
        super().__init__('ardupilot_mavros_setpoint_bridge')

        self.declare_parameter('mode', 'state')  # state | path
        self.declare_parameter('state_topic', '/trajectory/state')
        self.declare_parameter('path_topic', '/trajectory/reference')
        self.declare_parameter('vehicle_pose_topic', '/space_cobot/pose')
        self.declare_parameter('setpoint_topic', '/mavros/setpoint_position/local')

        self.declare_parameter('publish_period_s', 0.05)
        self.declare_parameter('arrival_radius_m', 0.4)
        self.declare_parameter('lookahead_index', 3)
        self.declare_parameter('hold_last', True)

        self.mode = str(self.get_parameter('mode').value)
        self.state_topic = str(self.get_parameter('state_topic').value)
        self.path_topic = str(self.get_parameter('path_topic').value)
        self.vehicle_pose_topic = str(self.get_parameter('vehicle_pose_topic').value)
        self.setpoint_topic = str(self.get_parameter('setpoint_topic').value)

        self.publish_period_s = float(self.get_parameter('publish_period_s').value)
        self.arrival_radius_m = float(self.get_parameter('arrival_radius_m').value)
        self.lookahead_index = int(self.get_parameter('lookahead_index').value)
        self.hold_last = bool(self.get_parameter('hold_last').value)

        if self.mode not in {'state', 'path'}:
            raise RuntimeError("mode must be 'state' or 'path'")

        self.setpoint_pub = self.create_publisher(PoseStamped, self.setpoint_topic, 10)

        self.latest_state: Optional[PoseStamped] = None
        self.latest_pose: Optional[PoseStamped] = None
        self.path: List[PoseStamped] = []
        self.path_index = 0

        self.create_subscription(PoseStamped, self.state_topic, self.state_cb, 10)
        self.create_subscription(Path, self.path_topic, self.path_cb, 10)
        self.create_subscription(PoseStamped, self.vehicle_pose_topic, self.vehicle_pose_cb, 10)

        self.timer = self.create_timer(max(0.01, self.publish_period_s), self.timer_cb)

        self.get_logger().info(
            f'ardupilot_mavros_setpoint_bridge started: mode={self.mode}, '
            f'setpoint_topic={self.setpoint_topic}, period={self.publish_period_s:.3f}s'
        )
        self.get_logger().info(
            'This node is a setpoint bridge (interface layer), not a full GNC control-law implementation.'
        )

    def state_cb(self, msg: PoseStamped) -> None:
        self.latest_state = msg

    def path_cb(self, msg: Path) -> None:
        if not msg.poses:
            return
        self.path = msg.poses
        self.path_index = 0
        self.get_logger().info(f'Received trajectory path with {len(self.path)} poses.')

    def vehicle_pose_cb(self, msg: PoseStamped) -> None:
        self.latest_pose = msg

    def timer_cb(self) -> None:
        target = self.select_target()
        if target is None:
            return

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = target.header.frame_id if target.header.frame_id else 'map'
        out.pose = target.pose
        self.setpoint_pub.publish(out)

    def select_target(self) -> Optional[PoseStamped]:
        if self.mode == 'state':
            return self.latest_state

        # path mode
        if not self.path:
            return None

        if self.latest_pose is not None:
            while self.path_index < len(self.path):
                d = self.distance(self.latest_pose, self.path[self.path_index])
                if d > self.arrival_radius_m:
                    break
                self.path_index += 1

        idx = self.path_index + max(0, self.lookahead_index)
        if idx >= len(self.path):
            if not self.hold_last:
                return None
            idx = len(self.path) - 1

        return self.path[idx]

    @staticmethod
    def distance(a: PoseStamped, b: PoseStamped) -> float:
        dx = a.pose.position.x - b.pose.position.x
        dy = a.pose.position.y - b.pose.position.y
        dz = a.pose.position.z - b.pose.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)


def main() -> None:
    rclpy.init()
    node = ArdupilotMavrosSetpointBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
