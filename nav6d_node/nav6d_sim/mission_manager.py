import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node


class MissionManager(Node):
    """Forward mission waypoints to nav6d sequential goals for simulation and real flights."""

    def __init__(self) -> None:
        super().__init__('mission_manager')

        self.declare_parameter('waypoints_topic', '/waypoints')
        self.declare_parameter('mavros_waypoints_topic', '/mavros/mission/waypoints')
        self.declare_parameter('pose_topic', '/space_cobot/pose')
        self.declare_parameter('goal_topic', '/nav6d/goal')
        self.declare_parameter('goal_frame_id', 'map')
        self.declare_parameter('arrival_threshold_m', 0.6)
        self.declare_parameter('progress_check_period_s', 0.25)
        self.declare_parameter('continuous_mission_mode', False)
        self.declare_parameter('continuous_path_topic', '/mission_manager/continuous_path')

        # Local-map origin used when MAVROS waypoints are global (lat/lon/alt)
        self.declare_parameter('origin_lat', 0.0)
        self.declare_parameter('origin_lon', 0.0)
        self.declare_parameter('origin_alt', 0.0)
        self.declare_parameter('default_waypoint_alt_m', 1.5)
        self.declare_parameter('mavros_waypoints_are_global', True)

        self.waypoints_topic = str(self.get_parameter('waypoints_topic').value)
        self.mavros_waypoints_topic = str(self.get_parameter('mavros_waypoints_topic').value)
        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.goal_topic = str(self.get_parameter('goal_topic').value)
        self.goal_frame_id = str(self.get_parameter('goal_frame_id').value)
        self.arrival_threshold_m = float(self.get_parameter('arrival_threshold_m').value)
        self.progress_check_period_s = float(self.get_parameter('progress_check_period_s').value)
        self.continuous_mission_mode = bool(self.get_parameter('continuous_mission_mode').value)
        self.continuous_path_topic = str(self.get_parameter('continuous_path_topic').value)

        self.origin_lat = float(self.get_parameter('origin_lat').value)
        self.origin_lon = float(self.get_parameter('origin_lon').value)
        self.origin_alt = float(self.get_parameter('origin_alt').value)
        self.default_waypoint_alt_m = float(self.get_parameter('default_waypoint_alt_m').value)
        self.mavros_waypoints_are_global = bool(self.get_parameter('mavros_waypoints_are_global').value)

        self.waypoints: List[Pose] = []
        self.current_index = 0
        self.current_pose: Optional[Pose] = None

        # Generic path waypoints (RViz plugin, mavproxy_waypoint_loader, custom publishers)
        self.create_subscription(Path, self.waypoints_topic, self.waypoints_callback, 10)

        # Optional MAVROS mission subscription
        self._mavros_enabled = self.init_mavros_subscription()

        self.create_subscription(PoseStamped, self.pose_topic, self.pose_callback, 10)
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.continuous_path_pub = self.create_publisher(Path, self.continuous_path_topic, 10)
        self.timer = self.create_timer(max(0.05, self.progress_check_period_s), self.check_progress)

        self.get_logger().info(
            f'Mission manager ready. path={self.waypoints_topic}, '
            f'mavros={self.mavros_waypoints_topic if self._mavros_enabled else "disabled"}, '
            f'pose={self.pose_topic}, goal={self.goal_topic}, frame={self.goal_frame_id}, '
            f'continuous_mode={self.continuous_mission_mode}'
        )

    def init_mavros_subscription(self) -> bool:
        try:
            from mavros_msgs.msg import WaypointList  # type: ignore
        except ImportError:
            self.get_logger().warn(
                'mavros_msgs not available; MAVROS mission input disabled. '\
                'Install mavros_msgs or publish nav_msgs/Path on /waypoints.'
            )
            return False

        self.create_subscription(WaypointList, self.mavros_waypoints_topic, self.mavros_waypoints_callback, 10)
        return True

    def waypoints_callback(self, msg: Path) -> None:
        if not msg.poses:
            return
        self.waypoints = [pose.pose for pose in msg.poses]
        self.current_index = 0
        self.get_logger().info(f'Received {len(self.waypoints)} path waypoint(s) from {self.waypoints_topic}.')
        if self.continuous_mission_mode:
            self.publish_continuous_path()
            return
        self.send_next_goal()

    def mavros_waypoints_callback(self, msg) -> None:
        if not msg.waypoints:
            return

        converted: List[Pose] = []
        for wp in msg.waypoints:
            # Skip non-positional mission items (e.g., command-only rows).
            if int(wp.command) not in (16, 22):
                continue

            pose = Pose()
            if self.mavros_waypoints_are_global:
                local = self.llh_to_local_enu(float(wp.x_lat), float(wp.y_long), float(wp.z_alt))
                if local is None:
                    continue
                pose.position.x, pose.position.y, pose.position.z = local
            else:
                pose.position.x = float(wp.x_lat)
                pose.position.y = float(wp.y_long)
                pose.position.z = float(wp.z_alt)

            if abs(pose.position.z) < 1e-3:
                pose.position.z = self.default_waypoint_alt_m
            pose.orientation.w = 1.0
            converted.append(pose)

        if not converted:
            self.get_logger().warn('MAVROS mission received but no usable waypoint commands were parsed.')
            return

        self.waypoints = converted
        self.current_index = 0
        self.get_logger().info(
            f'Received {len(self.waypoints)} MAVROS waypoint(s) from {self.mavros_waypoints_topic}.'
        )
        if self.continuous_mission_mode:
            self.publish_continuous_path()
            return
        self.send_next_goal()

    def llh_to_local_enu(self, lat_deg: float, lon_deg: float, alt_m: float) -> Optional[Tuple[float, float, float]]:
        if abs(self.origin_lat) < 1e-9 and abs(self.origin_lon) < 1e-9:
            self.get_logger().warn(
                'origin_lat/origin_lon are zero; cannot convert MAVROS global waypoints. '
                'Set origin params or set mavros_waypoints_are_global:=false.'
            )
            return None

        r_earth = 6378137.0
        d_lat = math.radians(lat_deg - self.origin_lat)
        d_lon = math.radians(lon_deg - self.origin_lon)
        lat0 = math.radians(self.origin_lat)

        east = r_earth * d_lon * math.cos(lat0)
        north = r_earth * d_lat
        up = alt_m - self.origin_alt
        return (east, north, up)


    def publish_continuous_path(self) -> None:
        if not self.waypoints:
            return

        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.goal_frame_id
        msg.poses = []

        for pose in self.waypoints:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = pose
            msg.poses.append(ps)

        self.continuous_path_pub.publish(msg)
        self.get_logger().info(
            f'Published continuous mission path with {len(msg.poses)} waypoint(s) on {self.continuous_path_topic}'
        )

    def pose_callback(self, msg: PoseStamped) -> None:
        self.current_pose = msg.pose

    def send_next_goal(self) -> None:
        if self.current_index >= len(self.waypoints):
            self.get_logger().info('Mission complete.')
            return

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.goal_frame_id
        goal.pose = self.waypoints[self.current_index]

        self.goal_pub.publish(goal)
        self.get_logger().info(f'Sending waypoint index={self.current_index}')

    def check_progress(self) -> None:
        if self.continuous_mission_mode:
            return
        if self.current_pose is None or self.current_index >= len(self.waypoints):
            return

        wp = self.waypoints[self.current_index]
        dx = wp.position.x - self.current_pose.position.x
        dy = wp.position.y - self.current_pose.position.y
        dz = wp.position.z - self.current_pose.position.z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist < self.arrival_threshold_m:
            self.get_logger().info(f'Reached waypoint index={self.current_index} (dist={dist:.2f} m)')
            self.current_index += 1
            self.send_next_goal()


def main() -> None:
    rclpy.init()
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
