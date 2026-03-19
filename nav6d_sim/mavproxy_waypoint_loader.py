import math
from pathlib import Path as FsPath
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

Point3 = Tuple[float, float, float]


class MavproxyWaypointLoader(Node):
    """Load waypoints from a MAVProxy/QGC txt file and publish /waypoints as nav_msgs/Path."""

    def __init__(self) -> None:
        super().__init__('mavproxy_waypoint_loader')

        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_period_s', 1.0)
        self.declare_parameter('origin_lat', 0.0)
        self.declare_parameter('origin_lon', 0.0)
        self.declare_parameter('origin_alt', 0.0)
        self.declare_parameter('default_takeoff_alt', 5.0)

        self.waypoint_file = str(self.get_parameter('waypoint_file').value).strip()
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.origin_lat = float(self.get_parameter('origin_lat').value)
        self.origin_lon = float(self.get_parameter('origin_lon').value)
        self.origin_alt = float(self.get_parameter('origin_alt').value)
        self.default_takeoff_alt = float(self.get_parameter('default_takeoff_alt').value)
        period = float(self.get_parameter('publish_period_s').value)

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '/waypoints', qos)

        self.path_msg = self.load_waypoints(self.waypoint_file)
        self.publish_path()
        self.timer = self.create_timer(max(0.2, period), self.publish_path)

        self.get_logger().info(
            f'Loaded {len(self.path_msg.poses)} waypoint(s) from {self.waypoint_file}; publishing /waypoints in {self.frame_id}'
        )

    def publish_path(self) -> None:
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path_msg)

    def load_waypoints(self, file_path: str) -> Path:
        if not file_path:
            raise RuntimeError('waypoint_file is empty. Provide --ros-args -p waypoint_file:=/abs/path/to/waypoints.txt')
        path = FsPath(file_path).expanduser()
        if not path.exists():
            raise RuntimeError(f'Waypoint file not found: {path}')

        lines = [ln.strip() for ln in path.read_text(encoding='utf-8').splitlines() if ln.strip() and not ln.strip().startswith('#')]
        points = self.parse_lines(lines)

        msg = Path()
        msg.header.frame_id = self.frame_id
        msg.poses = [self.to_pose_stamped(p) for p in points]
        return msg

    def parse_lines(self, lines: List[str]) -> List[Point3]:
        if not lines:
            raise RuntimeError('Waypoint file is empty.')

        if lines[0].startswith('QGC WPL'):
            points = self.parse_qgc_wpl(lines[1:])
        else:
            points = self.parse_xyz_lines(lines)

        if not points:
            raise RuntimeError('No valid waypoint parsed from file.')
        return points

    def parse_qgc_wpl(self, lines: List[str]) -> List[Point3]:
        points: List[Point3] = []
        for line in lines:
            tokens = line.split()
            if len(tokens) < 12:
                continue

            command = int(float(tokens[3]))
            lat = float(tokens[8])
            lon = float(tokens[9])
            alt = float(tokens[10])

            # MAV_CMD_NAV_WAYPOINT=16, MAV_CMD_NAV_TAKEOFF=22
            if command == 22 and abs(alt) < 1e-3:
                alt = self.default_takeoff_alt

            if not (abs(lat) > 1e-8 and abs(lon) > 1e-8):
                # Skip non-positional commands (e.g. home row often 0,0)
                continue

            points.append(self.llh_to_local_enu(lat, lon, alt))

        return points

    def parse_xyz_lines(self, lines: List[str]) -> List[Point3]:
        points: List[Point3] = []
        for line in lines:
            tokens = line.replace(',', ' ').split()
            if len(tokens) < 3:
                continue
            x, y, z = float(tokens[0]), float(tokens[1]), float(tokens[2])
            points.append((x, y, z))
        return points

    def llh_to_local_enu(self, lat_deg: float, lon_deg: float, alt_m: float) -> Point3:
        if abs(self.origin_lat) < 1e-9 and abs(self.origin_lon) < 1e-9:
            raise RuntimeError(
                'QGC WPL detected but origin_lat/origin_lon are not set. '
                'Provide local map origin via --ros-args -p origin_lat:=.. -p origin_lon:=..'
            )

        r_earth = 6378137.0
        d_lat = math.radians(lat_deg - self.origin_lat)
        d_lon = math.radians(lon_deg - self.origin_lon)
        lat0 = math.radians(self.origin_lat)

        east = r_earth * d_lon * math.cos(lat0)
        north = r_earth * d_lat
        up = alt_m - self.origin_alt
        return (east, north, up)

    def to_pose_stamped(self, point: Point3) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = float(point[0])
        pose.pose.position.y = float(point[1])
        pose.pose.position.z = float(point[2])
        pose.pose.orientation.w = 1.0
        return pose


def main() -> None:
    rclpy.init()
    node: Optional[MavproxyWaypointLoader] = None
    try:
        node = MavproxyWaypointLoader()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
