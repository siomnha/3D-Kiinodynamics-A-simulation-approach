import math
import time
from typing import Optional, Tuple

import rclpy
import tf2_ros
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from octomap_msgs.msg import Octomap
from rclpy.node import Node
from std_msgs.msg import Bool, String


class ReplanMonitor(Node):
    """Monitor map/progress/TF conditions and emit replan requests (observe-first style)."""

    def __init__(self) -> None:
        super().__init__('replan_monitor')

        self.declare_parameter('octomap_topic', '/octomap_full')
        self.declare_parameter('trajectory_topic', '/trajectory/reference')
        self.declare_parameter('pose_topic', '/space_cobot/pose')
        self.declare_parameter('replan_request_topic', '/planning/replan_request')
        self.declare_parameter('replan_reason_topic', '/planning/replan_reason')

        self.declare_parameter('check_period_s', 0.5)
        self.declare_parameter('replan_rate_limit_s', 2.0)
        self.declare_parameter('map_stale_timeout_s', 4.0)
        self.declare_parameter('stuck_time_s', 5.0)
        self.declare_parameter('stuck_distance_m', 0.4)

        self.declare_parameter('tf_check_enable', True)
        self.declare_parameter('tf_parent_frame', 'map')
        self.declare_parameter('tf_child_frame', 'base_link')

        self.octomap_topic = str(self.get_parameter('octomap_topic').value)
        self.trajectory_topic = str(self.get_parameter('trajectory_topic').value)
        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.replan_request_topic = str(self.get_parameter('replan_request_topic').value)
        self.replan_reason_topic = str(self.get_parameter('replan_reason_topic').value)

        self.check_period_s = float(self.get_parameter('check_period_s').value)
        self.replan_rate_limit_s = float(self.get_parameter('replan_rate_limit_s').value)
        self.map_stale_timeout_s = float(self.get_parameter('map_stale_timeout_s').value)
        self.stuck_time_s = float(self.get_parameter('stuck_time_s').value)
        self.stuck_distance_m = float(self.get_parameter('stuck_distance_m').value)

        self.tf_check_enable = bool(self.get_parameter('tf_check_enable').value)
        self.tf_parent_frame = str(self.get_parameter('tf_parent_frame').value)
        self.tf_child_frame = str(self.get_parameter('tf_child_frame').value)

        self.replan_pub = self.create_publisher(Bool, self.replan_request_topic, 10)
        self.reason_pub = self.create_publisher(String, self.replan_reason_topic, 10)

        self.latest_map_time = 0.0
        self.latest_map_crc = -1
        self.latest_map_changed_time = 0.0

        self.latest_pose: Optional[PoseStamped] = None
        self.last_progress_pose: Optional[Tuple[float, float, float]] = None
        self.last_progress_time = time.monotonic()

        self.latest_reference: Optional[Path] = None
        self.last_replan_emit_time = 0.0

        self.create_subscription(Octomap, self.octomap_topic, self.map_cb, 10)
        self.create_subscription(Path, self.trajectory_topic, self.reference_cb, 10)
        self.create_subscription(PoseStamped, self.pose_topic, self.pose_cb, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(max(0.2, self.check_period_s), self.check_cb)
        self.get_logger().info('replan_monitor active (observe/trigger mode).')

    def map_cb(self, msg: Octomap) -> None:
        self.latest_map_time = time.monotonic()
        data_bytes = bytes((x + 256) % 256 for x in msg.data)
        crc = (sum(data_bytes) + len(data_bytes)) & 0xFFFFFFFF
        if crc != self.latest_map_crc:
            self.latest_map_crc = crc
            self.latest_map_changed_time = self.latest_map_time

    def reference_cb(self, msg: Path) -> None:
        self.latest_reference = msg

    def pose_cb(self, msg: PoseStamped) -> None:
        self.latest_pose = msg

    def check_cb(self) -> None:
        now = time.monotonic()

        if self.latest_reference is None or not self.latest_reference.poses:
            return

        if now - self.last_replan_emit_time < self.replan_rate_limit_s:
            return

        reason = self.detect_reason(now)
        if reason is None:
            return

        self.emit_replan(reason)
        self.last_replan_emit_time = now

    def detect_reason(self, now: float) -> Optional[str]:
        if self.latest_map_time <= 0.0:
            return 'MAP_UNAVAILABLE'

        if now - self.latest_map_time > self.map_stale_timeout_s:
            return 'MAP_STALE'

        # Trigger on map change while trajectory exists (conservative observe-first logic)
        if now - self.latest_map_changed_time <= self.check_period_s * 1.5:
            return 'MAP_CHANGED_NEAR_TRAJECTORY'

        if self.progress_is_stuck(now):
            return 'PROGRESS_STUCK'

        if self.tf_check_enable and not self.tf_ok():
            return 'TF_DEGRADED'

        return None

    def progress_is_stuck(self, now: float) -> bool:
        if self.latest_pose is None:
            return False

        p = self.latest_pose.pose.position
        current = (p.x, p.y, p.z)
        if self.last_progress_pose is None:
            self.last_progress_pose = current
            self.last_progress_time = now
            return False

        moved = self.distance(current, self.last_progress_pose)
        if moved >= self.stuck_distance_m:
            self.last_progress_pose = current
            self.last_progress_time = now
            return False

        return (now - self.last_progress_time) >= self.stuck_time_s

    def tf_ok(self) -> bool:
        try:
            self.tf_buffer.lookup_transform(
                self.tf_parent_frame,
                self.tf_child_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.15),
            )
            return True
        except Exception:
            return False

    def emit_replan(self, reason: str) -> None:
        req = Bool()
        req.data = True
        self.replan_pub.publish(req)

        why = String()
        why.data = reason
        self.reason_pub.publish(why)
        self.get_logger().warn(f'Replan requested: {reason}')

    @staticmethod
    def distance(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def main() -> None:
    rclpy.init()
    node = ReplanMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
