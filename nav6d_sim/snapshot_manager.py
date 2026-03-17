import os
import subprocess
import time
import zlib
from collections import deque
from pathlib import Path
from typing import Deque

import rclpy
from octomap_msgs.msg import Octomap
from rclpy.node import Node
from std_msgs.msg import String


class SnapshotManager(Node):
    """Periodically save OctoMap snapshots to .bt for snapshot-based planning."""

    def __init__(self) -> None:
        super().__init__('snapshot_manager')

        self.declare_parameter('octomap_topic', '/octomap_binary')
        self.declare_parameter('snapshot_period_s', 5.0)
        self.declare_parameter('min_snapshot_interval_s', 2.0)
        self.declare_parameter('output_dir', '/tmp/octomap_snapshots')
        self.declare_parameter('snapshot_prefix', 'map_snapshot')
        self.declare_parameter('mode', 'overwrite')  # overwrite | rolling
        self.declare_parameter('max_snapshots', 5)
        self.declare_parameter('save_on_change_only', True)
        self.declare_parameter('saver_timeout_s', 8.0)
        self.declare_parameter('reload_event_topic', '/planning/map_reload')

        self.octomap_topic = str(self.get_parameter('octomap_topic').value)
        self.snapshot_period_s = float(self.get_parameter('snapshot_period_s').value)
        self.min_snapshot_interval_s = float(self.get_parameter('min_snapshot_interval_s').value)
        self.output_dir = Path(str(self.get_parameter('output_dir').value)).expanduser()
        self.snapshot_prefix = str(self.get_parameter('snapshot_prefix').value)
        self.mode = str(self.get_parameter('mode').value)
        self.max_snapshots = int(self.get_parameter('max_snapshots').value)
        self.save_on_change_only = bool(self.get_parameter('save_on_change_only').value)
        self.saver_timeout_s = float(self.get_parameter('saver_timeout_s').value)
        self.reload_event_topic = str(self.get_parameter('reload_event_topic').value)

        if self.mode not in {'overwrite', 'rolling'}:
            raise RuntimeError("mode must be 'overwrite' or 'rolling'")

        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.last_crc: int = -1
        self.last_snapshot_monotonic = 0.0
        self.latest_msg_time = 0.0
        self.counter = 0
        self.snapshot_paths: Deque[Path] = deque(maxlen=max(1, self.max_snapshots))

        self.reload_pub = self.create_publisher(String, self.reload_event_topic, 10)

        self.create_subscription(Octomap, self.octomap_topic, self.octomap_callback, 10)
        self.timer = self.create_timer(max(0.2, self.snapshot_period_s), self.snapshot_timer_cb)

        self.get_logger().info(
            f'snapshot_manager active: topic={self.octomap_topic}, mode={self.mode}, output_dir={self.output_dir}'
        )

    def octomap_callback(self, msg: Octomap) -> None:
        self.latest_msg_time = time.monotonic()
        data_bytes = bytes((x + 256) % 256 for x in msg.data)
        self.current_crc = zlib.crc32(data_bytes)

    def snapshot_timer_cb(self) -> None:
        now = time.monotonic()
        if self.latest_msg_time <= 0.0:
            self.get_logger().warn('No OctoMap message received yet; skip snapshot.')
            return

        if now - self.last_snapshot_monotonic < self.min_snapshot_interval_s:
            return

        current_crc = getattr(self, 'current_crc', -1)
        if self.save_on_change_only and current_crc == self.last_crc:
            self.get_logger().debug('OctoMap unchanged; skip snapshot.')
            return

        snapshot_path = self.next_snapshot_path()
        if self.run_octomap_saver(snapshot_path):
            self.last_snapshot_monotonic = now
            self.last_crc = current_crc
            self.snapshot_paths.append(snapshot_path)
            self.prune_old_snapshots()
            self.publish_reload_event(snapshot_path)

    def next_snapshot_path(self) -> Path:
        if self.mode == 'overwrite':
            return self.output_dir / f'{self.snapshot_prefix}.bt'
        self.counter += 1
        return self.output_dir / f'{self.snapshot_prefix}_{self.counter:05d}.bt'

    def run_octomap_saver(self, path: Path) -> bool:
        saver_cmds = [
            [
                'ros2', 'run', 'octomap_server', 'octomap_saver_node',
                '--ros-args', '-p', f'octomap_path:={path}', '-r', f'octomap_binary:={self.octomap_topic}'
            ],
            [
                'ros2', 'run', 'octomap_server', 'octomap_saver',
                '-f', str(path), '--ros-args', '-r', f'octomap_binary:={self.octomap_topic}'
            ],
        ]
        self.get_logger().info(f'Saving snapshot -> {path}')
        for cmd in saver_cmds:
            try:
                subprocess.run(cmd, check=True, timeout=self.saver_timeout_s)
                return True
            except subprocess.TimeoutExpired:
                self.get_logger().error('octomap saver timed out.')
                return False
            except subprocess.CalledProcessError:
                continue
        self.get_logger().error('No working octomap saver executable found (tried octomap_saver_node, octomap_saver).')


    def prune_old_snapshots(self) -> None:
        if self.mode != 'rolling' or self.max_snapshots <= 0:
            return

        files = sorted(self.output_dir.glob(f'{self.snapshot_prefix}_*.bt'))
        if len(files) <= self.max_snapshots:
            return

        to_remove = files[: len(files) - self.max_snapshots]
        for f in to_remove:
            try:
                os.remove(f)
            except OSError as exc:
                self.get_logger().warn(f'Failed to remove old snapshot {f}: {exc}')

    def publish_reload_event(self, snapshot_path: Path) -> None:
        msg = String()
        msg.data = str(snapshot_path)
        self.reload_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = SnapshotManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
