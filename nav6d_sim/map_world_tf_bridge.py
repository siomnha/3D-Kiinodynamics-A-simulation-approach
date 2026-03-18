import math

import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node


class MapWorldTfBridge(Node):
    """Publish static transform map->world (or configurable frames) for frame unification."""

    def __init__(self) -> None:
        super().__init__('map_world_tf_bridge')

        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'world')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('roll', 0.0)
        self.declare_parameter('pitch', 0.0)
        self.declare_parameter('yaw', 0.0)

        parent = str(self.get_parameter('parent_frame').value)
        child = str(self.get_parameter('child_frame').value)
        x = float(self.get_parameter('x').value)
        y = float(self.get_parameter('y').value)
        z = float(self.get_parameter('z').value)
        roll = float(self.get_parameter('roll').value)
        pitch = float(self.get_parameter('pitch').value)
        yaw = float(self.get_parameter('yaw').value)

        br = tf2_ros.StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        qx, qy, qz, qw = self.quat_from_rpy(roll, pitch, yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        br.sendTransform(t)
        self.get_logger().info(
            f'Published static TF {parent}->{child}: xyz=({x:.3f},{y:.3f},{z:.3f}) rpy=({roll:.3f},{pitch:.3f},{yaw:.3f})'
        )

    @staticmethod
    def quat_from_rpy(roll: float, pitch: float, yaw: float):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw


def main() -> None:
    rclpy.init()
    node = MapWorldTfBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
