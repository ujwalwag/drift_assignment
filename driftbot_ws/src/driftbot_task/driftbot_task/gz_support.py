# Copyright 2025 ujwalwag
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""GZ bridge support."""

from __future__ import annotations

import copy

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import TransformBroadcaster

_WHEEL_RADIUS = 0.165


def _stamp(node: Node, sec: int, nanosec: int) -> TimeMsg:
    if sec == 0 and nanosec == 0:
        return node.get_clock().now().to_msg()
    return TimeMsg(sec=sec, nanosec=nanosec)


def _rotate_vec_by_quat(
    qx: float, qy: float, qz: float, qw: float, vx: float, vy: float, vz: float
) -> tuple[float, float, float]:
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    wx = qw * tx + qy * tz - qz * ty
    wy = qw * ty + qz * tx - qx * tz
    wz = qw * tz + qx * ty - qy * tx
    return (vx + wx, vy + wy, vz + wz)


class GzBridgeSupport(Node):
    def __init__(self) -> None:
        super().__init__('gz_bridge_support')
        self.declare_parameter('tf_child_frame', 'base_footprint')
        self.declare_parameter('baselink_z_offset', _WHEEL_RADIUS)
        self.declare_parameter('scan_output_frame', 'lidar_link')

        self._child = str(self.get_parameter('tf_child_frame').value)
        zv = self.get_parameter('baselink_z_offset').value
        self._z_off = float(zv) if isinstance(zv, (int, float)) else float(str(zv))
        self._scan_frame = str(
            self.get_parameter('scan_output_frame').value or 'lidar_link'
        )

        qos_be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Joint relay
        self._js_pub = self.create_publisher(JointState, 'joint_states', qos_be)
        self.create_subscription(JointState, 'joint_states_gz', self._js_cb, qos_be)
        self._js_ok = False

        # Odom TF
        self._tf_b = TransformBroadcaster(self)
        odom_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, 'odom', self._odom_cb, odom_qos)

        # Scan relay
        self._scan_pub = self.create_publisher(LaserScan, 'scan', qos_be)
        self.create_subscription(LaserScan, 'scan_raw', self._scan_cb, qos_be)
        self._scan_n = 0
        self.create_timer(5.0, self._scan_warn)

        self.get_logger().info('gz_bridge_support up')

    def _js_cb(self, msg: JointState) -> None:
        h = msg.header
        if not self._js_ok:
            if h.stamp.sec == 0 and h.stamp.nanosec == 0:
                return
            self._js_ok = True
        out = copy.deepcopy(msg)
        out.header.stamp = _stamp(self, h.stamp.sec, h.stamp.nanosec)
        self._js_pub.publish(out)

    def _odom_cb(self, msg: Odometry) -> None:
        t = TransformStamped()
        h = msg.header
        t.header.stamp = _stamp(self, h.stamp.sec, h.stamp.nanosec)
        t.header.frame_id = h.frame_id
        t.child_frame_id = self._child
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        if msg.child_frame_id == 'base_link' and self._child == 'base_footprint':
            dx, dy, dz = _rotate_vec_by_quat(qx, qy, qz, qw, 0.0, 0.0, -self._z_off)
            t.transform.translation.x = p.x + dx
            t.transform.translation.y = p.y + dy
            t.transform.translation.z = p.z + dz
        else:
            t.transform.translation.x = p.x
            t.transform.translation.y = p.y
            t.transform.translation.z = p.z
        t.transform.rotation = q
        self._tf_b.sendTransform(t)

    def _scan_cb(self, msg: LaserScan) -> None:
        self._scan_n += 1
        st = msg.header.stamp
        if st.sec == 0 and st.nanosec == 0:
            st = self.get_clock().now().to_msg()
        out = copy.deepcopy(msg)
        out.header.frame_id = self._scan_frame
        out.header.stamp = st
        self._scan_pub.publish(out)

    def _scan_warn(self) -> None:
        if self._scan_n == 0:
            self.get_logger().warn('No /scan_raw received')


def main() -> None:
    rclpy.init()
    n = GzBridgeSupport()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            n.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
