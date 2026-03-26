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

"""JS relay, odom TF."""
from __future__ import annotations

import copy

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


def _stamp_or_now(node: Node, sec: int, nanosec: int) -> TimeMsg:
    if sec == 0 and nanosec == 0:
        return node.get_clock().now().to_msg()
    return TimeMsg(sec=sec, nanosec=nanosec)


def _quat_rotate_vec(
    qx: float, qy: float, qz: float, qw: float, vx: float, vy: float, vz: float
):
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    wx = qw * tx + qy * tz - qz * ty
    wy = qw * ty + qz * tx - qx * tz
    wz = qw * tz + qx * ty - qy * tx
    return (vx + wx, vy + wy, vz + wz)


_WHEEL_RADIUS = 0.165


class SimTfRelay(Node):
    """GZ js + TF."""

    def __init__(self) -> None:
        super().__init__('sim_tf_relay')
        self.declare_parameter('tf_child_frame', 'base_footprint')
        self.declare_parameter('baselink_z_offset', _WHEEL_RADIUS)
        pv = self.get_parameter('tf_child_frame').get_parameter_value()
        self._tf_child = pv.string_value
        zv = self.get_parameter('baselink_z_offset').get_parameter_value()
        self._z_off = float(zv.double_value)

        qos_be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._joint_pub = self.create_publisher(JointState, 'joint_states', qos_be)
        self.create_subscription(JointState, 'joint_states_gz', self._joint_cb, qos_be)
        self._first_valid_js = False  # skip t=0

        self._br = TransformBroadcaster(self)
        odom_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, 'odom', self._odom_cb, odom_qos)

    def _joint_cb(self, msg: JointState) -> None:
        h = msg.header
        if not self._first_valid_js:
            if h.stamp.sec == 0 and h.stamp.nanosec == 0:
                return  # stale stamp
            self._first_valid_js = True
        out = copy.deepcopy(msg)
        out.header.stamp = _stamp_or_now(self, h.stamp.sec, h.stamp.nanosec)
        self._joint_pub.publish(out)

    def _odom_cb(self, msg: Odometry) -> None:
        t = TransformStamped()
        h = msg.header
        t.header.stamp = _stamp_or_now(self, h.stamp.sec, h.stamp.nanosec)
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = self._tf_child
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        if msg.child_frame_id == 'base_link' and self._tf_child == 'base_footprint':
            dx, dy, dz = _quat_rotate_vec(qx, qy, qz, qw, 0.0, 0.0, -self._z_off)
            t.transform.translation.x = p.x + dx
            t.transform.translation.y = p.y + dy
            t.transform.translation.z = p.z + dz
        else:
            t.transform.translation.x = p.x
            t.transform.translation.y = p.y
            t.transform.translation.z = p.z
        t.transform.rotation = q
        self._br.sendTransform(t)


def main() -> None:
    rclpy.init()
    node = SimTfRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
