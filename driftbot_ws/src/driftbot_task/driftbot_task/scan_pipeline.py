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

"""Scan relay + local grid."""
from __future__ import annotations

import copy
import math
from typing import List, Optional, Tuple

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener


# relay + cloud
class ScanRelay(Node):
    def __init__(self) -> None:
        super().__init__('scan_relay')
        self.declare_parameter('output_frame', 'lidar_link')
        frame = self.get_parameter('output_frame').get_parameter_value().string_value
        self._frame = frame or 'lidar_link'

        qos_be = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_rel = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(LaserScan, 'scan', qos_be)
        self._pub_pc = self.create_publisher(PointCloud2, 'scan_cloud_odom', qos_rel)
        self.create_subscription(LaserScan, 'scan_raw', self._cb, qos_be)

        self._tf_buf = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listen = TransformListener(self._tf_buf, self, spin_thread=True)

        self._rx = 0
        self._tf_cloud_ok = False
        self._logged_tf_wait = False
        self.create_timer(5.0, self._diag_cb)
        self.get_logger().info(
            f'/scan_raw -> /scan (frame={self._frame}, preserve sim stamp) + /scan_cloud_odom'
        )

    def _diag_cb(self) -> None:
        if self._rx == 0:
            self.get_logger().warn(
                'No LaserScan on /scan_raw yet (bridge gz /scan -> /scan_raw).'
            )

    def _cb(self, msg: LaserScan) -> None:
        self._rx += 1
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()

        out = copy.deepcopy(msg)
        out.header.frame_id = self._frame
        out.header.stamp = stamp
        self._pub.publish(out)

        try:
            tf_odom_lidar = self._tf_buf.lookup_transform(
                'odom',
                self._frame,
                Time.from_msg(stamp),
                timeout=Duration(seconds=0, nanoseconds=200_000_000),
            )
        except TransformException as ex:
            if not self._tf_cloud_ok:
                if not self._logged_tf_wait:
                    self.get_logger().info(
                        f'scan_cloud_odom: waiting for TF odom→{self._frame} '
                        '(normal until /clock and odom→base TF are up).'
                    )
                    self._logged_tf_wait = True
                return
            if self._rx % 50 == 1:
                self.get_logger().warn(
                    f'scan_cloud_odom: no TF odom←{self._frame}: {ex}'
                )
            return
        self._tf_cloud_ok = True

        pts: List[Tuple[float, float, float]] = []
        n = len(msg.ranges)
        if n == 0:
            return
        inc = msg.angle_increment
        for i in range(n):
            r = msg.ranges[i]
            if not (msg.range_min < r < msg.range_max):
                continue
            ang = msg.angle_min + i * inc
            ps = PointStamped()
            ps.header.frame_id = self._frame
            ps.header.stamp = stamp
            ps.point.x = float(r * math.cos(ang))
            ps.point.y = float(r * math.sin(ang))
            ps.point.z = 0.0
            try:
                pod = do_transform_point(ps, tf_odom_lidar)
                pts.append((pod.point.x, pod.point.y, pod.point.z))
            except Exception:
                continue

        if not pts:
            return

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        ch = Header()
        ch.stamp = stamp
        ch.frame_id = 'odom'
        cloud = pc2.create_cloud(ch, fields, pts)
        self._pub_pc.publish(cloud)


# costmap
def _bresenham(
    x0: int, y0: int, x1: int, y1: int
) -> List[Tuple[int, int]]:
    cells: List[Tuple[int, int]] = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    while True:
        cells.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return cells


class ScanCostmapNode(Node):
    def __init__(self) -> None:
        super().__init__('scan_costmap')
        self.declare_parameter('resolution', 0.08)
        self.declare_parameter('width', 80)
        self.declare_parameter('height', 80)
        self.declare_parameter('inflation_cells', 1)

        gp = self.get_parameter
        self._res = float(gp('resolution').get_parameter_value().double_value)
        self._w = int(gp('width').get_parameter_value().integer_value)
        self._h = int(gp('height').get_parameter_value().integer_value)
        self._inflate = int(gp('inflation_cells').get_parameter_value().integer_value)

        self._tf_buf = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(
            self._tf_buf, self, spin_thread=True
        )

        self._rx = 0.0
        self._ry = 0.0
        self._have_pose = False

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(OccupancyGrid, 'local_costmap', map_qos)
        qos_scan = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, 'scan', self._scan_cb, qos_scan)
        self.create_subscription(Odometry, 'odom', self._odom_cb, 20)

    def _odom_cb(self, msg: Odometry) -> None:
        self._rx = msg.pose.pose.position.x
        self._ry = msg.pose.pose.position.y
        self._have_pose = True

    def _world_to_cell(
        self, ox: float, oy: float, wx: float, wy: float
    ) -> Optional[Tuple[int, int]]:
        ix = int((wx - ox) / self._res)
        iy = int((wy - oy) / self._res)
        if 0 <= ix < self._w and 0 <= iy < self._h:
            return (ix, iy)
        return None

    def _scan_cb(self, msg: LaserScan) -> None:
        if not self._have_pose:
            return
        try:
            tf_o = self._tf_buf.lookup_transform(
                'odom',
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.15),
            )
        except TransformException as ex:
            self.get_logger().debug(f'tf skip: {ex}')
            return

        ox = self._rx - (self._w * self._res) / 2.0
        oy = self._ry - (self._h * self._res) / 2.0

        data = [-1] * (self._w * self._h)
        n = len(msg.ranges)
        if n == 0:
            return
        inc = msg.angle_increment
        robot_cell = self._world_to_cell(ox, oy, self._rx, self._ry)
        if robot_cell is None:
            return
        rcx, rcy = robot_cell

        for i in range(n):
            r = msg.ranges[i]
            if not (msg.range_min < r < msg.range_max):
                continue
            ang = msg.angle_min + i * inc
            pt = PointStamped()
            pt.header = msg.header
            pt.point.x = float(r * math.cos(ang))
            pt.point.y = float(r * math.sin(ang))
            pt.point.z = 0.0
            try:
                p_odom = do_transform_point(pt, tf_o)
            except Exception:
                continue
            wx, wy = p_odom.point.x, p_odom.point.y
            hit = self._world_to_cell(ox, oy, wx, wy)
            if hit is None:
                continue
            hx, hy = hit
            for cx, cy in _bresenham(rcx, rcy, hx, hy):
                idx = cy * self._w + cx
                if data[idx] < 0:
                    data[idx] = 0
            for dx in range(-self._inflate, self._inflate + 1):
                for dy in range(-self._inflate, self._inflate + 1):
                    nx, ny = hx + dx, hy + dy
                    if 0 <= nx < self._w and 0 <= ny < self._h:
                        data[ny * self._w + nx] = 100

        grid = OccupancyGrid()
        grid.header.stamp = msg.header.stamp
        grid.header.frame_id = 'odom'
        grid.info = MapMetaData()
        grid.info.resolution = self._res
        grid.info.width = self._w
        grid.info.height = self._h
        grid.info.origin.position.x = ox
        grid.info.origin.position.y = oy
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        grid.data = data
        self._pub.publish(grid)


def main_relay() -> None:
    rclpy.init()
    node = ScanRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_costmap() -> None:
    rclpy.init()
    node = ScanCostmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
