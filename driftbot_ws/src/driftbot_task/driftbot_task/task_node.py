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

"""Tidying waypoint node."""

from __future__ import annotations

import csv
import math
import os
import subprocess
import time
from typing import Any, Dict, List, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker
import yaml


# spawn / odom zero
SPAWN_XY = (1.5, 9.0)
BOX_XY = (8.5, 9.0)

# object centers (sdf)
_PICKABLE_WORLD_XYZ: Dict[str, Tuple[float, float, float]] = {
    'toy_block_1': (-5.0, 3.5, 0.05),
    'toy_block_2': (-2.5, 8.0, 0.045),
    'toy_block_3': (8.0, 7.0, 0.05),
    'toy_cylinder_1': (-7.0, 7.0, 0.05),
    'can': (1.5, 7.5, 0.065),
    'ball': (6.0, 3.5, 0.05),
}

# magnet reach radii
_PICKABLE_SURFACE_RADIUS_M: Dict[str, float] = {
    'ball': 0.05,
    'can': 0.075,
    'toy_block_1': 0.065,
    'toy_block_2': 0.055,
    'toy_block_3': 0.065,
    'toy_cylinder_1': 0.065,
}


def quat_yaw(qz: float, qw: float) -> float:
    return math.atan2(2.0 * qz * qw, qw * qw - qz * qz)


def angle_diff(a: float, b: float) -> float:
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


def room_label(wx: float, wy: float) -> str:
    if -10.0 <= wx < 0.0 and 0.0 <= wy <= 10.0:
        return 'Room1'
    if 0.0 <= wx <= 10.0 and 0.0 <= wy <= 10.0:
        return 'Room2'
    if math.hypot(wx - BOX_XY[0], wy - BOX_XY[1]) < 2.5:
        return 'Drop_zone'
    return 'Outside_map'


# gz attach topics
_GZ_ATTACH_TOPIC = {
    'can': '/can/attach',
    'ball': '/ball/attach',
    'toy_block_1': '/toy_block_1/attach',
    'toy_block_2': '/toy_block_2/attach',
    'toy_block_3': '/toy_block_3/attach',
    'toy_cylinder_1': '/toy_cylinder_1/attach',
}


def _pick_face_xy_for_model(model: str) -> Optional[Tuple[float, float]]:
    """Pick target XY."""
    o = _PICKABLE_WORLD_XYZ.get(model)
    if o is None:
        return None
    return (o[0], o[1])


def _gz_publish_empty(topic: str) -> None:
    subprocess.run(
        ['gz', 'topic', '-t', topic, '-m', 'gz.msgs.Empty', '-p', ''],
        capture_output=True,
        timeout=5.0,
        check=False,
    )


def gz_detach_all_pickables() -> None:
    """Magnet off all."""
    _gz_publish_empty('/magnet_off')


def _default_config() -> Dict[str, Any]:
    return {
        'motion': {
            'loop_rate_hz': 20.0,
            'mission_time_limit_sec': 280.0,
            'per_waypoint_timeout_sec': 75.0,
            'pos_tol': 0.28,
            'v_max': 0.38,
            'w_max': 1.0,
            'nav_align_yaw_tol_rad': 0.1,
            'rotate_to_goal_gain': 2.4,
            'approach_linear_gain': 0.48,
            'pick_target_distance_m': 0.01,
            'pick_approach_v_max': 0.1,
            'pick_approach_timeout_sec': 35.0,
            'drop_approach_standoff_m': 0.5,
        },
        'progress': {
            'movement_time_allowance_sec': 12.0,
            'required_movement_radius': 0.45,
        },
        'recovery': {
            'spin_duration_sec': 1.1,
            'spin_w': 0.85,
            'backup_duration_sec': 0.65,
            'backup_v': -0.14,
            'wait_after_recovery_sec': 0.35,
            'min_rear_clearance_for_backup': 0.42,
        },
        'obstacle': {
            'front_stop': 0.38,
            'front_slow': 0.62,
            'rear_check_deg': 55.0,
            # wall follow
            'wall_follow_enable': True,
            'wall_follow_trigger_front_m': 0.45,
            'wall_follow_stuck_before_sec': 3.5,
            'wall_follow_max_duration_sec': 10.0,
            'wall_follow_linear_m_s': 0.12,
            'wall_follow_angular_gain': 2.0,
            'wall_follow_desired_side_m': 0.38,
        },
        'telemetry': {'publish_goal_markers': True},
        # bin zone
        'drop_zone': {
            'center_x': 8.5,
            'center_y': 9.0,
            'half_extent_x': 0.442,
            'half_extent_y': 0.585,
            'approach_margin_m': 0.35,
        },
        'arm': {
            'settle_drop': 0.9,
            'pick_magnet_pause': 0.45,
            'drop_magnet_pause': 0.35,
            'pick_creep_sec': 0.0,
            'drop_creep_sec': 0.0,
            'magnet_attach_radius_m': 0.13,
            'magnet_reach_slop_m': 0.02,
            'enforce_magnet_reach': True,
            # pre-odom hold
            'wait_stream_arm': True,
            'wait_arm_hz': 35.0,
            'wait_stream_pose': {'shoulder': 0.0, 'elbow': 0.0, 'wrist': 0.0},
            # arm ramp sim
            'bootstrap_hold_hz': 25.0,
            'bootstrap_stages': [
                {'shoulder': -0.00006, 'elbow': 0.0, 'wrist': -0.00017, 'settle': 1.2},
            ],
            'pick_pose_stream_sec': 0.45,
            'pick_pose_stream_hz': 28.0,
            'drop_pose_stream_sec': 0.35,
            'drop_pose_stream_hz': 28.0,
            'stage_settle_sec': 0.72,
            'attach_wait_sec': 2.5,
            'attach_wait_arm_hz': 25.0,
            # joint targets
            'stages': {
                'stage1': {'shoulder': 0.38, 'elbow': -0.88, 'wrist': 0.06},
                'stage2': {'shoulder': 0.42, 'elbow': -1.32, 'wrist': 0.06},
                'stage3': {'shoulder': 1.12, 'elbow': -1.92, 'wrist': 0.02},
            },
        },
        'waypoints': [],
    }


def _deep_merge(base: Dict[str, Any], over: Dict[str, Any]) -> Dict[str, Any]:
    out = dict(base)
    for k, v in over.items():
        if k in out and isinstance(out[k], dict) and isinstance(v, dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def load_tidying_config(path: Optional[str]) -> Dict[str, Any]:
    cfg = _default_config()
    if not path or not os.path.isfile(path):
        return cfg
    with open(path, 'r', encoding='utf-8') as f:
        loaded = yaml.safe_load(f)
    if isinstance(loaded, dict):
        cfg = _deep_merge(cfg, loaded)
    return cfg


class DriftbotTaskNode(Node):
    def __init__(self) -> None:
        super().__init__('driftbot_task')
        self.declare_parameter('log_path', '')
        self.declare_parameter('tidy_config', '')

        self._log_path = self.get_parameter('log_path').get_parameter_value().string_value
        tc = self.get_parameter('tidy_config').get_parameter_value().string_value.strip()
        if not tc:
            tc = os.path.join(
                get_package_share_directory('driftbot_task'),
                'config',
                'tidying.yaml',
            )
        self._cfg = load_tidying_config(tc if os.path.isfile(tc) else None)
        if not os.path.isfile(tc):
            self.get_logger().warn(f'Missing config {tc}, using defaults + empty waypoints')

        self._odom_xy_yaw: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._total_dist = 0.0
        self._last_odom: Optional[Tuple[float, float]] = None
        self._scan_min = float('inf')
        self._scan_min_front = float('inf')
        self._scan_min_front_l = float('inf')
        self._scan_min_front_r = float('inf')
        self._scan_left = float('inf')
        self._scan_right = float('inf')
        self._scan_min_rear = float('inf')
        self._scan_seen = False
        self._cam_samples = 0

        self._rooms_visited = set()
        self._csv_file = None
        self._csv_writer = None
        self._marker_id = 0

        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._marker_pub = self.create_publisher(Marker, 'tidying_goal_marker', 10)
        qos_scan = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, 'odom', self._odom_cb, 50)
        self.create_subscription(LaserScan, 'scan', self._scan_cb, qos_scan)
        self.create_subscription(
            Image,
            '/camera/image_raw',
            self._img_cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.create_timer(1.0, self._status_timer_cb)

        qos_arm = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._arm_sh = self.create_publisher(Float64, 'arm_cmd_shoulder', qos_arm)
        self._arm_el = self.create_publisher(Float64, 'arm_cmd_elbow', qos_arm)
        self._arm_wr = self.create_publisher(Float64, 'arm_cmd_wrist', qos_arm)

        self._tf_buf = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buf, self, spin_thread=True)

    def _magnet_world_xyz(self) -> Optional[Tuple[float, float, float]]:
        """Magnet TF world."""
        try:
            t = self._tf_buf.lookup_transform(
                'odom',
                'magnet_zone_link',
                self.get_clock().now(),
                timeout=Duration(seconds=0, nanoseconds=250_000_000),
            )
        except TransformException as ex:
            self.get_logger().debug(f'magnet_zone_link TF: {ex}')
            return None
        tr = t.transform.translation
        return (SPAWN_XY[0] + tr.x, SPAWN_XY[1] + tr.y, tr.z)

    def _magnet_reach_allows_attach(self, model: str) -> bool:
        """Attach radius check."""
        arm = self._cfg.get('arm') or {}
        defs = _default_config()['arm']
        if not bool(arm.get('enforce_magnet_reach', defs.get('enforce_magnet_reach', True))):
            return True
        rm = float(arm.get('magnet_attach_radius_m', defs.get('magnet_attach_radius_m', 0.13)))
        slop = float(arm.get('magnet_reach_slop_m', defs.get('magnet_reach_slop_m', 0.0)))
        center = _PICKABLE_WORLD_XYZ.get(model)
        rs = _PICKABLE_SURFACE_RADIUS_M.get(model)
        if center is None or rs is None:
            return True
        m = self._magnet_world_xyz()
        if m is None:
            self.get_logger().warn(
                'magnet_zone_link not in TF yet — skipping reach check (attach allowed)'
            )
            return True
        d = math.sqrt(
            (m[0] - center[0]) ** 2 + (m[1] - center[1]) ** 2 + (m[2] - center[2]) ** 2
        )
        limit = rm + rs + max(0.0, slop)
        if d > limit:
            self.get_logger().warn(
                f'Magnet reach: dist to "{model}" center {d:.3f} m > {limit:.3f} m '
                f'(magnet R={rm:.3f} + object ~{rs:.3f}).'
            )
            return False
        return True

    def gz_attach_pickable(self, model: str) -> bool:
        """Publish attach."""
        topic = _GZ_ATTACH_TOPIC.get(model)
        if not topic:
            return False
        if not self._magnet_reach_allows_attach(model):
            return False
        _gz_publish_empty(topic)
        return True

    def _sleep_sim(self, duration_sec: float) -> None:
        """Sim sleep."""
        if duration_sec <= 0.0:
            return
        end = self.get_clock().now() + Duration(seconds=duration_sec)
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for_sim_ready(self, timeout_wall_sec: float = 120.0) -> None:
        """Wait first odom."""
        arm = self._cfg.get('arm') or {}
        defs = _default_config()['arm']
        stream = bool(arm.get('wait_stream_arm', defs.get('wait_stream_arm', True)))
        hz = float(arm.get('wait_arm_hz', defs.get('wait_arm_hz', 35.0)))
        period_wall = 1.0 / max(5.0, hz)
        last_pub_wall = 0.0
        wpose = arm.get('wait_stream_pose')
        if not isinstance(wpose, dict):
            wpose = defs['wait_stream_pose']  # type: ignore[assignment]
        wsh = float(wpose.get('shoulder', 0.0))
        wel = float(wpose.get('elbow', 0.0))
        wwr = float(wpose.get('wrist', 0.0))

        wall_end = time.monotonic() + timeout_wall_sec
        self.get_logger().info('Waiting for simulation (first odom)...')
        while time.monotonic() < wall_end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if stream:
                noww = time.monotonic()
                if noww - last_pub_wall >= period_wall:
                    self._arm_publish_rad(wsh, wel, wwr)
                    last_pub_wall = noww
            if self._last_odom is not None:
                self.get_logger().info('Simulation ready — odom received.')
                return
        self.get_logger().warn(
            'Timeout waiting for odom; continuing — mission may misbehave until topics connect.'
        )

    def world_pose(self) -> Tuple[float, float, float]:
        ox, oy, yaw = self._odom_xy_yaw
        return (SPAWN_XY[0] + ox, SPAWN_XY[1] + oy, yaw)

    def _open_log(self) -> None:
        path = self._log_path.strip() or '/tmp/driftbot_tidying_path.csv'
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        self._csv_file = open(path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(['sim_time_sec', 'world_x', 'world_y', 'yaw', 'room', 'path_m'])
        self.get_logger().info(f'Logging odometry path to {path}')

    def _log_sample(self) -> None:
        if self._csv_writer is None:
            return
        wx, wy, yaw = self.world_pose()
        t = self.get_clock().now().seconds_nanoseconds()
        sec = t[0] + t[1] * 1e-9
        self._csv_writer.writerow(
            [
                f'{sec:.3f}',
                f'{wx:.4f}',
                f'{wy:.4f}',
                f'{yaw:.4f}',
                room_label(wx, wy),
                f'{self._total_dist:.4f}',
            ]
        )
        self._csv_file.flush()

    def _publish_goal_marker(self, goal_wx: float, goal_wy: float, label: str) -> None:
        if not self._cfg['telemetry'].get('publish_goal_markers', True):
            return
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'tidying_waypoint'
        self._marker_id += 1
        m.id = self._marker_id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = goal_wx - SPAWN_XY[0]
        m.pose.position.y = goal_wy - SPAWN_XY[1]
        m.pose.position.z = 0.15
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.22
        m.color.r = 0.1
        m.color.g = 0.7
        m.color.b = 0.2
        m.color.a = 0.9
        m.lifetime.sec = 0
        self.get_logger().debug(f'marker {label}')
        self._marker_pub.publish(m)

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = quat_yaw(q.z, q.w)
        self._odom_xy_yaw = (p.x, p.y, yaw)
        if self._last_odom is None:
            self._last_odom = (p.x, p.y)
            return
        lx, ly = self._last_odom
        self._total_dist += math.hypot(p.x - lx, p.y - ly)
        self._last_odom = (p.x, p.y)

    def _scan_cb(self, msg: LaserScan) -> None:
        self._scan_seen = True
        n = len(msg.ranges)
        if n == 0:
            return
        inc = msg.angle_increment
        valid = []
        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < msg.range_max:
                ang = msg.angle_min + i * inc
                valid.append((i, r, ang))
        if not valid:
            return
        self._scan_min = min(r for _, r, _ in valid)
        mid = n // 2
        span = max(3, n // 10)
        front_narrow = [r for i, r, a in valid if mid - span <= i <= mid + span]
        if front_narrow:
            self._scan_min_front = min(front_narrow)
        fl = [r for i, r, a in valid if mid < i <= mid + span * 2]
        fr = [r for i, r, a in valid if mid - span * 2 <= i < mid]
        if fl:
            self._scan_min_front_l = min(fl)
        if fr:
            self._scan_min_front_r = min(fr)
        left = [r for i, r, a in valid if i >= n * 3 // 4]
        right = [r for i, r, a in valid if i <= n // 4]
        if left:
            self._scan_left = min(left)
        if right:
            self._scan_right = min(right)

        rear_deg = math.radians(float(self._cfg['obstacle'].get('rear_check_deg', 55.0)))
        rear_rs = [r for _, r, a in valid if abs(abs(a) - math.pi) < rear_deg]
        if rear_rs:
            self._scan_min_rear = min(rear_rs)
        else:
            self._scan_min_rear = float('inf')

    def _img_cb(self, msg: Image) -> None:
        if msg.data:
            self._cam_samples += 1

    def _status_timer_cb(self) -> None:
        wx, wy, yaw = self.world_pose()
        ox, oy, _ = self._odom_xy_yaw
        self.get_logger().info(
            f'[telemetry] path={self._total_dist:.2f} m | odom=({ox:.2f},{oy:.2f}) | '
            f'world=({wx:.2f},{wy:.2f}) room={room_label(wx, wy)} | '
            f'scan_min={self._scan_min:.2f} front={self._scan_min_front:.2f} | '
            f'cam_msgs={self._cam_samples}'
        )

    def stop(self) -> None:
        self._cmd_pub.publish(Twist())

    def _obstacle_clearance(self) -> Tuple[float, float, float, float]:
        return (
            self._scan_min_front,
            self._scan_left,
            self._scan_right,
            self._scan_min_rear,
        )

    def _linear_speed_scale(self, front: float) -> float:
        obs = self._cfg['obstacle']
        f_stop = float(obs.get('front_stop', 0.38))
        f_slow = float(obs.get('front_slow', 0.62))
        if math.isinf(front):
            return 1.0
        if front <= f_stop:
            return 0.12
        if front >= f_slow:
            return 1.0
        return 0.12 + 0.88 * (front - f_stop) / (f_slow - f_stop)

    def _drive_for_sim_duration(self, twist: Twist, dur: Duration) -> None:
        end = self.get_clock().now() + dur
        period = 1.0 / max(10.0, float(self._cfg['motion'].get('loop_rate_hz', 20.0)))
        while self.get_clock().now() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            self._cmd_pub.publish(twist)
            self._sleep_sim(period)
        self.stop()

    def run_recovery(self) -> None:
        """Unstuck routine."""
        rec = self._cfg['recovery']
        self.get_logger().warn('Progress stalled — recovery (spin / backup / wait)')
        spin = Twist()
        spin.angular.z = float(rec.get('spin_w', 0.85))
        spin_dur = float(rec.get('spin_duration_sec', 1.1))
        self._drive_for_sim_duration(spin, Duration(seconds=spin_dur))

        rear = self._scan_min_rear
        min_rear = float(rec.get('min_rear_clearance_for_backup', 0.42))
        if not math.isinf(rear) and rear >= min_rear:
            back = Twist()
            back.linear.x = float(rec.get('backup_v', -0.14))
            back_dur = float(rec.get('backup_duration_sec', 0.65))
            self._drive_for_sim_duration(back, Duration(seconds=back_dur))

        wait = float(rec.get('wait_after_recovery_sec', 0.35))
        if wait > 0.0:
            self._drive_for_sim_duration(Twist(), Duration(seconds=wait))

    def navigate_to(
        self,
        goal_wx: float,
        goal_wy: float,
        label: str,
        timeout_sec: Optional[float] = None,
        pos_tol_override: Optional[float] = None,
    ) -> bool:
        """Drive to XY goal."""
        m = self._cfg['motion']
        pr = self._cfg['progress']
        if timeout_sec is None:
            timeout_sec = float(m.get('per_waypoint_timeout_sec', 75.0))
        pos_tol = (
            float(pos_tol_override)
            if pos_tol_override is not None
            else float(m.get('pos_tol', 0.28))
        )
        v_max = float(m.get('v_max', 0.38))
        w_max = float(m.get('w_max', 1.0))
        align_tol = float(m.get('nav_align_yaw_tol_rad', 0.1))
        k_w = float(m.get('rotate_to_goal_gain', 2.4))
        k_v = float(m.get('approach_linear_gain', 0.48))
        period = 1.0 / max(10.0, float(m.get('loop_rate_hz', 20.0)))

        end = self.get_clock().now() + Duration(seconds=timeout_sec)
        prog_allow = float(pr.get('movement_time_allowance_sec', 12.0))
        prog_rad = float(pr.get('required_movement_radius', 0.45))
        prog_deadline = self.get_clock().now() + Duration(seconds=prog_allow)
        ax, ay, _ = self.world_pose()
        obs = self._cfg.get('obstacle') or {}
        wf_enable = bool(obs.get('wall_follow_enable', True))
        wf_trig = float(obs.get('wall_follow_trigger_front_m', 0.45))
        wf_stuck = float(obs.get('wall_follow_stuck_before_sec', 3.5))
        wf_dur = float(obs.get('wall_follow_max_duration_sec', 10.0))
        wf_v = float(obs.get('wall_follow_linear_m_s', 0.12))
        wf_kw = float(obs.get('wall_follow_angular_gain', 2.0))
        wf_side = float(obs.get('wall_follow_desired_side_m', 0.38))
        wall_follow_until = None
        stagn_t = 0.0
        last_dist = float('inf')

        while rclpy.ok():
            now = self.get_clock().now()
            if now > end:
                self.get_logger().warn(f'Timeout reaching {label} ({goal_wx:.1f},{goal_wy:.1f})')
                self.stop()
                return False
            rclpy.spin_once(self, timeout_sec=0.0)

            wx, wy, yaw = self.world_pose()
            self._rooms_visited.add(room_label(wx, wy))

            if self.get_clock().now() > prog_deadline:
                moved = math.hypot(wx - ax, wy - ay)
                if moved < prog_rad:
                    self.run_recovery()
                ax, ay = wx, wy
                prog_deadline = self.get_clock().now() + Duration(seconds=prog_allow)

            dx = goal_wx - wx
            dy = goal_wy - wy
            dist = math.hypot(dx, dy)
            if dist < pos_tol:
                self.stop()
                self._publish_goal_marker(goal_wx, goal_wy, label)
                self.get_logger().info(f'Goal reached: {label}')
                return True

            front, left, right, _rear = self._obstacle_clearance()

            if wall_follow_until is not None:
                if now < wall_follow_until:
                    right_m = right if not math.isinf(right) else 2.0
                    twist = Twist()
                    twist.linear.x = wf_v
                    twist.angular.z = max(
                        -w_max * 0.75,
                        min(w_max * 0.75, wf_kw * (wf_side - right_m)),
                    )
                    self._cmd_pub.publish(twist)
                    self._sleep_sim(period)
                    continue
                wall_follow_until = None
                stagn_t = 0.0
                last_dist = dist

            if dist < last_dist - 0.02:
                last_dist = dist
                stagn_t = 0.0
            else:
                stagn_t += period
            if wf_enable and front < wf_trig and stagn_t >= wf_stuck:
                wall_follow_until = now + Duration(seconds=wf_dur)
                stagn_t = 0.0
                self.get_logger().info('Nav: wall follow')

            target_h = math.atan2(dy, dx)
            h_err = angle_diff(target_h, yaw)
            v_scale = self._linear_speed_scale(front)

            twist = Twist()
            if front < 0.52 and not math.isinf(front):
                twist.linear.x = 0.04 * v_scale
                fl = self._scan_min_front_l
                fr = self._scan_min_front_r
                if math.isinf(fl) and math.isinf(fr):
                    bias = 1.0 if left > right else -1.0
                else:
                    bias = 1.0 if fl > fr else -1.0
                twist.angular.z = bias * w_max * 0.75
            elif abs(h_err) > align_tol:
                twist.angular.z = max(-w_max, min(w_max, k_w * h_err))
            else:
                twist.angular.z = max(-w_max, min(w_max, k_w * h_err))
                twist.linear.x = min(v_max * v_scale, k_v * dist)

            self._cmd_pub.publish(twist)
            self._sleep_sim(period)

        self.stop()
        return False

    def rotate_to_face(
        self, target_wx: float, target_wy: float, timeout_sec: float = 35.0
    ) -> None:
        """Turn to point."""
        m = self._cfg['motion']
        w_max = float(m.get('w_max', 1.0))
        k_w = 2.85
        face_tol = 0.11
        period = 1.0 / max(10.0, float(m.get('loop_rate_hz', 20.0)))
        end = self.get_clock().now() + Duration(seconds=timeout_sec)
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.0)
            wx, wy, yaw = self.world_pose()
            target_h = math.atan2(target_wy - wy, target_wx - wx)
            h_err = angle_diff(target_h, yaw)
            if abs(h_err) < face_tol:
                self.stop()
                return
            twist = Twist()
            twist.angular.z = max(-w_max, min(w_max, k_w * h_err))
            self._cmd_pub.publish(twist)
            self._sleep_sim(period)
        self.stop()

    def _drop_approach_hold_xy(self) -> Tuple[float, float]:
        """Drop standoff XY."""
        dz = self._cfg.get('drop_zone')
        if not isinstance(dz, dict):
            cx, cy = BOX_XY[0], BOX_XY[1]
        else:
            cx = float(dz.get('center_x', BOX_XY[0]))
            cy = float(dz.get('center_y', BOX_XY[1]))
        m = self._cfg.get('motion') or {}
        standoff = float(m.get('drop_approach_standoff_m', 0.5))
        wx, wy, _ = self.world_pose()
        vx, vy = wx - cx, wy - cy
        d = math.hypot(vx, vy)
        if d < 0.08:
            return (cx - standoff, cy)
        return (cx + standoff * vx / d, cy + standoff * vy / d)

    def _approach_xy_until_distance(self, tx: float, ty: float, dist_target: float) -> None:
        """Creep to distance."""
        m = self._cfg.get('motion') or {}
        defs = _default_config()['motion']
        timeout = float(
            m.get('pick_approach_timeout_sec', defs.get('pick_approach_timeout_sec', 35.0))
        )
        v_max = float(m.get('pick_approach_v_max', defs.get('pick_approach_v_max', 0.1)))
        w_max = float(m.get('w_max', 1.0))
        period = 1.0 / max(10.0, float(m.get('loop_rate_hz', 20.0)))
        end = self.get_clock().now() + Duration(seconds=timeout)
        self.get_logger().info(
            f'Pick approach: creep until distance ≤ {dist_target:.2f} m (object in front)'
        )
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.0)
            wx, wy, yaw = self.world_pose()
            dx, dy = tx - wx, ty - wy
            dist = math.hypot(dx, dy)
            if dist <= dist_target:
                self.stop()
                self.get_logger().info(f'Pick approach: hold at {dist:.3f} m')
                return
            target_h = math.atan2(dy, dx)
            h_err = angle_diff(target_h, yaw)
            twist = Twist()
            if abs(h_err) > 0.12:
                twist.angular.z = max(-w_max, min(w_max, 2.6 * h_err))
            else:
                twist.linear.x = min(v_max, max(0.035, 0.42 * dist))
            front, _, _, _ = self._obstacle_clearance()
            if front < 0.34 and not math.isinf(front):
                twist.linear.x = min(twist.linear.x, 0.05)
            self._cmd_pub.publish(twist)
            self._sleep_sim(period)
        self.stop()
        self.get_logger().warn('Pick approach: timeout — continuing arm sequence anyway')

    def _arm_rviz_stage_triplet(self, stage_key: str) -> Dict[str, float]:
        """Stage joint dict."""
        arm = self._cfg.get('arm') or {}
        defs = _default_config()['arm']
        st = arm.get('stages')
        if isinstance(st, dict):
            d = st.get(stage_key)
            if isinstance(d, dict):
                sh = d.get('shoulder')
                el = d.get('elbow')
                wr = d.get('wrist')
                if sh is not None and el is not None and wr is not None:
                    return {
                        'shoulder': float(sh),
                        'elbow': float(el),
                        'wrist': float(wr),
                    }
        fb_st = defs.get('stages') or {}
        if isinstance(fb_st, dict) and isinstance(fb_st.get(stage_key), dict):
            d = fb_st[stage_key]
            return {
                'shoulder': float(d['shoulder']),
                'elbow': float(d['elbow']),
                'wrist': float(d['wrist']),
            }
        s1 = fb_st.get('stage1') if isinstance(fb_st, dict) else None
        if isinstance(s1, dict):
            self.get_logger().warn(
                f'arm.stages.{stage_key} missing joint triple; using stage1 as fallback'
            )
            return {
                'shoulder': float(s1['shoulder']),
                'elbow': float(s1['elbow']),
                'wrist': float(s1['wrist']),
            }
        return {'shoulder': 0.0, 'elbow': 0.0, 'wrist': 0.0}

    def _arm_goto_rviz_stage(self, stage_key: str) -> None:
        arm = self._cfg.get('arm') or {}
        defs = _default_config()['arm']
        settle_map = arm.get('stage_settle')
        if isinstance(settle_map, dict) and stage_key in settle_map:
            sec = float(settle_map[stage_key])
        else:
            sec = float(arm.get('stage_settle_sec', defs.get('stage_settle_sec', 0.72)))
        p = self._arm_rviz_stage_triplet(stage_key)
        self._arm_publish_rad(p['shoulder'], p['elbow'], p['wrist'])
        self._sleep_sim(sec)

    def _arm_publish_rad(self, shoulder: float, elbow: float, wrist: float) -> None:
        self._arm_sh.publish(Float64(data=shoulder))
        self._arm_el.publish(Float64(data=elbow))
        self._arm_wr.publish(Float64(data=wrist))
        for _ in range(3):
            rclpy.spin_once(self, timeout_sec=0.0)

    def _arm_hold_rad(
        self, shoulder: float, elbow: float, wrist: float, duration_sec: float, hz: float
    ) -> None:
        """Hold joints; sim+wall clock."""
        if duration_sec <= 0.0:
            return
        period = 1.0 / max(5.0, hz)
        t_wall_start = time.monotonic()
        end = self.get_clock().now() + Duration(seconds=duration_sec)
        while rclpy.ok():
            wall_elapsed = time.monotonic() - t_wall_start
            sim_done = self.get_clock().now() >= end
            if sim_done and wall_elapsed >= duration_sec:
                break
            self._arm_publish_rad(shoulder, elbow, wrist)
            self._sleep_sim(period)

    def arm_stage_to_carry(self) -> None:
        """Bootstrap arm poses."""
        arm = self._cfg.get('arm') or {}
        defs = _default_config()['arm']
        stages = arm.get('bootstrap_stages')
        if not isinstance(stages, list) or not stages:
            stages = defs['bootstrap_stages']  # type: ignore[assignment]
        hz = float(arm.get('bootstrap_hold_hz', defs.get('bootstrap_hold_hz', 25.0)))
        self.get_logger().info(f'Arm: staged carry — {len(stages)} segments @ {hz:.0f} Hz sim')
        for st in stages:
            if not isinstance(st, dict):
                continue
            sh = float(st.get('shoulder', 0.0))
            el = float(st.get('elbow', 0.0))
            wr = float(st.get('wrist', 0.0))
            settle = float(st.get('settle', 0.4))
            self._arm_hold_rad(sh, el, wr, settle, hz)

    def run_pick_sequence(
        self,
        pick_face: Optional[Tuple[float, float]],
        pick_model: str,
    ) -> None:
        """Pick arm+magnets."""
        arm = self._cfg.get('arm') or {}
        defs = _default_config()['arm']
        m = self._cfg.get('motion') or {}
        mdef = _default_config()['motion']
        if pick_face is not None:
            self.rotate_to_face(pick_face[0], pick_face[1])
            dist_tgt = float(
                m.get('pick_target_distance_m', mdef.get('pick_target_distance_m', 0.25))
            )
            self._approach_xy_until_distance(pick_face[0], pick_face[1], dist_tgt)

        self.get_logger().info('Pick arm: RViz stage1 (rest / default)')
        self._arm_goto_rviz_stage('stage1')
        self.get_logger().info('Pick arm: RViz stage2 (intermediate)')
        self._arm_goto_rviz_stage('stage2')
        self.get_logger().info('Pick arm: RViz stage3 (magnet reach)')
        self._arm_goto_rviz_stage('stage3')
        hold_sec = float(arm.get('pick_pose_stream_sec', defs.get('pick_pose_stream_sec', 0.0)))
        stream_hz = float(arm.get('pick_pose_stream_hz', defs.get('pick_pose_stream_hz', 0.0)))
        if hold_sec > 0.0 and stream_hz > 0.0:
            p = self._arm_rviz_stage_triplet('stage3')
            self._arm_hold_rad(p['shoulder'], p['elbow'], p['wrist'], hold_sec, stream_hz)
        for _ in range(12):
            rclpy.spin_once(self, timeout_sec=0.0)
        creep = float(arm.get('pick_creep_sec', 0.0))
        if creep > 0.0:
            self.get_logger().info(f'Pick: creep {creep:.1f}s (LiDAR contact hint)')
            self.approach_contact(creep)
        self.get_logger().info(f'Pick: magnet on — attach "{pick_model}"')
        if not self.gz_attach_pickable(pick_model):
            self.get_logger().error(
                f'Attach failed for "{pick_model}". '
                f'Valid models: {sorted(_GZ_ATTACH_TOPIC)}. '
                'If model is valid, magnet may be beyond '
                'arm.magnet_attach_radius_m (URDF sphere).'
            )
        wait_attach = float(arm.get('attach_wait_sec', defs.get('attach_wait_sec', 2.5)))
        stream_hz = float(arm.get('attach_wait_arm_hz', defs.get('attach_wait_arm_hz', 25.0)))
        p3 = self._arm_rviz_stage_triplet('stage3')
        period = 1.0 / max(5.0, stream_hz) if stream_hz > 0.0 else None
        t_next_wall = time.monotonic()
        t_end = self.get_clock().now() + Duration(seconds=wait_attach)
        while rclpy.ok() and self.get_clock().now() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if period is not None:
                noww = time.monotonic()
                if noww >= t_next_wall:
                    self._arm_publish_rad(p3['shoulder'], p3['elbow'], p3['wrist'])
                    t_next_wall = noww + period
        self._sleep_sim(float(arm.get('pick_magnet_pause', 0.45)))
        self.get_logger().info('Pick arm: stage2 (lift off table)')
        self._arm_goto_rviz_stage('stage2')
        self.get_logger().info('Pick arm: stage1 (carry / transport)')
        self._arm_goto_rviz_stage('stage1')

    def run_drop_sequence(self) -> None:
        """Drop sequence."""
        arm = self._cfg.get('arm') or {}
        defs = _default_config()['arm']
        self.get_logger().info('Drop arm: RViz stage2')
        self._arm_goto_rviz_stage('stage2')
        self.get_logger().info('Drop arm: stage3 over bin (same joints as pick reach)')
        p3 = self._arm_rviz_stage_triplet('stage3')
        self._arm_publish_rad(p3['shoulder'], p3['elbow'], p3['wrist'])
        self._sleep_sim(float(arm.get('settle_drop', defs.get('settle_drop', 0.9))))
        d_hold = float(arm.get('drop_pose_stream_sec', defs.get('drop_pose_stream_sec', 0.0)))
        d_hz = float(arm.get('drop_pose_stream_hz', defs.get('drop_pose_stream_hz', 0.0)))
        if d_hold > 0.0 and d_hz > 0.0:
            p = self._arm_rviz_stage_triplet('stage3')
            self._arm_hold_rad(p['shoulder'], p['elbow'], p['wrist'], d_hold, d_hz)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.0)
        creep = float(arm.get('drop_creep_sec', 0.0))
        if creep > 0.0:
            self.get_logger().info(f'Drop: creep {creep:.1f}s')
            self.approach_contact(creep)
        self.get_logger().info('Drop: magnet off (detach all pickables)')
        gz_detach_all_pickables()
        self._sleep_sim(float(arm.get('drop_magnet_pause', 0.35)))
        self.get_logger().info('Drop arm: RViz stage2 (retract)')
        self._arm_goto_rviz_stage('stage2')
        self.get_logger().info('Drop arm: RViz stage1 (rest)')
        self._arm_goto_rviz_stage('stage1')

    def approach_contact(self, duration_sec: float = 2.8) -> None:
        """Nudge forward."""
        t_end = self.get_clock().now() + Duration(seconds=duration_sec)
        period = 1.0 / max(10.0, float(self._cfg['motion'].get('loop_rate_hz', 20.0)))
        while self.get_clock().now() < t_end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            front, _, _, _ = self._obstacle_clearance()
            t = Twist()
            t.linear.x = 0.1
            if front < 0.22 and not math.isinf(front):
                self.get_logger().info('Near object (LiDAR front range)')
                break
            self._cmd_pub.publish(t)
            self._sleep_sim(period)
        self.stop()

    def run_mission(self) -> None:
        self.wait_for_sim_ready()
        self.get_logger().info(
            'Arm: staged carry after odom '
            '(see arm.wait_stream_* / bootstrap_stages)'
        )
        self.arm_stage_to_carry()
        self._open_log()
        m = self._cfg['motion']
        mlim = float(m.get('mission_time_limit_sec', 280.0))
        deadline = self.get_clock().now() + Duration(seconds=mlim)
        log_next = self.get_clock().now()
        waypoints: List[Dict[str, Any]] = self._cfg.get('waypoints') or []
        if not waypoints:
            self.get_logger().error('No waypoints in config; aborting')
            return

        nwp = len(waypoints)
        self.get_logger().info(
            f'Starting tidying mission — {nwp} waypoints: '
            'navigate (align then drive) → optional drop/pick → next'
        )
        picked = False
        wi = 0
        while rclpy.ok() and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.0)
            if wi >= nwp:
                break
            wp = waypoints[wi]
            wx = float(wp['x'])
            wy = float(wp['y'])
            label = str(wp.get('name', f'wp{wi}'))
            pos_tol_wp = wp.get('pos_tol')
            pos_tol_override = float(pos_tol_wp) if pos_tol_wp is not None else None
            if wp.get('release_magnet'):
                hx, hy = self._drop_approach_hold_xy()
                self.get_logger().info(
                    f'Executing waypoint {wi + 1}/{nwp}: {label} -> '
                    f'drop hold ({hx:.2f},{hy:.2f}) [standoff from bin]'
                )
                ok = self.navigate_to(
                    hx,
                    hy,
                    f'{label} (bin standoff)',
                    pos_tol_override=pos_tol_override,
                )
            else:
                self.get_logger().info(
                    f'Executing waypoint {wi + 1}/{nwp}: {label} -> ({wx:.1f},{wy:.1f})'
                )
                ok = self.navigate_to(
                    wx,
                    wy,
                    label,
                    pos_tol_override=pos_tol_override,
                )
            if not ok:
                self.get_logger().warn(f'Waypoint failed (continue): {label}')
            else:
                if wp.get('release_magnet'):
                    self.run_drop_sequence()
                    picked = False

            pick_face: Optional[Tuple[float, float]] = None
            pick_model: Optional[str] = None
            if isinstance(wp.get('pick'), dict):
                pk = wp['pick']
                if 'face_x' in pk and 'face_y' in pk:
                    pick_face = (float(pk['face_x']), float(pk['face_y']))
                if 'model' in pk:
                    pick_model = str(pk['model']).strip()
                do_pick = True
            elif wp.get('pick_after'):
                do_pick = True
                if 'pick_face_x' in wp and 'pick_face_y' in wp:
                    pick_face = (float(wp['pick_face_x']), float(wp['pick_face_y']))
                if wp.get('pick_model') is not None:
                    pick_model = str(wp['pick_model']).strip()
            else:
                do_pick = False

            if do_pick and ok:
                if not pick_model:
                    pick_model = 'can'
                    self.get_logger().warn(
                        'Waypoint has pick_after but no pick_model; defaulting to "can". '
                        'Set pick_model (SDF model name) to avoid wrong attach.'
                    )
                if pick_face is None:
                    pick_face = _pick_face_xy_for_model(pick_model)
                self.run_pick_sequence(pick_face, pick_model)
                picked = True

            wi += 1

            if (self.get_clock().now() - log_next).nanoseconds * 1e-9 >= 0.25:
                self._log_sample()
                log_next = self.get_clock().now()

        if picked:
            self.get_logger().info('Detaching pickables at mission end')
            gz_detach_all_pickables()
            self._sleep_sim(0.5)

        self.stop()
        wx, wy, _ = self.world_pose()
        rooms = ', '.join(sorted(self._rooms_visited))
        self.get_logger().info(
            f'=== Mission complete === distance_travelled={self._total_dist:.2f} m | '
            f'rooms_seen=[{rooms}] | final_world=({wx:.2f},{wy:.2f}) | '
            f'scan_ok={self._scan_seen} | cam_msgs={self._cam_samples}'
        )
        if self._csv_file:
            self._csv_file.close()
            self._csv_file = None


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = DriftbotTaskNode()
    try:
        node.run_mission()
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
