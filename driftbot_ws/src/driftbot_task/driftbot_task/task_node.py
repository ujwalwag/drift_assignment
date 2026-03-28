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

"""Scripted Gazebo mission.

Every translation leg: **face goal bearing** (in place), then **drive with ω=0**.
All ``cmd_vel`` goes through one function so you can flip yaw sign for ROS/GZ.

World pose: Gazebo diff-drive ``/odom`` is in a frame whose +X was **robot
forward at spawn**. Convert to world with ``spawn_yaw`` (must match
``home.launch`` / ``ros_gz_sim create -Y``).

In-place spins on the toy leg use **IMU** only (``/imu/data``): diff-drive
odom integrates yaw too fast for reliable closed-loop turns on the spot.
"""

from __future__ import annotations

import math
import os
import subprocess
import time
from typing import Any, Dict, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Empty, Float64
import yaml

OBJECTS: Dict[str, Tuple[float, float, float]] = {
    'can': (1.5, 9.0, 0.065),
    'toy_block_1': (7.5, 5.0, 0.05),
    'ball': (1.5, 2.0, 0.05),
}
ATTACH_TOPIC = {
    'can': '/can/attach',
    'toy_block_1': '/toy_block_1/attach',
    'ball': '/ball/attach',
}
DEFAULT_BOX = (8.5, 9.0)


def quat_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def angle_diff(target: float, current: float) -> float:
    d = target - current
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def gz_empty(topic: str) -> None:
    subprocess.run(
        ['gz', 'topic', '-t', topic, '-m', 'gz.msgs.Empty', '-p', ''],
        capture_output=True,
        timeout=5.0,
        check=False,
    )


def _deep_merge(a: Dict[str, Any], b: Dict[str, Any]) -> Dict[str, Any]:
    out = dict(a)
    for k, v in b.items():
        if k in out and isinstance(out[k], dict) and isinstance(v, dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def default_config() -> Dict[str, Any]:
    return {
        'motion': {
            'v_max': 0.28,
            'turn_k': 0.65,
            'turn_w_max': 0.36,
            'heading_tol_rad': 0.05,
            'turn_hz': 30.0,
            'loop_hz': 20.0,
            'brake_iters': 12,
            'pos_tol': 0.14,
            'pre_drop_pos_tol': 0.055,
            'drive_odom_step_cap': True,
            'drive_brake_at_goal': True,
            'drive_timeout_sec': 120.0,
            'turn_timeout_sec': 45.0,
            'approach_k': 0.45,
            'slow_radius_m': 0.55,
            'v_near_max': 0.08,
            'v_crawl_min': 0.03,
            'pause_sec': 0.3,
            'halt_pause_sec': 0.55,
            'cmd_vel_angular_scale': 1.0,
            # Before drop after (7.5, pre_drop_y): +π/2 CCW; flip sign in yaml if needed.
            'pre_drop_turn_rad': 1.5707963267948966,
            'pre_drop_y': 8.5,
            # Block until /imu/data has orientation before any IMU turn.
            'imu_wait_timeout_sec': 90.0,
        },
        'drop_zone': {'center_x': DEFAULT_BOX[0], 'center_y': DEFAULT_BOX[1]},
        'arm': {
            'stage_settle_sec': 0.72,
            'settle_drop': 1.0,
            'attach_wait_sec': 1.2,
            'attach_hz': 5.0,
            'pick_magnet_pause': 0.25,
            'drop_magnet_pause': 0.4,
            'pick_standoff_m': 0.22,
            'pick_pose_stream_sec': 0.28,
            'pick_pose_stream_hz': 28.0,
            'attach_pre_delay_sec': 0.22,
            'attach_burst_count': 3,
            'attach_burst_period_sec': 0.1,
            'attach_resend_sec': 0.32,
            'drop_pose_stream_sec': 0.35,
            'drop_pose_stream_hz': 28.0,
            'stages': {
                'stage1': {'shoulder': -0.00006, 'elbow': 0.0, 'wrist': -0.00017},
                'stage2': {'shoulder': -0.00006, 'elbow': 0.0, 'wrist': -1.309},
                'stage3': {'shoulder': -0.63, 'elbow': 0.0, 'wrist': -1.309},
            },
        },
    }


def load_yaml(path: Optional[str]) -> Dict[str, Any]:
    cfg = default_config()
    if path and os.path.isfile(path):
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        if isinstance(data, dict):
            cfg = _deep_merge(cfg, data)
    return cfg


class TaskNode(Node):
    def __init__(self) -> None:
        super().__init__('driftbot_task')
        self.declare_parameter('mission_config', '')
        self.declare_parameter('spawn_x', '7.5')
        self.declare_parameter('spawn_y', '9.0')
        self.declare_parameter('spawn_yaw', '-1.5708')
        self.declare_parameter('imu_topic', '/imu/data')

        mc = self.get_parameter('mission_config').value
        if not str(mc).strip():
            mc = os.path.join(
                get_package_share_directory('driftbot_task'), 'config', 'mission.yaml'
            )
        self._cfg = load_yaml(mc if os.path.isfile(str(mc)) else None)

        def sf(name: str, default: float) -> float:
            v = self.get_parameter(name).value
            if isinstance(v, (int, float)):
                return float(v)
            return float(str(v).strip())

        self._spawn = (sf('spawn_x', 7.5), sf('spawn_y', 9.0))
        self._spawn_yaw = sf('spawn_yaw', -math.pi / 2.0)
        self._w_scale = float(self._cfg['motion'].get('cmd_vel_angular_scale', 1.0))

        self._odom: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._odom_n = 0
        self._imu_yaw: Optional[float] = None
        self._imu_n = 0
        self._carry = False

        self._cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, 'odom', self._odom_cb, 50)
        imu_topic = str(self.get_parameter('imu_topic').value or '/imu/data')
        self.create_subscription(Imu, imu_topic, self._imu_cb, 50)
        self.create_subscription(Image, '/camera/image_raw', lambda *_: None, qos)

        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pub_sh = self.create_publisher(Float64, 'arm_cmd_shoulder', q)
        self._pub_el = self.create_publisher(Float64, 'arm_cmd_elbow', q)
        self._pub_wr = self.create_publisher(Float64, 'arm_cmd_wrist', q)

        qa = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._pub_attach: Dict[str, Any] = {
            t: self.create_publisher(Empty, t, qa)
            for t in sorted(set(ATTACH_TOPIC.values()))
        }
        self._pub_magnet_off = self.create_publisher(Empty, '/magnet_off', qa)

        self.get_logger().info(
            f'cmd_vel angular scale = {self._w_scale}; '
            f'spawn world ({self._spawn[0]}, {self._spawn[1]}) yaw={self._spawn_yaw:.4f}'
        )

    def _m(self) -> Dict[str, Any]:
        return self._cfg['motion']

    def _attach_pulse(self, topic: str) -> None:
        pub = self._pub_attach.get(topic)
        if pub is not None:
            pub.publish(Empty())
        gz_empty(topic)

    def _magnet_off_pulse(self) -> None:
        self._pub_magnet_off.publish(Empty())
        gz_empty('/magnet_off')

    def _send_cmd(self, tw: Twist) -> None:
        out = Twist()
        out.linear.x = tw.linear.x
        out.linear.y = tw.linear.y
        out.linear.z = tw.linear.z
        out.angular.x = tw.angular.x
        out.angular.y = tw.angular.y
        out.angular.z = tw.angular.z * self._w_scale
        self._cmd.publish(out)

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._odom = (p.x, p.y, quat_yaw(q.x, q.y, q.z, q.w))
        self._odom_n += 1

    def _imu_cb(self, msg: Imu) -> None:
        q = msg.orientation
        n = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if n < 1e-10:
            return
        self._imu_yaw = quat_yaw(q.x, q.y, q.z, q.w)
        self._imu_n += 1

    def world_pose(self) -> Tuple[float, float, float]:
        """Map planar odom (spawn-aligned) into fixed world XY and yaw."""
        ox, oy, yaw_odom = self._odom
        c = math.cos(self._spawn_yaw)
        s = math.sin(self._spawn_yaw)
        wx = self._spawn[0] + c * ox - s * oy
        wy = self._spawn[1] + s * ox + c * oy
        w_yaw = wrap_pi(self._spawn_yaw + yaw_odom)
        return (wx, wy, w_yaw)

    def _period(self) -> float:
        hz = float(self._m().get('loop_hz', 20.0))
        return 1.0 / max(5.0, hz)

    def _turn_dt(self) -> float:
        hz = float(self._m().get('turn_hz', 30.0))
        return 1.0 / max(5.0, hz)

    def _spin_sleep(self, sec: float) -> None:
        if sec <= 0.0:
            return
        end = self.get_clock().now() + Duration(seconds=sec)
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def _pause(self) -> None:
        s = float(self._m().get('pause_sec', 0.0))
        if s > 0.0:
            self._spin_sleep(s)

    def _brake(self) -> None:
        z = Twist()
        dt = self._period()
        for _ in range(max(4, int(self._m().get('brake_iters', 12)))):
            self._send_cmd(z)
            rclpy.spin_once(self, timeout_sec=0.0)
            self._spin_sleep(dt)

    def _halt_stop(self, label: str = '') -> None:
        """Zero cmd_vel, brake iterations, then hold so the base settles."""
        msg = f'Halt stop {label}'.strip()
        if msg != 'Halt stop':
            self.get_logger().info(msg)
        self.stop()
        self._brake()
        self._spin_sleep(float(self._m().get('halt_pause_sec', 0.5)))

    def stop(self) -> None:
        self._send_cmd(Twist())

    def turn_to_yaw(self, target_yaw: float, label: str = '') -> None:
        m = self._m()
        tol = float(m.get('heading_tol_rad', 0.05))
        k = float(m.get('turn_k', 0.55))
        wcap = float(m.get('turn_w_max', 0.2))
        tout = float(m.get('turn_timeout_sec', 45.0))
        dt = self._turn_dt()
        end = self.get_clock().now() + Duration(seconds=tout)
        self.get_logger().info(f'Turn → {target_yaw:.4f} rad {label}')
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.0)
            err = angle_diff(target_yaw, self.world_pose()[2])
            if abs(err) < tol:
                break
            tw = Twist()
            tw.angular.z = max(-wcap, min(wcap, k * err))
            self._send_cmd(tw)
            self._spin_sleep(dt)
        self.stop()
        self._brake()

    def turn_by(self, delta: float, label: str = '') -> None:
        rclpy.spin_once(self, timeout_sec=0.0)
        y = self.world_pose()[2]
        self.turn_to_yaw(wrap_pi(y + delta), label)

    def _wait_imu_ready(self, why: str = '') -> None:
        m = self._m()
        tout = float(m.get('imu_wait_timeout_sec', 90.0))
        t0 = time.monotonic()
        msg = f'Waiting for IMU {why}'.strip()
        if msg != 'Waiting for IMU':
            self.get_logger().info(msg)
        while rclpy.ok() and time.monotonic() - t0 < tout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._imu_n >= 3 and self._imu_yaw is not None:
                return
        imu_topic = str(self.get_parameter('imu_topic').value or '/imu/data')
        raise RuntimeError(
            f'IMU required: no orientation on {imu_topic} within {tout}s. '
            'Check base_link IMU in URDF, gz-sim-imu-system in world, and '
            'ros_gz_bridge /imu → /imu/data.'
        )

    def turn_to_imu_yaw(self, target_yaw: float, label: str = '') -> None:
        """Closed-loop yaw using IMU orientation (world yaw of base_link)."""
        m = self._m()
        tol = float(m.get('heading_tol_rad', 0.05))
        k = float(m.get('turn_k', 0.55))
        wcap = float(m.get('turn_w_max', 0.2))
        tout = float(m.get('turn_timeout_sec', 45.0))
        dt = self._turn_dt()
        end = self.get_clock().now() + Duration(seconds=tout)
        self.get_logger().info(f'IMU turn → {target_yaw:.4f} rad {label}')
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.0)
            if self._imu_yaw is None:
                raise RuntimeError('IMU orientation lost during turn')
            err = angle_diff(target_yaw, self._imu_yaw)
            if abs(err) < tol:
                break
            tw = Twist()
            tw.angular.z = max(-wcap, min(wcap, k * err))
            self._send_cmd(tw)
            self._spin_sleep(dt)
        self.stop()
        self._brake()
        if self._imu_yaw is None:
            raise RuntimeError('IMU orientation missing after turn')
        if abs(angle_diff(target_yaw, self._imu_yaw)) >= tol:
            raise RuntimeError(
                f'IMU turn did not converge: want {target_yaw:.4f} have {self._imu_yaw:.4f} {label}'
            )

    def turn_by_imu(self, delta: float, label: str = '') -> None:
        """Relative spin using IMU only (blocks until IMU valid)."""
        self._wait_imu_ready(label)
        rclpy.spin_once(self, timeout_sec=0.0)
        y0 = self._imu_yaw
        if y0 is None:
            raise RuntimeError('IMU yaw unavailable')
        self.turn_to_imu_yaw(wrap_pi(y0 + delta), label)

    def face_toward(self, wx: float, wy: float, label: str = '') -> None:
        rclpy.spin_once(self, timeout_sec=0.0)
        rx, ry, _ = self.world_pose()
        self.turn_to_yaw(math.atan2(wy - ry, wx - rx), label)

    def _pre_drop_drive_hardcoded(self) -> None:
        """After ``turn_by_imu(π)``: IMU face +map Y, then constant ``linear.x`` until ``wy`` ≥ 8.5.

        Relative π does not land on +Y in the IMU frame (often ~−π → along-logic dies); this
        uses a fixed yaw π/2 then stops on ``world_pose`` Y only (same frame as toy row).
        """
        m = self._m()
        tol = float(m.get('pre_drop_pos_tol', 0.055))
        self._wait_imu_ready('pre-drop')
        # Hardcoded column stop (meters); keep in sync with ``pre_drop_y`` in yaml.
        _GOAL_Y = 8.5
        self.turn_to_imu_yaw(math.pi / 2.0, 'pre-drop face +Y')
        self._halt_stop()
        v_max = float(m.get('v_max', 0.28))
        dt = self._period()
        tout = float(m.get('drive_timeout_sec', 120.0))
        use_cap = bool(m.get('drive_odom_step_cap', True))
        brake_goal = bool(m.get('drive_brake_at_goal', True))
        end = self.get_clock().now() + Duration(seconds=tout)
        self.get_logger().info(f'Hard +Y drive until wy>={_GOAL_Y - tol:.3f}')
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.0)
            _, wy, _ = self.world_pose()
            rem = _GOAL_Y - tol - wy
            if rem <= 0.0:
                self.stop()
                if brake_goal:
                    self._brake()
                self.get_logger().info(f'Pre-drop done wy={wy:.4f}')
                return
            tw = Twist()
            tw.angular.z = 0.0
            v = v_max
            if use_cap:
                cap_v = rem / max(dt, 1e-4)
                v = min(v, cap_v)
            tw.linear.x = v
            self._send_cmd(tw)
            self._spin_sleep(dt)
        self.stop()
        self.get_logger().warn('Pre-drop drive timeout')

    def drive_straight_to(
        self,
        wx: float,
        wy: float,
        label: str = '',
        *,
        pos_tol: Optional[float] = None,
    ) -> None:
        """Face-aligned leg: ``linear.x`` only; stop when ``hypot(dx,dy) < tol``."""
        m = self._m()
        v_max = float(m.get('v_max', 0.28))
        kv = float(m.get('approach_k', 0.45))
        tol = float(pos_tol) if pos_tol is not None else float(m.get('pos_tol', 0.14))
        slow_r = float(m.get('slow_radius_m', 0.55))
        v_near = float(m.get('v_near_max', 0.08))
        v_crawl = float(m.get('v_crawl_min', 0.03))
        tout = float(m.get('drive_timeout_sec', 120.0))
        use_cap = bool(m.get('drive_odom_step_cap', True))
        brake_goal = bool(m.get('drive_brake_at_goal', True))
        dt = self._period()
        end = self.get_clock().now() + Duration(seconds=tout)
        self.get_logger().info(f'Stride → ({wx:.2f},{wy:.2f}) tol={tol:.3f} {label}')
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.0)
            rx, ry, _ = self.world_pose()
            dx, dy = wx - rx, wy - ry
            dist = math.hypot(dx, dy)
            if dist < tol:
                self.stop()
                if brake_goal:
                    self._brake()
                self.get_logger().info(f'At goal {label}')
                return
            tw = Twist()
            tw.angular.z = 0.0
            v = min(v_max, kv * dist)
            if slow_r > 0.0 and dist < slow_r:
                slp = dist / slow_r
                v = min(v, max(v_near * slp, v_crawl))
            if use_cap and dist > tol:
                cap_v = max(0.0, dist - tol) / max(dt, 1e-4)
                v = min(v, cap_v)
            tw.linear.x = v
            self._send_cmd(tw)
            self._spin_sleep(dt)
        self.stop()
        self.get_logger().warn(f'Drive timeout {label}')

    def drive_to(
        self,
        wx: float,
        wy: float,
        label: str = '',
        *,
        pos_tol: Optional[float] = None,
    ) -> None:
        self.face_toward(wx, wy, f'face {label}')
        self.drive_straight_to(wx, wy, label, pos_tol=pos_tol)

    def _arm_set(self, sh: float, el: float, wr: float) -> None:
        self._pub_sh.publish(Float64(data=sh))
        self._pub_el.publish(Float64(data=el))
        self._pub_wr.publish(Float64(data=wr))
        for _ in range(3):
            rclpy.spin_once(self, timeout_sec=0.0)

    def _stage(self, key: str) -> None:
        arm = self._cfg['arm']
        st = arm['stages'][key]
        sec = float(arm.get('stage_settle_sec', 0.72))
        self._arm_set(float(st['shoulder']), float(st['elbow']), float(st['wrist']))
        self._spin_sleep(sec)

    def pick(self, name: str, *, drive_in: bool = True) -> None:
        o = OBJECTS.get(name)
        if not o:
            self.get_logger().error(f'Unknown object {name}')
            return
        arm = self._cfg['arm']
        ox, oy = o[0], o[1]
        self.face_toward(ox, oy, f'pick {name} face')
        rx, ry, _ = self.world_pose()
        dx, dy = ox - rx, oy - ry
        d = math.hypot(dx, dy) or 1e-6
        so = float(arm.get('pick_standoff_m', 0.22))
        if drive_in:
            gx = ox - (dx / d) * so
            gy = oy - (dy / d) * so
            # Same bearing as facing the object; no extra spin before the short creep.
            self.drive_straight_to(gx, gy, f'pick {name} in')
        self.face_toward(ox, oy, f'pick {name} aim')

        self._stage('stage1')
        self._stage('stage2')
        self._stage('stage3')
        ph = float(arm.get('pick_pose_stream_sec', 0.45))
        phz = float(arm.get('pick_pose_stream_hz', 28.0))
        if ph > 0.0 and phz > 0.0:
            p = arm['stages']['stage3']
            per = 1.0 / max(5.0, phz)
            t_end = self.get_clock().now() + Duration(seconds=ph)
            while rclpy.ok() and self.get_clock().now() < t_end:
                self._arm_set(float(p['shoulder']), float(p['elbow']), float(p['wrist']))
                self._spin_sleep(per)

        top = ATTACH_TOPIC.get(name)
        if top:
            self._spin_sleep(float(arm.get('attach_pre_delay_sec', 0.45)))
            nburst = max(1, int(arm.get('attach_burst_count', 5)))
            bper = float(arm.get('attach_burst_period_sec', 0.18))
            for i in range(nburst):
                self._attach_pulse(top)
                if i + 1 < nburst:
                    self._spin_sleep(bper)

        wait_a = float(arm.get('attach_wait_sec', 2.5))
        ahz = float(arm.get('attach_hz', 5.0))
        aresend = float(arm.get('attach_resend_sec', 0.45))
        p3 = arm['stages']['stage3']
        per = 1.0 / max(5.0, ahz)
        nxt_arm = time.monotonic()
        nxt_attach = time.monotonic()
        t_end = self.get_clock().now() + Duration(seconds=wait_a)
        while rclpy.ok() and self.get_clock().now() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            if now >= nxt_arm:
                self._arm_set(float(p3['shoulder']), float(p3['elbow']), float(p3['wrist']))
                nxt_arm = now + per
            if top and now >= nxt_attach:
                self._attach_pulse(top)
                nxt_attach = now + aresend
        self._spin_sleep(float(arm.get('pick_magnet_pause', 0.5)))
        self._stage('stage2')
        self._stage('stage1')
        self._carry = True
        self.get_logger().info(f'Pick done {name}')

    def drop(self) -> None:
        arm = self._cfg['arm']
        dz = self._cfg.get('drop_zone', {})
        bx = float(dz.get('center_x', DEFAULT_BOX[0]))
        by = float(dz.get('center_y', DEFAULT_BOX[1]))
        self.face_toward(bx, by, 'drop face bin')
        self._stage('stage2')
        p3 = arm['stages']['stage3']
        self._arm_set(float(p3['shoulder']), float(p3['elbow']), float(p3['wrist']))
        self._spin_sleep(float(arm.get('settle_drop', 1.0)))
        dh = float(arm.get('drop_pose_stream_sec', 0.35))
        dhz = float(arm.get('drop_pose_stream_hz', 28.0))
        if dh > 0.0 and dhz > 0.0:
            per = 1.0 / max(5.0, dhz)
            t_end = self.get_clock().now() + Duration(seconds=dh)
            while rclpy.ok() and self.get_clock().now() < t_end:
                self._arm_set(float(p3['shoulder']), float(p3['elbow']), float(p3['wrist']))
                self._spin_sleep(per)
        self._magnet_off_pulse()
        self._spin_sleep(float(arm.get('drop_magnet_pause', 0.4)))
        self._stage('stage2')
        self._stage('stage1')
        self._carry = False
        self.get_logger().info('Drop done')

    def run_mission(self) -> None:
        self.get_logger().info('Wait odom…')
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < 60.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._odom_n >= 3:
                break
        self._spin_sleep(0.5)
        x, y, yw = self.world_pose()
        self.get_logger().info(
            f'World ({x:.3f},{y:.3f}) yaw={yw:.3f} spawn=({self._spawn[0]},{self._spawn[1]}) '
            f'spawn_yaw={self._spawn_yaw:.3f}'
        )

        pi = math.pi
        m = self._m()
        self._wait_imu_ready('(mission start)')

        try:
            # 1) Row → halt → pick
            self.drive_to(7.5, 5.4, 'toy row')
            self._halt_stop()
            self.pick('toy_block_1', drive_in=False)
            self._halt_stop()
            # 2) 180° in place (IMU only)
            self.turn_by_imu(pi, '180 after pick')
            self._halt_stop()
            # 3) Hardcoded +Y leg to (7.5, 8.5): see _pre_drop_drive_hardcoded.
            self._pre_drop_drive_hardcoded()
            self._halt_stop()
            # 4) 90° (IMU only) → drop
            d90 = float(m.get('pre_drop_turn_rad', pi / 2.0))
            self.turn_by_imu(d90, '90 before drop')
            self._halt_stop()
            self._pause()
            self.drop()

            self.drive_to(1.75, 9.0, 'can halt')
            self._pause()
            self.pick('can')

            self.drive_to(7.5, 9.0, 'after can → bin')
            self._pause()
            self.drop()

            self.drive_to(7.5, 2.0, 'to corridor')
            self._pause()
            self.drive_to(1.75, 2.0, 'ball halt')
            self._pause()
            self.pick('ball')

            self.drive_to(7.5, 2.0, 'return y=2')
            self._pause()
            self.drive_to(7.5, 9.0, 'final bin row')
            self._pause()
            self.drop()

            self.turn_to_yaw(pi, 'park π')
        finally:
            self.stop()
            if self._carry:
                self.get_logger().warn('Still carrying — magnet_off')
                self._magnet_off_pulse()
                self._carry = False
        self.get_logger().info('Mission finished')


def main() -> None:
    rclpy.init()
    n = TaskNode()
    try:
        n.run_mission()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        n.stop()
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
