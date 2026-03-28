"""TaskNode — robot API and mission runner."""
from math import atan2, cos, sin, hypot, pi

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu, Image
from std_msgs.msg import Float64, Empty

# World-frame positions
BLOCK_XY = (7.5, 5.0)
CAN_XY = (1.5, 9.0)
BALL_XY = (1.5, 2.0)
BIN_XY = (7.5, 9.0)

# Arm stages
ARM_CARRY = (-0.00006, 0.0, -0.00017)
ARM_STAGE1 = (-0.00006, 0.0, -0.00017)
ARM_STAGE2 = (-0.00006, 0.0, -1.309)
ARM_STAGE3 = (-0.63, 0.0, -1.309)
ARM_STAGE4 = (0.2, 0.0, -1.309)

# Motion tuning
LINEAR_SPEED = 0.18
ANGULAR_SPEED = 1.0
POS_TOLERANCE = 0.12
HEADING_TOLERANCE = 0.08
PICK_STANDOFF = 0.60
BRAKE_DISTANCE = 0.15

# Bug2 tuning
BUG2_WALL_DIST = 0.35
BUG2_WALL_SPEED = 0.12
BUG2_WALL_GAIN = 1.2
BUG2_OBSTACLE_THRESH = 0.40
BUG2_LEAVE_THRESH = 0.06
BUG2_FRONT_CLEAR = 0.60


class TaskNode(Node):

    def __init__(self):
        super().__init__('task_node')

        # Parameters
        self.declare_parameter('mission_id', 1)
        self.declare_parameter('spawn_x', 7.5)
        self.declare_parameter('spawn_y', 9.0)
        self.declare_parameter('spawn_yaw', -1.5708)

        self._spawn_x = self.get_parameter('spawn_x').value
        self._spawn_y = self.get_parameter('spawn_y').value
        self._spawn_yaw = self.get_parameter('spawn_yaw').value

        # String conversion
        if isinstance(self._spawn_x, str):
            self._spawn_x = float(self._spawn_x)
        if isinstance(self._spawn_y, str):
            self._spawn_y = float(self._spawn_y)
        if isinstance(self._spawn_yaw, str):
            self._spawn_yaw = float(self._spawn_yaw)

        # Subscriptions
        self.create_subscription(
            Odometry, '/odom', self._odom_cb, QoSProfile(depth=50))
        self.create_subscription(
            LaserScan, '/scan', self._scan_cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(
            Imu, '/imu/data', self._imu_cb,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(
            Image, '/camera/image_raw', self._img_cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

        # Publishers
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        reliable_10 = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._arm_shoulder_pub = self.create_publisher(
            Float64, '/arm_cmd_shoulder', reliable_10)
        self._arm_elbow_pub = self.create_publisher(
            Float64, '/arm_cmd_elbow', reliable_10)
        self._arm_wrist_pub = self.create_publisher(
            Float64, '/arm_cmd_wrist', reliable_10)

        self._attach_block_pub = self.create_publisher(
            Empty, '/toy_block_1/attach', 10)
        self._attach_can_pub = self.create_publisher(
            Empty, '/can/attach', 10)
        self._attach_ball_pub = self.create_publisher(
            Empty, '/ball/attach', 10)
        self._magnet_off_pub = self.create_publisher(
            Empty, '/magnet_off', 10)

        # Internal state
        self._ox = None
        self._oy = None
        self._oyaw = None
        self._imu_yaw = None
        self._imu_yaw_origin = None
        self._scan_ranges = None
        self._scan_front_min = float('inf')
        self._scan_right_min = float('inf')
        self._scan_left_min = float('inf')
        self._carrying = False
        self._img_count = 0

        # Status timer
        self.create_timer(1.0, self._status_line)

        self.get_logger().info(
            f'TaskNode ready  spawn=({self._spawn_x}, {self._spawn_y}, '
            f'{self._spawn_yaw})')

    # Callbacks
    def _odom_cb(self, msg):
        self._ox = msg.pose.pose.position.x
        self._oy = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._oyaw = atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _imu_cb(self, msg):
        q = msg.orientation
        yaw = atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self._imu_yaw_origin is None:
            self._imu_yaw_origin = yaw
        self._imu_yaw = yaw

    def _scan_cb(self, msg):
        self._scan_ranges = msg.ranges
        n = len(msg.ranges)

        def valid(i):
            r = msg.ranges[i]
            if msg.range_min <= r <= msg.range_max:
                return r
            return float('inf')

        center = n // 2
        window = n // 12
        front_vals = [valid(i) for i in range(center - window, center + window)]
        self._scan_front_min = min(front_vals) if front_vals else float('inf')

        right_vals = [valid(i) for i in range(0, n // 4)]
        self._scan_right_min = min(right_vals) if right_vals else float('inf')

        left_vals = [valid(i) for i in range(3 * n // 4, n)]
        self._scan_left_min = min(left_vals) if left_vals else float('inf')

    def _img_cb(self, msg):
        self._img_count += 1

    def _status_line(self):
        pose = self.world_pose()
        if pose is None:
            return
        mid = self.get_parameter('mission_id').value
        wx, wy, wyaw = pose
        imu = self._imu_yaw if self._imu_yaw is not None else float('nan')
        self.get_logger().info(
            f'M={mid} world=({wx:.2f},{wy:.2f}) yaw={wyaw:.2f} '
            f'imu={imu:.2f} carry={self._carrying} imgs={self._img_count}')

    # Coordinate helpers
    def world_pose(self):
        if self._ox is None:
            return None
        sy = sin(self._spawn_yaw)
        cy = cos(self._spawn_yaw)
        wx = self._spawn_x + self._ox * cy - self._oy * sy
        wy = self._spawn_y + self._ox * sy + self._oy * cy
        if self._imu_yaw is not None and self._imu_yaw_origin is not None:
            imu_delta = self._wrap_pi(self._imu_yaw - self._imu_yaw_origin)
            wyaw = self._wrap_pi(self._spawn_yaw + imu_delta)
        else:
            wyaw = self._wrap_pi(self._spawn_yaw + self._oyaw)
        return (wx, wy, wyaw)

    @staticmethod
    def _wrap_pi(a):
        while a > pi:
            a -= 2 * pi
        while a < -pi:
            a += 2 * pi
        return a

    # Low-level motion
    def _publish_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self._cmd_pub.publish(msg)

    def _spin(self):
        rclpy.spin_once(self, timeout_sec=0.05)

    def stop(self):
        self._publish_cmd(0.0, 0.0)

    def halt(self, seconds=0.3):
        self.stop()
        t0 = self.get_clock().now()
        while rclpy.ok():
            self._spin()
            elapsed = (self.get_clock().now() - t0).nanoseconds * 1e-9
            if elapsed >= seconds:
                break
        self.stop()

    def _sleep(self, seconds):
        t0 = self.get_clock().now()
        while rclpy.ok():
            self._spin()
            elapsed = (self.get_clock().now() - t0).nanoseconds * 1e-9
            if elapsed >= seconds:
                return

    # Sensor waits
    def wait_for_odom(self, timeout=30.0):
        t0 = self.get_clock().now()
        while rclpy.ok() and self._ox is None:
            self._spin()
            elapsed = (self.get_clock().now() - t0).nanoseconds * 1e-9
            if elapsed > timeout:
                self.get_logger().error('Timed out waiting for /odom')
                return
        self.get_logger().info('Odom received')

    def wait_for_imu(self, timeout=15.0):
        t0 = self.get_clock().now()
        while rclpy.ok() and self._imu_yaw is None:
            self._spin()
            elapsed = (self.get_clock().now() - t0).nanoseconds * 1e-9
            if elapsed > timeout:
                self.get_logger().warn(
                    'IMU not available, will use odom for turns')
                return
        self.get_logger().info('IMU received')

    # Turning
    def face_toward(self, tx, ty):
        while rclpy.ok():
            pose = self.world_pose()
            if pose is None:
                self._spin()
                continue
            wx, wy, wyaw = pose
            bearing = atan2(ty - wy, tx - wx)
            err = self._wrap_pi(bearing - wyaw)
            if abs(err) < HEADING_TOLERANCE:
                self.stop()
                return
            w = ANGULAR_SPEED if err > 0 else -ANGULAR_SPEED
            self._publish_cmd(0.0, w)
            self._spin()
        self.stop()

    def turn_to_world_yaw(self, target_wyaw):
        while rclpy.ok():
            pose = self.world_pose()
            if pose is None:
                self._spin()
                continue
            err = self._wrap_pi(target_wyaw - pose[2])
            if abs(err) < HEADING_TOLERANCE:
                self.stop()
                return
            w = ANGULAR_SPEED if err > 0 else -ANGULAR_SPEED
            self._publish_cmd(0.0, w)
            self._spin()
        self.stop()

    def turn_by_imu(self, delta_rad):
        if self._imu_yaw is None:
            pose = self.world_pose()
            if pose is None:
                return
            target_wyaw = self._wrap_pi(pose[2] + delta_rad)
            self.turn_to_world_yaw(target_wyaw)
            return

        start_imu = self._imu_yaw
        target_imu = self._wrap_pi(start_imu + delta_rad)

        while rclpy.ok():
            self._spin()
            if self._imu_yaw is None:
                continue
            err = self._wrap_pi(target_imu - self._imu_yaw)
            if abs(err) < HEADING_TOLERANCE:
                self.stop()
                return
            w = ANGULAR_SPEED if err > 0 else -ANGULAR_SPEED
            self._publish_cmd(0.0, w)
        self.stop()

    def turn_to_imu_yaw(self, target_imu_yaw):
        if self._imu_yaw is None:
            self.get_logger().warn('No IMU, using odom turn')
            return
        while rclpy.ok():
            self._spin()
            if self._imu_yaw is None:
                continue
            err = self._wrap_pi(target_imu_yaw - self._imu_yaw)
            if abs(err) < HEADING_TOLERANCE:
                self.stop()
                return
            w = ANGULAR_SPEED if err > 0 else -ANGULAR_SPEED
            self._publish_cmd(0.0, w)
        self.stop()

    def settle_heading(self, target_wyaw, tolerance=0.01):
        """Micro-correct heading."""
        self.get_logger().info(
            f'Settling heading to {target_wyaw:.4f} rad '
            f'(tol={tolerance:.3f})')
        while rclpy.ok():
            pose = self.world_pose()
            if pose is None:
                self._spin()
                continue
            err = self._wrap_pi(target_wyaw - pose[2])
            if abs(err) < tolerance:
                self.stop()
                self.get_logger().info(
                    f'Heading locked at {pose[2]:.4f} rad')
                return
            w = 0.4 * err
            w = max(min(w, 0.3), -0.3)
            self._publish_cmd(0.0, w)
            self._spin()
        self.stop()

    # Driving
    def drive_straight(self, distance, hold_wyaw=None):
        """IMU heading hold drive."""
        if self._ox is None:
            return
        start_ox, start_oy = self._ox, self._oy

        if (hold_wyaw is not None
                and self._imu_yaw_origin is not None):
            target_imu = self._wrap_pi(
                self._imu_yaw_origin + (hold_wyaw - self._spawn_yaw))
        else:
            target_imu = self._imu_yaw

        self.get_logger().info(
            f'drive_straight {distance:.2f}m  target_imu={target_imu:.4f}')

        while rclpy.ok():
            self._spin()
            if self._ox is None:
                continue
            traveled = hypot(self._ox - start_ox, self._oy - start_oy)
            remaining = distance - traveled

            if remaining < POS_TOLERANCE:
                self.stop()
                return

            speed = LINEAR_SPEED
            if remaining < BRAKE_DISTANCE:
                speed = max(LINEAR_SPEED * (remaining / BRAKE_DISTANCE), 0.04)

            omega = 0.0
            if target_imu is not None and self._imu_yaw is not None:
                omega = 2.0 * self._wrap_pi(target_imu - self._imu_yaw)

            self._publish_cmd(speed, omega)
        self.stop()

    def drive_straight_to(self, tx, ty):
        while rclpy.ok():
            pose = self.world_pose()
            if pose is None:
                self._spin()
                continue
            wx, wy, wyaw = pose
            dist = hypot(tx - wx, ty - wy)

            if dist < POS_TOLERANCE:
                self.stop()
                return

            bearing = atan2(ty - wy, tx - wx)
            heading_err = self._wrap_pi(bearing - wyaw)

            speed = LINEAR_SPEED
            if dist < BRAKE_DISTANCE:
                speed = max(LINEAR_SPEED * (dist / BRAKE_DISTANCE), 0.04)

            omega = 1.5 * heading_err
            self._publish_cmd(speed, omega)
            self._spin()
        self.stop()

    def drive_to(self, tx, ty):
        self.face_toward(tx, ty)
        self.halt()
        self.drive_straight_to(tx, ty)

    # Bug2 navigation
    def bug2_drive_to(self, tx, ty):
        self.get_logger().info(f'Bug2 driving to ({tx:.1f}, {ty:.1f})')

        pose = self.world_pose()
        while pose is None:
            self._spin()
            pose = self.world_pose()
        start_x, start_y = pose[0], pose[1]

        mode = 'GO_TO_GOAL'
        hit_point = None

        while rclpy.ok():
            pose = self.world_pose()
            if pose is None:
                self._spin()
                continue
            wx, wy, wyaw = pose
            dist_to_goal = hypot(tx - wx, ty - wy)

            if dist_to_goal < POS_TOLERANCE:
                self.stop()
                self.get_logger().info('Bug2: arrived')
                return

            if mode == 'GO_TO_GOAL':
                if self._scan_front_min < BUG2_OBSTACLE_THRESH:
                    mode = 'WALL_FOLLOW'
                    hit_point = (wx, wy)
                    self.get_logger().info(
                        f'Bug2: obstacle at ({wx:.2f},{wy:.2f}), '
                        f'wall-following')
                    continue

                bearing = atan2(ty - wy, tx - wx)
                heading_err = self._wrap_pi(bearing - wyaw)

                speed = LINEAR_SPEED
                if dist_to_goal < BRAKE_DISTANCE:
                    speed = max(
                        LINEAR_SPEED * (dist_to_goal / BRAKE_DISTANCE), 0.04)

                self._publish_cmd(speed, 1.5 * heading_err)

            elif mode == 'WALL_FOLLOW':
                side_dist = self._scan_right_min
                side_err = BUG2_WALL_DIST - side_dist
                omega = BUG2_WALL_GAIN * side_err

                if self._scan_front_min < BUG2_OBSTACLE_THRESH:
                    self._publish_cmd(0.0, ANGULAR_SPEED)
                else:
                    self._publish_cmd(BUG2_WALL_SPEED, omega)

                m_line_dist = self._point_to_line_dist(
                    wx, wy, (start_x, start_y), (tx, ty))
                dist_at_hit = hypot(tx - hit_point[0], ty - hit_point[1])
                moved_from_hit = hypot(wx - hit_point[0], wy - hit_point[1])
                front_clear = self._scan_front_min > BUG2_FRONT_CLEAR

                if (m_line_dist < BUG2_LEAVE_THRESH
                        and dist_to_goal < dist_at_hit
                        and moved_from_hit > 0.3
                        and front_clear):
                    mode = 'GO_TO_GOAL'
                    self.get_logger().info(
                        f'Bug2: rejoining m-line ({wx:.2f},{wy:.2f})')

            self._spin()

        self.stop()

    @staticmethod
    def _point_to_line_dist(px, py, line_start, line_end):
        x1, y1 = line_start
        x2, y2 = line_end
        num = abs((y2 - y1) * px - (x2 - x1) * py + x2 * y1 - y2 * x1)
        den = hypot(y2 - y1, x2 - x1)
        if den < 1e-9:
            return hypot(px - x1, py - y1)
        return num / den

    # Arm control
    def publish_arm(self, shoulder, elbow, wrist):
        s, e, w = Float64(), Float64(), Float64()
        s.data = float(shoulder)
        e.data = float(elbow)
        w.data = float(wrist)
        self._arm_shoulder_pub.publish(s)
        self._arm_elbow_pub.publish(e)
        self._arm_wrist_pub.publish(w)

    def arm_carry(self):
        self.publish_arm(*ARM_CARRY)

    # Pick and drop
    def _get_attach_pub(self, topic):
        mapping = {
            '/toy_block_1/attach': self._attach_block_pub,
            '/can/attach': self._attach_can_pub,
            '/ball/attach': self._attach_ball_pub,
        }
        return mapping[topic]

    def pick(self, attach_topic, obj_x, obj_y):
        self.get_logger().info(f'Picking at ({obj_x:.1f}, {obj_y:.1f})')

        self.face_toward(obj_x, obj_y)
        self.halt()

        while rclpy.ok():
            pose = self.world_pose()
            if pose is None:
                self._spin()
                continue
            wx, wy, wyaw = pose
            dist = hypot(obj_x - wx, obj_y - wy)

            if dist <= PICK_STANDOFF:
                self.stop()
                break

            bearing = atan2(obj_y - wy, obj_x - wx)
            heading_err = self._wrap_pi(bearing - wyaw)

            brake_zone = PICK_STANDOFF + 0.20
            if dist < brake_zone:
                frac = (dist - PICK_STANDOFF) / 0.20
                speed = max(0.03, LINEAR_SPEED * 0.3 * frac)
            else:
                speed = min(LINEAR_SPEED, 0.3 * dist)

            self._publish_cmd(speed, 1.0 * heading_err)
            self._spin()

        self.halt(0.5)

        self.publish_arm(*ARM_STAGE1)
        self._sleep(0.4)
        self.publish_arm(*ARM_STAGE2)
        self._sleep(0.4)
        self.publish_arm(*ARM_STAGE3)
        self._sleep(0.5)

        attach_pub = self._get_attach_pub(attach_topic)
        for _ in range(5):
            attach_pub.publish(Empty())
            self._sleep(0.1)
        self._sleep(0.5)

        self.publish_arm(*ARM_STAGE2)
        self._sleep(0.3)
        self.publish_arm(*ARM_STAGE1)
        self._sleep(0.3)
        self.arm_carry()
        self._sleep(0.3)

        self._carrying = True
        self.get_logger().info('Pick complete')

    def drop(self):
        self.get_logger().info('Dropping object')

        self.publish_arm(*ARM_STAGE1)
        self._sleep(0.3)
        self.publish_arm(*ARM_STAGE2)
        self._sleep(0.3)
        self.publish_arm(*ARM_STAGE4)
        self._sleep(0.5)

        for _ in range(5):
            self._magnet_off_pub.publish(Empty())
            self._sleep(0.1)
        self._sleep(0.3)

        self.publish_arm(*ARM_STAGE2)
        self._sleep(0.3)
        self.publish_arm(*ARM_STAGE1)
        self._sleep(0.3)
        self.arm_carry()
        self._sleep(0.3)

        self._carrying = False
        self.get_logger().info('Drop complete')

    def magnet_off_safe(self):
        if self._carrying:
            for _ in range(5):
                self._magnet_off_pub.publish(Empty())
                self._sleep(0.1)
            self._carrying = False

    # Mission runner
    def run_mission(self):
        from driftbot_task.missions import run
        mission_id = self.get_parameter('mission_id').value
        if isinstance(mission_id, str):
            mission_id = int(mission_id)
        self.get_logger().info(f'=== Starting Mission {mission_id} ===')
        try:
            run(self, mission_id)
        except Exception as e:
            self.get_logger().error(f'Mission failed: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
        finally:
            self.stop()
            self.magnet_off_safe()
        self.get_logger().info(f'=== Mission {mission_id} Complete ===')


def main(args=None):
    rclpy.init(args=args)
    node = TaskNode()
    try:
        node.run_mission()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.magnet_off_safe()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
