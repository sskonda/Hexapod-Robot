#!/usr/bin/env python3

import copy
import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster

from .calibration_store import JOINT_NAMES, servo_angles_from_leg_coordinates
from .kalman_filter import AngleKalmanFilter


BASE_FOOTPOINTS = [
    [137.1, 189.4, 0.0],
    [225.0, 0.0, 0.0],
    [137.1, -189.4, 0.0],
    [-137.1, -189.4, 0.0],
    [-225.0, 0.0, 0.0],
    [-137.1, 189.4, 0.0],
]

LEG_MOUNT_ANGLES_DEG = [54.0, 0.0, -54.0, -126.0, 180.0, 126.0]
LEG_X_OFFSETS_MM = [94.0, 85.0, 94.0, 94.0, 85.0, 94.0]
LEG_Z_OFFSET_MM = 50.0
WAVE_SEQUENCE = [5, 2, 1, 0, 3, 4]


@dataclass
class GaitCycleState:
    gait_name: str
    total_steps: int
    lift_step_mm: float
    planar_delta_mm: list
    baseline_points: list
    points: list
    step_index: int = 0
    odom_vx_mps: float = 0.0
    odom_vy_mps: float = 0.0
    odom_vtheta_rps: float = 0.0


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def map_value(value, from_low, from_high, to_low, to_high):
    return (to_high - to_low) * (value - from_low) / (from_high - from_low) + to_low


def matrix_multiply(a, b):
    return [
        [
            a[row][0] * b[0][col] + a[row][1] * b[1][col] + a[row][2] * b[2][col]
            for col in range(3)
        ]
        for row in range(3)
    ]


def matrix_vector_multiply(matrix, vector):
    return [
        matrix[row][0] * vector[0] + matrix[row][1] * vector[1] + matrix[row][2] * vector[2]
        for row in range(3)
    ]


class LocomotionNode(Node):
    def __init__(self):
        super().__init__('locomotion')

        self.declare_parameter('use_imu', True)
        self.declare_parameter('balance_gain', 0.1)
        self.declare_parameter('gait', 'tripod')
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('default_body_height_mm', -25.0)
        self.declare_parameter('step_height_mm', 40.0)
        self.declare_parameter('max_linear_speed_mps', 0.06)
        self.declare_parameter('max_lateral_speed_mps', 0.05)
        self.declare_parameter('max_yaw_rate_rad_s', 0.9)
        self.declare_parameter('max_stride_x_mm', 35.0)
        self.declare_parameter('max_stride_y_mm', 35.0)
        self.declare_parameter('max_turn_per_cycle_deg', 25.0)
        self.declare_parameter('max_body_x_shift_mm', 40.0)
        self.declare_parameter('max_body_y_shift_mm', 40.0)
        self.declare_parameter('max_body_z_shift_mm', 20.0)
        self.declare_parameter('max_roll_deg', 15.0)
        self.declare_parameter('max_pitch_deg', 15.0)
        self.declare_parameter('max_yaw_deg', 15.0)
        self.declare_parameter('yaw_correction_gain', 0.0)
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_odom_tf', True)
        self.use_imu = bool(self.get_parameter('use_imu').value)
        self.balance_gain = float(self.get_parameter('balance_gain').value)
        self.gait = str(self.get_parameter('gait').value).strip().lower()
        self.control_rate_hz = max(1.0, float(self.get_parameter('control_rate_hz').value))
        self.command_timeout_sec = max(0.0, float(self.get_parameter('command_timeout_sec').value))
        self.default_body_height_mm = float(self.get_parameter('default_body_height_mm').value)
        self.step_height_mm = max(1.0, float(self.get_parameter('step_height_mm').value))
        self.max_linear_speed_mps = max(0.001, float(self.get_parameter('max_linear_speed_mps').value))
        self.max_lateral_speed_mps = max(0.001, float(self.get_parameter('max_lateral_speed_mps').value))
        self.max_yaw_rate_rad_s = max(0.001, float(self.get_parameter('max_yaw_rate_rad_s').value))
        self.max_stride_x_mm = max(1.0, float(self.get_parameter('max_stride_x_mm').value))
        self.max_stride_y_mm = max(1.0, float(self.get_parameter('max_stride_y_mm').value))
        self.max_turn_per_cycle_deg = max(1.0, float(self.get_parameter('max_turn_per_cycle_deg').value))
        self.max_body_x_shift_mm = max(0.0, float(self.get_parameter('max_body_x_shift_mm').value))
        self.max_body_y_shift_mm = max(0.0, float(self.get_parameter('max_body_y_shift_mm').value))
        self.max_body_z_shift_mm = max(0.0, float(self.get_parameter('max_body_z_shift_mm').value))
        self.max_roll_deg = max(0.0, float(self.get_parameter('max_roll_deg').value))
        self.max_pitch_deg = max(0.0, float(self.get_parameter('max_pitch_deg').value))
        self.max_yaw_deg = max(0.0, float(self.get_parameter('max_yaw_deg').value))
        self.yaw_correction_gain = float(self.get_parameter('yaw_correction_gain').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.odom_frame_id = str(self.get_parameter('odom_frame_id').value)
        self.base_frame_id = str(self.get_parameter('base_frame_id').value)
        self.publish_odom_tf = bool(self.get_parameter('publish_odom_tf').value)

        if self.gait not in ('tripod', 'wave'):
            self.get_logger().warn(f'Unsupported gait "{self.gait}", defaulting to tripod')
            self.gait = 'tripod'

        self.cmd_linear_x_mps = 0.0
        self.cmd_linear_y_mps = 0.0
        self.cmd_yaw_rate_rad_s = 0.0
        self.last_motion_cmd_time = self.get_clock().now()

        self.body_pose_deg = [0.0, 0.0, 0.0]
        self.body_shift_mm = [0.0, 0.0, 0.0]

        self.imu_roll_deg = 0.0
        self.imu_pitch_deg = 0.0
        self.imu_yaw_rate_rps = 0.0
        self.roll_filter = AngleKalmanFilter()
        self.pitch_filter = AngleKalmanFilter()
        self.last_imu_stamp = None
        self.last_invalid_warn_time = 0.0
        self.gait_state = None

        self.odom_x_m = 0.0
        self.odom_y_m = 0.0
        self.odom_theta_rad = 0.0

        self.target_publisher = self.create_publisher(JointState, 'servo_targets', 10)
        self.odom_publisher = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10,
        )
        self.body_pose_subscription = self.create_subscription(
            Vector3,
            'body_pose',
            self.body_pose_callback,
            10,
        )
        self.body_shift_subscription = self.create_subscription(
            Vector3,
            'body_shift',
            self.body_shift_callback,
            10,
        )
        self.imu_subscription = None

        if self.use_imu:
            self.imu_subscription = self.create_subscription(
                Imu,
                'imu/data_raw',
                self.imu_callback,
                10,
            )

        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.control_loop)

        self.get_logger().info('Locomotion controller ready')
        self.get_logger().info('Topics: /cmd_vel, /body_pose, /body_shift, /servo_targets')
        self.get_logger().info(
            f'cmd_vel uses m/s and rad/s. body_pose uses roll/pitch/yaw in degrees. '
            f'body_shift uses x/y/z in mm. Publishing odom on /{self.odom_topic} '
            f'with TF {"enabled" if self.publish_odom_tf else "disabled"}.'
        )

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_linear_x_mps = clamp(msg.linear.x, -self.max_linear_speed_mps, self.max_linear_speed_mps)
        self.cmd_linear_y_mps = clamp(msg.linear.y, -self.max_lateral_speed_mps, self.max_lateral_speed_mps)
        self.cmd_yaw_rate_rad_s = clamp(msg.angular.z, -self.max_yaw_rate_rad_s, self.max_yaw_rate_rad_s)
        self.last_motion_cmd_time = self.get_clock().now()

    def body_pose_callback(self, msg: Vector3):
        self.body_pose_deg[0] = clamp(msg.x, -self.max_roll_deg, self.max_roll_deg)
        self.body_pose_deg[1] = clamp(msg.y, -self.max_pitch_deg, self.max_pitch_deg)
        self.body_pose_deg[2] = clamp(msg.z, -self.max_yaw_deg, self.max_yaw_deg)

    def body_shift_callback(self, msg: Vector3):
        self.body_shift_mm[0] = clamp(msg.x, -self.max_body_x_shift_mm, self.max_body_x_shift_mm)
        self.body_shift_mm[1] = clamp(msg.y, -self.max_body_y_shift_mm, self.max_body_y_shift_mm)
        self.body_shift_mm[2] = clamp(msg.z, -self.max_body_z_shift_mm, self.max_body_z_shift_mm)

    def imu_callback(self, msg: Imu):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        accel_roll = math.degrees(math.atan2(ay, az))
        accel_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))

        gyro_roll_rate = math.degrees(msg.angular_velocity.x)
        gyro_pitch_rate = math.degrees(msg.angular_velocity.y)

        if self.last_imu_stamp is None:
            dt = 0.1  # matches the 10 Hz publish rate in imu_publisher.py
        else:
            dt = (
                msg.header.stamp.sec - self.last_imu_stamp.sec
                + (msg.header.stamp.nanosec - self.last_imu_stamp.nanosec) * 1e-9
            )
            dt = max(1e-4, min(dt, 1.0))  # clamp to a sane range
        self.last_imu_stamp = msg.header.stamp

        self.imu_roll_deg = self.roll_filter.update(accel_roll, gyro_roll_rate, dt)
        self.imu_pitch_deg = self.pitch_filter.update(accel_pitch, gyro_pitch_rate, dt)
        # angular_velocity.z is already in rad/s (imu_publisher converts gyro data)
        self.imu_yaw_rate_rps = msg.angular_velocity.z

    def control_loop(self):
        stance_points = self.calculate_stance_points()
        motion = self.active_motion_command()

        if self.motion_is_zero(motion):
            self.gait_state = None
            self.publish_points(stance_points)
        else:
            if self.gait_state is None or self.gait_state.step_index >= self.gait_state.total_steps:
                self.gait_state = self.start_gait_cycle(stance_points, motion)

            if self.gait_state is None:
                self.publish_points(stance_points)
            else:
                self.advance_gait_cycle(self.gait_state)
                self.publish_points(self.gait_state.points)

        self._publish_odometry()

    def _publish_odometry(self):
        dt = 1.0 / self.control_rate_hz

        if self.gait_state is not None:
            vx = self.gait_state.odom_vx_mps
            vy = self.gait_state.odom_vy_mps
            vtheta = self.gait_state.odom_vtheta_rps
        else:
            vx = vy = vtheta = 0.0

        # Use IMU gyro Z for heading — it measures actual rotation including slip,
        # unlike the kinematic estimate which assumes the commanded turn was executed.
        heading_rate = self.imu_yaw_rate_rps if self.use_imu else vtheta

        # Rotate body-frame velocity into world frame and integrate position
        cos_h = math.cos(self.odom_theta_rad)
        sin_h = math.sin(self.odom_theta_rad)
        self.odom_x_m += (vx * cos_h - vy * sin_h) * dt
        self.odom_y_m += (vx * sin_h + vy * cos_h) * dt
        self.odom_theta_rad += heading_rate * dt

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame_id
        msg.child_frame_id = self.base_frame_id
        msg.pose.pose.position.x = self.odom_x_m
        msg.pose.pose.position.y = self.odom_y_m
        # Quaternion from yaw only: x=0, y=0, z=sin(θ/2), w=cos(θ/2)
        msg.pose.pose.orientation.z = math.sin(self.odom_theta_rad / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.odom_theta_rad / 2.0)
        # Conservative covariances prevent downstream filters from treating the
        # gait-integrated odometry as perfect ground truth.
        msg.pose.covariance[0] = 0.04
        msg.pose.covariance[7] = 0.04
        msg.pose.covariance[14] = 1e6
        msg.pose.covariance[21] = 1e6
        msg.pose.covariance[28] = 1e6
        msg.pose.covariance[35] = 0.09
        # Twist in body frame
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = heading_rate
        msg.twist.covariance[0] = 0.02
        msg.twist.covariance[7] = 0.02
        msg.twist.covariance[14] = 1e6
        msg.twist.covariance[21] = 1e6
        msg.twist.covariance[28] = 1e6
        msg.twist.covariance[35] = 0.04
        self.odom_publisher.publish(msg)

        if self.publish_odom_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = msg.header.stamp
            tf_msg.header.frame_id = self.odom_frame_id
            tf_msg.child_frame_id = self.base_frame_id
            tf_msg.transform.translation.x = self.odom_x_m
            tf_msg.transform.translation.y = self.odom_y_m
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.x = 0.0
            tf_msg.transform.rotation.y = 0.0
            tf_msg.transform.rotation.z = math.sin(self.odom_theta_rad / 2.0)
            tf_msg.transform.rotation.w = math.cos(self.odom_theta_rad / 2.0)
            self.tf_broadcaster.sendTransform(tf_msg)

    def active_motion_command(self):
        age_sec = (self.get_clock().now() - self.last_motion_cmd_time).nanoseconds / 1e9
        if age_sec > self.command_timeout_sec:
            return 0.0, 0.0, 0.0

        yaw_rate = self.cmd_yaw_rate_rad_s

        # When no yaw is commanded but the IMU detects rotation (leg slip or imbalance),
        # inject a counter-rotation proportional to the measured drift rate.
        # Capped at 30% of max yaw to avoid overwhelming the translation.
        if self.use_imu and self.yaw_correction_gain != 0.0 and abs(yaw_rate) < 1e-6:
            correction = -self.imu_yaw_rate_rps * self.yaw_correction_gain
            yaw_rate = clamp(correction, -self.max_yaw_rate_rad_s * 0.3, self.max_yaw_rate_rad_s * 0.3)

        return self.cmd_linear_x_mps, self.cmd_linear_y_mps, yaw_rate

    def motion_is_zero(self, motion):
        return all(abs(value) < 1e-6 for value in motion)

    def calculate_stance_points(self):
        roll_deg, pitch_deg, yaw_deg = self.commanded_body_pose()
        position = [
            -self.body_shift_mm[0],
            -self.body_shift_mm[1],
            self.default_body_height_mm - self.body_shift_mm[2],
        ]

        rotation_matrix = self.rotation_matrix(roll_deg, pitch_deg, yaw_deg)
        foot_positions = []
        for footpoint in BASE_FOOTPOINTS:
            rotated = matrix_vector_multiply(rotation_matrix, footpoint)
            foot_positions.append([
                position[0] + rotated[0],
                position[1] + rotated[1],
                position[2] + rotated[2],
            ])

        return foot_positions

    def commanded_body_pose(self):
        roll_deg = self.body_pose_deg[0]
        pitch_deg = self.body_pose_deg[1]
        yaw_deg = self.body_pose_deg[2]

        if self.use_imu and self.balance_gain != 0.0:
            roll_deg -= self.imu_roll_deg * self.balance_gain
            pitch_deg -= self.imu_pitch_deg * self.balance_gain

        return (
            clamp(roll_deg, -self.max_roll_deg, self.max_roll_deg),
            clamp(pitch_deg, -self.max_pitch_deg, self.max_pitch_deg),
            clamp(yaw_deg, -self.max_yaw_deg, self.max_yaw_deg),
        )

    def rotation_matrix(self, roll_deg, pitch_deg, yaw_deg):
        roll_angle = math.radians(roll_deg)
        pitch_angle = math.radians(pitch_deg)
        yaw_angle = math.radians(yaw_deg)

        # This preserves the same roll/pitch/yaw convention used in Sample_Code/Server/control.py.
        rotation_x = [
            [1.0, 0.0, 0.0],
            [0.0, math.cos(pitch_angle), -math.sin(pitch_angle)],
            [0.0, math.sin(pitch_angle), math.cos(pitch_angle)],
        ]
        rotation_y = [
            [math.cos(roll_angle), 0.0, -math.sin(roll_angle)],
            [0.0, 1.0, 0.0],
            [math.sin(roll_angle), 0.0, math.cos(roll_angle)],
        ]
        rotation_z = [
            [math.cos(yaw_angle), -math.sin(yaw_angle), 0.0],
            [math.sin(yaw_angle), math.cos(yaw_angle), 0.0],
            [0.0, 0.0, 1.0],
        ]

        return matrix_multiply(matrix_multiply(rotation_x, rotation_y), rotation_z)

    def start_gait_cycle(self, stance_points, motion):
        linear_x_mps, linear_y_mps, yaw_rate_rad_s = motion

        speed_ratio = max(
            abs(linear_x_mps) / self.max_linear_speed_mps,
            abs(linear_y_mps) / self.max_lateral_speed_mps,
            abs(yaw_rate_rad_s) / self.max_yaw_rate_rad_s,
        )
        speed_ratio = clamp(speed_ratio, 0.0, 1.0)
        if speed_ratio <= 0.0:
            return None

        if self.gait == 'wave':
            raw_steps = round(map_value(speed_ratio, 0.0, 1.0, 171.0, 45.0))
            segment_steps = max(1, raw_steps // 6)
            total_steps = max(6, segment_steps * 6)
        else:
            total_steps = max(8, round(map_value(speed_ratio, 0.0, 1.0, 126.0, 22.0)))

        cycle_duration_sec = total_steps / self.control_rate_hz
        stride_x_mm = clamp(
            linear_x_mps * 1000.0 * cycle_duration_sec,
            -self.max_stride_x_mm,
            self.max_stride_x_mm,
        )
        stride_y_mm = clamp(
            linear_y_mps * 1000.0 * cycle_duration_sec,
            -self.max_stride_y_mm,
            self.max_stride_y_mm,
        )
        turn_deg = clamp(
            math.degrees(yaw_rate_rad_s) * cycle_duration_sec,
            -self.max_turn_per_cycle_deg,
            self.max_turn_per_cycle_deg,
        )

        if abs(stride_x_mm) < 1e-6 and abs(stride_y_mm) < 1e-6 and abs(turn_deg) < 1e-6:
            return None

        turn_rad = math.radians(turn_deg)
        points = copy.deepcopy(stance_points)
        planar_delta_mm = []
        for point in points:
            delta_x = (
                (point[0] * math.cos(turn_rad) + point[1] * math.sin(turn_rad) - point[0]) + stride_x_mm
            ) / total_steps
            delta_y = (
                (-point[0] * math.sin(turn_rad) + point[1] * math.cos(turn_rad) - point[1]) + stride_y_mm
            ) / total_steps
            planar_delta_mm.append([delta_x, delta_y])

        return GaitCycleState(
            gait_name=self.gait,
            total_steps=total_steps,
            lift_step_mm=self.step_height_mm / float(total_steps),
            planar_delta_mm=planar_delta_mm,
            baseline_points=copy.deepcopy(stance_points),
            points=points,
            odom_vx_mps=stride_x_mm * self.control_rate_hz / (total_steps * 1000.0),
            odom_vy_mps=stride_y_mm * self.control_rate_hz / (total_steps * 1000.0),
            odom_vtheta_rps=math.radians(turn_deg) * self.control_rate_hz / total_steps,
        )

    def advance_gait_cycle(self, gait_state: GaitCycleState):
        if gait_state.gait_name == 'wave':
            self.advance_wave_gait(gait_state)
        else:
            self.advance_tripod_gait(gait_state)
        gait_state.step_index += 1

    def advance_tripod_gait(self, gait_state: GaitCycleState):
        j = gait_state.step_index
        total_steps = gait_state.total_steps
        z_step = gait_state.lift_step_mm

        for group_index in range(3):
            even_leg = 2 * group_index
            odd_leg = even_leg + 1

            if j < total_steps / 8.0:
                self.shift_leg_planar(gait_state, even_leg, -4.0)
                self.shift_leg_planar(gait_state, odd_leg, 8.0)
                gait_state.points[odd_leg][2] = gait_state.baseline_points[odd_leg][2] + self.step_height_mm
            elif j < total_steps / 4.0:
                self.shift_leg_planar(gait_state, even_leg, -4.0)
                gait_state.points[odd_leg][2] -= z_step * 8.0
            elif j < (3.0 * total_steps) / 8.0:
                gait_state.points[even_leg][2] += z_step * 8.0
                self.shift_leg_planar(gait_state, odd_leg, -4.0)
            elif j < (5.0 * total_steps) / 8.0:
                self.shift_leg_planar(gait_state, even_leg, 8.0)
                self.shift_leg_planar(gait_state, odd_leg, -4.0)
            elif j < (3.0 * total_steps) / 4.0:
                gait_state.points[even_leg][2] -= z_step * 8.0
                self.shift_leg_planar(gait_state, odd_leg, -4.0)
            elif j < (7.0 * total_steps) / 8.0:
                self.shift_leg_planar(gait_state, even_leg, -4.0)
                gait_state.points[odd_leg][2] += z_step * 8.0
            else:
                self.shift_leg_planar(gait_state, even_leg, -4.0)
                self.shift_leg_planar(gait_state, odd_leg, 8.0)

    def advance_wave_gait(self, gait_state: GaitCycleState):
        total_steps = gait_state.total_steps
        segment_steps = max(1, total_steps // 6)
        lift_steps = max(1, total_steps // 18)
        swing_steps = max(lift_steps + 1, total_steps // 9)

        phase_index = min(5, gait_state.step_index // segment_steps)
        phase_step = gait_state.step_index % segment_steps
        lifted_leg = WAVE_SEQUENCE[phase_index]

        for leg_index in range(6):
            if leg_index == lifted_leg:
                if phase_step < lift_steps:
                    gait_state.points[leg_index][2] += gait_state.lift_step_mm * 18.0
                elif phase_step < swing_steps:
                    gait_state.points[leg_index][0] += gait_state.planar_delta_mm[leg_index][0] * 30.0
                    gait_state.points[leg_index][1] += gait_state.planar_delta_mm[leg_index][1] * 30.0
                elif phase_step < segment_steps:
                    gait_state.points[leg_index][2] -= gait_state.lift_step_mm * 18.0
            else:
                gait_state.points[leg_index][0] -= gait_state.planar_delta_mm[leg_index][0] * 2.0
                gait_state.points[leg_index][1] -= gait_state.planar_delta_mm[leg_index][1] * 2.0

    def shift_leg_planar(self, gait_state: GaitCycleState, leg_index, scale):
        gait_state.points[leg_index][0] += gait_state.planar_delta_mm[leg_index][0] * scale
        gait_state.points[leg_index][1] += gait_state.planar_delta_mm[leg_index][1] * scale

    def publish_points(self, points):
        leg_positions = self.transform_coordinates(points)
        if not self.check_point_validity(leg_positions):
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if (now_sec - self.last_invalid_warn_time) > 1.0:
                self.get_logger().warn('Skipping pose publish because a leg target is outside the reachable workspace')
                self.last_invalid_warn_time = now_sec
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = self.leg_positions_to_servo_targets(leg_positions)
        self.target_publisher.publish(msg)

    def transform_coordinates(self, points):
        leg_positions = []
        for leg_index, point in enumerate(points):
            angle = math.radians(LEG_MOUNT_ANGLES_DEG[leg_index])
            leg_positions.append([
                point[0] * math.cos(angle) + point[1] * math.sin(angle) - LEG_X_OFFSETS_MM[leg_index],
                -point[0] * math.sin(angle) + point[1] * math.cos(angle),
                point[2] - LEG_Z_OFFSET_MM,
            ])
        return leg_positions

    def check_point_validity(self, leg_positions):
        for x_value, y_value, z_value in leg_positions:
            leg_length = math.sqrt(x_value * x_value + y_value * y_value + z_value * z_value)
            if leg_length > 248.0 or leg_length < 90.0:
                return False
        return True

    def leg_positions_to_servo_targets(self, leg_positions):
        servo_targets = []
        for leg_index, coordinates in enumerate(leg_positions):
            servo_angles = servo_angles_from_leg_coordinates(leg_index, coordinates)
            servo_targets.extend([
                servo_angles['coxa'],
                servo_angles['femur'],
                servo_angles['tibia'],
            ])
        return servo_targets


def main(args=None):
    rclpy.init(args=args)
    node = LocomotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
