#!/usr/bin/env python3

import copy
import math
import time
from dataclasses import dataclass

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster

from .calibration_store import JOINT_NAMES, servo_angles_from_leg_coordinates
from .gait_math import (
    DEFAULT_TRIPOD_PLANAR_TRAVEL_SCALE,
    cycle_planar_travel_from_deltas,
    tripod_points_for_phase,
)
from .kalman_filter import AngleKalmanFilter
from .yaw_control import (
    compute_heading_hold_pid,
    quaternion_from_euler,
    quaternion_normalize,
    quaternion_to_yaw,
    resolve_parameter_value,
    yaw_is_trusted_from_covariance,
)


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
    cycle_planar_travel_mm: list
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


def normalize_angle(angle_rad):
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


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
    DEFAULT_YAW_KP = 0.45
    DEFAULT_LEGACY_YAW_CORRECTION_GAIN = 0.0
    DEFAULT_YAW_KI = 0.12
    DEFAULT_YAW_KD = 0.10
    DEFAULT_YAW_DEADBAND_DEG = 2.5
    DEFAULT_YAW_INTEGRATOR_LIMIT = 1.2
    DEFAULT_MAX_TRUSTED_YAW_COVARIANCE_RAD2 = 1.0
    DIAGNOSTIC_PUBLISH_PERIOD_SEC = 0.25

    def __init__(self):
        super().__init__('locomotion')

        self.declare_parameter('use_imu', True)
        self.declare_parameter('balance_gain', 0.1)
        self.declare_parameter('gait', 'tripod')
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('default_body_height_mm', -25.0)
        self.declare_parameter('step_height_mm', 40.0)
        self.declare_parameter(
            'tripod_planar_travel_scale',
            DEFAULT_TRIPOD_PLANAR_TRAVEL_SCALE,
        )
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
        self.declare_parameter('yaw_kp', self.DEFAULT_YAW_KP)
        self.declare_parameter(
            'yaw_correction_gain',
            self.DEFAULT_LEGACY_YAW_CORRECTION_GAIN,
        )
        self.declare_parameter('yaw_ki', self.DEFAULT_YAW_KI)
        self.declare_parameter('yaw_kd', self.DEFAULT_YAW_KD)
        self.declare_parameter('yaw_deadband_deg', self.DEFAULT_YAW_DEADBAND_DEG)
        self.declare_parameter('yaw_integrator_limit', self.DEFAULT_YAW_INTEGRATOR_LIMIT)
        self.declare_parameter(
            'max_trusted_yaw_covariance_rad2',
            self.DEFAULT_MAX_TRUSTED_YAW_COVARIANCE_RAD2,
        )
        self.declare_parameter(
            'yaw_hold_diagnostic_topic',
            '/locomotion/yaw_hold_diagnostics',
        )
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
        self.tripod_planar_travel_scale = max(
            0.1,
            float(self.get_parameter('tripod_planar_travel_scale').value),
        )
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
        self.yaw_kp, yaw_kp_conflict = resolve_parameter_value(
            float(self.get_parameter('yaw_kp').value),
            float(self.get_parameter('yaw_correction_gain').value),
            self.DEFAULT_YAW_KP,
            legacy_default_value=self.DEFAULT_LEGACY_YAW_CORRECTION_GAIN,
        )
        self.yaw_ki = max(0.0, float(self.get_parameter('yaw_ki').value))
        self.yaw_kd = max(0.0, float(self.get_parameter('yaw_kd').value))
        self.yaw_deadband_rad = math.radians(
            max(0.0, float(self.get_parameter('yaw_deadband_deg').value))
        )
        self.yaw_integrator_limit = max(
            0.0,
            float(self.get_parameter('yaw_integrator_limit').value),
        )
        self.max_trusted_yaw_covariance_rad2 = max(
            0.0,
            float(self.get_parameter('max_trusted_yaw_covariance_rad2').value),
        )
        self.yaw_hold_diagnostic_topic = str(
            self.get_parameter('yaw_hold_diagnostic_topic').value
        )
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
        self.imu_yaw_rad = 0.0
        self.imu_yaw_valid = False
        self.imu_yaw_trusted = False
        self.imu_yaw_covariance_rad2 = math.inf
        self.imu_orientation_quaternion = None
        self.roll_filter = AngleKalmanFilter()
        self.pitch_filter = AngleKalmanFilter()
        self.last_imu_stamp = None
        self.heading_hold_target_rad = None
        self.heading_integral_state = 0.0
        self.last_invalid_warn_time = 0.0
        self.last_diagnostic_publish_time = 0.0
        self.last_heading_hold_mode = 'idle'
        self.last_heading_hold_reason = 'startup'
        self.last_heading_hold_step = None
        self.last_heading_hold_correction_rad_s = 0.0
        self.gait_state = None

        self.odom_x_m = 0.0
        self.odom_y_m = 0.0
        self.odom_theta_rad = 0.0

        self.target_publisher = self.create_publisher(JointState, 'servo_targets', 10)
        self.odom_publisher = self.create_publisher(Odometry, self.odom_topic, 10)
        self.yaw_hold_diag_publisher = self.create_publisher(
            DiagnosticArray,
            self.yaw_hold_diagnostic_topic,
            10,
        )
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

        self.add_on_set_parameters_callback(self.parameter_update_callback)
        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.control_loop)

        self.get_logger().info('Locomotion controller ready')
        self.get_logger().info('Topics: /cmd_vel, /body_pose, /body_shift, /servo_targets')
        self.get_logger().info(
            f'cmd_vel uses m/s and rad/s. body_pose uses roll/pitch/yaw in degrees. '
            f'body_shift uses x/y/z in mm. Publishing odom on /{self.odom_topic} '
            f'with TF {"enabled" if self.publish_odom_tf else "disabled"}.'
        )
        self.get_logger().info(
            f'Heading hold PID gains kp={self.yaw_kp:.3f}, ki={self.yaw_ki:.3f}, '
            f'kd={self.yaw_kd:.3f}, '
            f'deadband={math.degrees(self.yaw_deadband_rad):.1f} deg, '
            f'integrator_limit={self.yaw_integrator_limit:.2f}.'
        )
        self.get_logger().info(
            'Heading hold trusts IMU yaw when orientation covariance yaw <= '
            f'{self.max_trusted_yaw_covariance_rad2:.2f} rad^2. '
            f'Publishing diagnostics on {self.yaw_hold_diagnostic_topic}.'
        )
        if self.gait == 'tripod':
            self.get_logger().info(
                f'Tripod gait planar travel scale={self.tripod_planar_travel_scale:.2f}.'
            )
        if yaw_kp_conflict:
            self.get_logger().warn(
                'Both yaw_kp and legacy yaw_correction_gain were set. Using yaw_kp.'
            )

    def parameter_update_callback(self, parameters):
        changed_fields = []

        for parameter in parameters:
            if parameter.name in ('yaw_kp', 'yaw_correction_gain'):
                value = float(parameter.value)
                if value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f'{parameter.name} must be non-negative.',
                    )
                self.yaw_kp = value
                changed_fields.append(f'{parameter.name}={self.yaw_kp:.3f}')
            elif parameter.name == 'yaw_ki':
                value = float(parameter.value)
                if value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='yaw_ki must be non-negative.',
                    )
                self.yaw_ki = value
                changed_fields.append(f'yaw_ki={self.yaw_ki:.3f}')
            elif parameter.name == 'yaw_kd':
                value = float(parameter.value)
                if value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='yaw_kd must be non-negative.',
                    )
                self.yaw_kd = value
                changed_fields.append(f'yaw_kd={self.yaw_kd:.3f}')
            elif parameter.name == 'yaw_deadband_deg':
                yaw_deadband_deg = float(parameter.value)
                if yaw_deadband_deg < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='yaw_deadband_deg must be non-negative.',
                    )
                self.yaw_deadband_rad = math.radians(yaw_deadband_deg)
                changed_fields.append(
                    f'yaw_deadband_deg={yaw_deadband_deg:.2f}'
                )
            elif parameter.name == 'yaw_integrator_limit':
                value = float(parameter.value)
                if value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='yaw_integrator_limit must be non-negative.',
                    )
                self.yaw_integrator_limit = value
                self.heading_integral_state = clamp(
                    self.heading_integral_state,
                    -self.yaw_integrator_limit,
                    self.yaw_integrator_limit,
                )
                changed_fields.append(
                    f'yaw_integrator_limit={self.yaw_integrator_limit:.3f}'
                )
            elif parameter.name == 'max_trusted_yaw_covariance_rad2':
                value = float(parameter.value)
                if value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='max_trusted_yaw_covariance_rad2 must be non-negative.',
                    )
                self.max_trusted_yaw_covariance_rad2 = value
                changed_fields.append(
                    'max_trusted_yaw_covariance_rad2='
                    f'{self.max_trusted_yaw_covariance_rad2:.3f}'
                )
            elif parameter.name == 'tripod_planar_travel_scale':
                value = float(parameter.value)
                if value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='tripod_planar_travel_scale must be greater than zero.',
                    )
                self.tripod_planar_travel_scale = value
                changed_fields.append(
                    f'tripod_planar_travel_scale={self.tripod_planar_travel_scale:.3f}'
                )

        if changed_fields:
            self.get_logger().info(
                'Updated locomotion heading-hold tuning: '
                + ', '.join(changed_fields)
            )

        return SetParametersResult(successful=True)

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
            dt = 0.02  # fallback used only for the first sample before timestamps are available
        else:
            dt = (
                msg.header.stamp.sec - self.last_imu_stamp.sec
                + (msg.header.stamp.nanosec - self.last_imu_stamp.nanosec) * 1e-9
            )
            dt = max(1e-4, min(dt, 1.0))  # clamp to a sane range
        self.last_imu_stamp = msg.header.stamp

        self.imu_roll_deg = self.roll_filter.update(accel_roll, gyro_roll_rate, dt)
        self.imu_pitch_deg = self.pitch_filter.update(accel_pitch, gyro_pitch_rate, dt)
        self.imu_yaw_rate_rps = msg.angular_velocity.z

        orientation_covariance = msg.orientation_covariance
        orientation_available = (
            orientation_covariance[0] >= 0.0
            and any(abs(value) > 1e-6 for value in (
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
            ))
        )
        if orientation_available:
            self.imu_orientation_quaternion = quaternion_normalize((
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
            ))
            self.imu_yaw_rad = quaternion_to_yaw(
                *self.imu_orientation_quaternion,
            )
            self.imu_yaw_valid = True
            self.imu_yaw_covariance_rad2 = (
                float(orientation_covariance[8])
                if len(orientation_covariance) >= 9
                else math.inf
            )
            self.imu_yaw_trusted = yaw_is_trusted_from_covariance(
                orientation_covariance,
                self.max_trusted_yaw_covariance_rad2,
            )
        else:
            self.imu_yaw_valid = False
            self.imu_yaw_trusted = False
            self.imu_yaw_covariance_rad2 = math.inf
            self.imu_orientation_quaternion = None

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
        self._publish_yaw_hold_diagnostics(motion)

    def _publish_odometry(self):
        dt = 1.0 / self.control_rate_hz

        if self.gait_state is not None:
            vx = self.gait_state.odom_vx_mps
            vy = self.gait_state.odom_vy_mps
            vtheta = self.gait_state.odom_vtheta_rps
        else:
            vx = vy = vtheta = 0.0

        heading_rate = vtheta
        if self.use_imu:
            yaw_available, yaw_rad, yaw_rate_rps = self._active_yaw_estimate()
            heading_rate = yaw_rate_rps
            if yaw_available:
                self.odom_theta_rad = yaw_rad
            else:
                self.odom_theta_rad = normalize_angle(self.odom_theta_rad + heading_rate * dt)
        else:
            self.odom_theta_rad = normalize_angle(self.odom_theta_rad + heading_rate * dt)

        # Rotate body-frame velocity into world frame and integrate position
        cos_h = math.cos(self.odom_theta_rad)
        sin_h = math.sin(self.odom_theta_rad)
        self.odom_x_m += (vx * cos_h - vy * sin_h) * dt
        self.odom_y_m += (vx * sin_h + vy * cos_h) * dt

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
        self.last_heading_hold_mode = 'idle'
        self.last_heading_hold_reason = 'no_motion_command'
        self.last_heading_hold_step = None
        self.last_heading_hold_correction_rad_s = 0.0

        if age_sec > self.command_timeout_sec:
            self._reset_heading_hold_state()
            self.last_heading_hold_reason = 'command_timeout'
            return 0.0, 0.0, 0.0

        linear_x = self.cmd_linear_x_mps
        linear_y = self.cmd_linear_y_mps
        yaw_rate = self.cmd_yaw_rate_rad_s
        translating = abs(linear_x) > 1e-6 or abs(linear_y) > 1e-6

        if not translating:
            self._reset_heading_hold_state()
            self.last_heading_hold_reason = 'no_translation'
        elif abs(yaw_rate) >= 1e-6:
            self._reset_heading_hold_state()
            self.last_heading_hold_mode = 'manual_yaw'
            self.last_heading_hold_reason = 'manual_yaw_command'
        elif self.use_imu and (self.yaw_kp != 0.0 or self.yaw_ki != 0.0 or self.yaw_kd != 0.0):
            correction = 0.0
            yaw_available, current_yaw_rad, current_yaw_rate_rps = self._active_yaw_estimate()
            if yaw_available:
                if self.heading_hold_target_rad is None:
                    self.heading_hold_target_rad = current_yaw_rad
                    self.heading_integral_state = 0.0
                    self.get_logger().info(
                        'Heading hold latched reference yaw '
                        f'{math.degrees(self.heading_hold_target_rad):.1f} deg, quaternion '
                        f'{self._format_quaternion(self._heading_reference_quaternion())}.'
                    )
                step = compute_heading_hold_pid(
                    target_yaw_rad=self.heading_hold_target_rad,
                    current_yaw_rad=current_yaw_rad,
                    current_yaw_rate_rps=current_yaw_rate_rps,
                    integral_state_rad=self.heading_integral_state,
                    control_dt_sec=1.0 / self.control_rate_hz,
                    kp=self.yaw_kp,
                    ki=self.yaw_ki,
                    kd=self.yaw_kd,
                    deadband_rad=self.yaw_deadband_rad,
                    integral_limit=self.yaw_integrator_limit,
                    correction_limit=self.max_yaw_rate_rad_s * 0.3,
                )
                correction = step.correction
                if step.integral_should_update:
                    self.heading_integral_state = step.integral_candidate
                self.last_heading_hold_mode = 'tracking'
                self.last_heading_hold_reason = 'yaw_trusted'
                self.last_heading_hold_step = step
                self.last_heading_hold_correction_rad_s = correction
            elif self.imu_yaw_valid:
                self.last_heading_hold_mode = 'open_loop'
                self.last_heading_hold_reason = 'yaw_untrusted'
                self._warn_unusable_yaw_estimate()
            else:
                self.last_heading_hold_mode = 'open_loop'
                self.last_heading_hold_reason = 'yaw_missing'
                self._warn_unusable_yaw_estimate()

            yaw_rate = clamp(
                correction,
                -self.max_yaw_rate_rad_s * 0.3,
                self.max_yaw_rate_rad_s * 0.3,
            )
        else:
            self._reset_heading_hold_state()
            self.last_heading_hold_reason = 'heading_hold_disabled'

        return linear_x, linear_y, yaw_rate

    def _reset_heading_hold_state(self):
        self.heading_hold_target_rad = None
        self.heading_integral_state = 0.0

    def _active_yaw_estimate(self):
        if self.imu_yaw_valid and self.imu_yaw_trusted:
            return True, self.imu_yaw_rad, self.imu_yaw_rate_rps

        return False, 0.0, self.imu_yaw_rate_rps

    def _heading_reference_quaternion(self):
        if self.heading_hold_target_rad is None:
            return None

        return quaternion_normalize(
            quaternion_from_euler(0.0, 0.0, self.heading_hold_target_rad)
        )

    def _format_quaternion(self, quaternion):
        if quaternion is None:
            return 'n/a'

        return (
            f'({quaternion[0]:.4f}, {quaternion[1]:.4f}, '
            f'{quaternion[2]:.4f}, {quaternion[3]:.4f})'
        )

    def _warn_unusable_yaw_estimate(self):
        now = time.monotonic()
        if now - self.last_invalid_warn_time < 2.0:
            return

        self.last_invalid_warn_time = now
        if self.imu_yaw_valid and not self.imu_yaw_trusted:
            self.get_logger().warn(
                'Heading hold is enabled, but IMU yaw is currently untrusted '
                f'(yaw covariance={self.imu_yaw_covariance_rad2:.3f} rad^2, '
                f'limit={self.max_trusted_yaw_covariance_rad2:.3f}). '
                'Yaw correction is temporarily disabled while translation continues open-loop.'
            )
            return

        self.get_logger().warn(
            'Heading hold is enabled, but no valid IMU yaw estimate is available yet. '
            'Yaw correction is temporarily disabled.'
        )

    def _publish_yaw_hold_diagnostics(self, motion):
        now = time.monotonic()
        if now - self.last_diagnostic_publish_time < self.DIAGNOSTIC_PUBLISH_PERIOD_SEC:
            return

        self.last_diagnostic_publish_time = now
        step = self.last_heading_hold_step
        reference_quaternion = self._heading_reference_quaternion()
        current_quaternion = self.imu_orientation_quaternion
        current_yaw_deg = (
            math.degrees(self.imu_yaw_rad) if self.imu_yaw_valid else math.nan
        )
        reference_yaw_deg = (
            math.degrees(self.heading_hold_target_rad)
            if self.heading_hold_target_rad is not None
            else math.nan
        )
        yaw_error_rad = step.yaw_error_rad if step is not None else 0.0
        deadbanded_yaw_error_rad = (
            step.deadbanded_yaw_error_rad if step is not None else 0.0
        )
        yaw_error_percent_of_180 = min(
            100.0,
            abs(yaw_error_rad) * 100.0 / math.pi,
        )

        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = 'hexapod_locomotion/yaw_hold'
        status.hardware_id = self.base_frame_id
        if self.last_heading_hold_mode == 'tracking' and self.imu_yaw_trusted:
            status.level = DiagnosticStatus.OK
            status.message = 'tracking heading'
        elif self.motion_is_zero(motion) or self.last_heading_hold_mode == 'manual_yaw':
            status.level = DiagnosticStatus.OK
            status.message = self.last_heading_hold_reason
        else:
            status.level = DiagnosticStatus.WARN
            status.message = self.last_heading_hold_reason

        status.values = [
            KeyValue(key='control_mode', value=self.last_heading_hold_mode),
            KeyValue(key='control_reason', value=self.last_heading_hold_reason),
            KeyValue(
                key='control_strategy',
                value=(
                    'yaw_error=reference-current; positive yaw error commands positive angular.z; '
                    'derivative opposes measured yaw rate.'
                ),
            ),
            KeyValue(
                key='tracking_active',
                value=str(self.last_heading_hold_mode == 'tracking').lower(),
            ),
            KeyValue(key='imu_yaw_valid', value=str(self.imu_yaw_valid).lower()),
            KeyValue(key='imu_yaw_trusted', value=str(self.imu_yaw_trusted).lower()),
            KeyValue(
                key='imu_yaw_covariance_rad2',
                value='nan'
                if not math.isfinite(self.imu_yaw_covariance_rad2)
                else f'{self.imu_yaw_covariance_rad2:.6f}',
            ),
            KeyValue(
                key='max_trusted_yaw_covariance_rad2',
                value=f'{self.max_trusted_yaw_covariance_rad2:.6f}',
            ),
            KeyValue(
                key='reference_yaw_deg',
                value='nan'
                if not math.isfinite(reference_yaw_deg)
                else f'{reference_yaw_deg:.3f}',
            ),
            KeyValue(
                key='current_yaw_deg',
                value='nan'
                if not math.isfinite(current_yaw_deg)
                else f'{current_yaw_deg:.3f}',
            ),
            KeyValue(
                key='yaw_error_deg',
                value=f'{math.degrees(yaw_error_rad):.3f}',
            ),
            KeyValue(
                key='yaw_error_percent_of_180deg',
                value=f'{yaw_error_percent_of_180:.3f}',
            ),
            KeyValue(
                key='deadbanded_yaw_error_deg',
                value=f'{math.degrees(deadbanded_yaw_error_rad):.3f}',
            ),
            KeyValue(
                key='measured_yaw_rate_deg_s',
                value=f'{math.degrees(self.imu_yaw_rate_rps):.3f}',
            ),
            KeyValue(
                key='manual_yaw_command_deg_s',
                value=f'{math.degrees(self.cmd_yaw_rate_rad_s):.3f}',
            ),
            KeyValue(
                key='auto_correction_yaw_rate_deg_s',
                value=f'{math.degrees(self.last_heading_hold_correction_rad_s):.3f}',
            ),
            KeyValue(
                key='integral_state_rad',
                value=f'{self.heading_integral_state:.6f}',
            ),
            KeyValue(
                key='pid_p_term_deg_s',
                value=f'{math.degrees(step.proportional_term) if step is not None else 0.0:.3f}',
            ),
            KeyValue(
                key='pid_i_term_deg_s',
                value=f'{math.degrees(step.integral_term) if step is not None else 0.0:.3f}',
            ),
            KeyValue(
                key='pid_d_term_deg_s',
                value=f'{math.degrees(step.derivative_term) if step is not None else 0.0:.3f}',
            ),
            KeyValue(
                key='pid_unsaturated_output_deg_s',
                value=f'{math.degrees(step.unsaturated_correction) if step is not None else 0.0:.3f}',
            ),
            KeyValue(
                key='pid_output_deg_s',
                value=f'{math.degrees(step.correction) if step is not None else 0.0:.3f}',
            ),
            KeyValue(
                key='pid_output_saturated',
                value=str(step.correction_is_saturated if step is not None else False).lower(),
            ),
            KeyValue(
                key='reference_quaternion_xyzw',
                value=self._format_quaternion(reference_quaternion),
            ),
            KeyValue(
                key='current_quaternion_xyzw',
                value=self._format_quaternion(current_quaternion),
            ),
        ]
        diag.status.append(status)
        self.yaw_hold_diag_publisher.publish(diag)

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
            cycle_planar_travel_mm=cycle_planar_travel_from_deltas(
                planar_delta_mm,
                total_steps,
                self.tripod_planar_travel_scale if self.gait == 'tripod' else 1.0,
            ),
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
        phase_progress = (gait_state.step_index + 1) / float(gait_state.total_steps)
        gait_state.points = tripod_points_for_phase(
            gait_state.baseline_points,
            gait_state.cycle_planar_travel_mm,
            self.step_height_mm,
            phase_progress,
        )

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
