#!/usr/bin/env python3

import math
from dataclasses import dataclass
from datetime import datetime, timezone

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Imu

from .kalman_calibration_store import (
    default_kalman_calibration,
    save_kalman_calibration,
)
from .kalman_filter import AngleKalmanFilter, DEFAULT_Q_ANGLE, DEFAULT_Q_BIAS, DEFAULT_R_MEASURE


@dataclass
class AxisAccumulator:
    name: str
    filter: AngleKalmanFilter
    sample_count: int = 0
    gain_angle_sum: float = 0.0
    gain_bias_sum: float = 0.0
    bias_sum: float = 0.0

    def add_sample(self):
        gain_angle, gain_bias = self.filter.last_gain
        self.sample_count += 1
        self.gain_angle_sum += gain_angle
        self.gain_bias_sum += gain_bias
        self.bias_sum += self.filter.bias

    def build_config(self):
        if self.sample_count <= 0:
            raise ValueError(f'No samples accumulated for {self.name}')

        return {
            'q_angle': self.filter.q_angle,
            'q_bias': self.filter.q_bias,
            'r_measure': self.filter.r_measure,
            'initial_bias_deg_s': self.bias_sum / self.sample_count,
            'use_fixed_gain': True,
            'kalman_gain_angle': self.gain_angle_sum / self.sample_count,
            'kalman_gain_bias': self.gain_bias_sum / self.sample_count,
        }


class KalmanCalibrationNode(Node):
    def __init__(self):
        super().__init__('kalman_calibration')

        default_yaml = (
            get_package_share_directory('hexapod_locomotion')
            + '/config/imu_kalman_calibration.yaml'
        )

        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('calibration_file', default_yaml)
        self.declare_parameter('warmup_duration_sec', 5.0)
        self.declare_parameter('accumulation_duration_sec', 15.0)
        self.declare_parameter('q_angle', DEFAULT_Q_ANGLE)
        self.declare_parameter('q_bias', DEFAULT_Q_BIAS)
        self.declare_parameter('r_measure', DEFAULT_R_MEASURE)
        self.declare_parameter('progress_log_period_sec', 1.0)

        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.calibration_file = str(self.get_parameter('calibration_file').value)
        self.warmup_duration_sec = max(0.0, float(self.get_parameter('warmup_duration_sec').value))
        self.accumulation_duration_sec = max(
            0.1,
            float(self.get_parameter('accumulation_duration_sec').value),
        )
        self.progress_log_period_sec = max(
            0.0,
            float(self.get_parameter('progress_log_period_sec').value),
        )

        q_angle = float(self.get_parameter('q_angle').value)
        q_bias = float(self.get_parameter('q_bias').value)
        r_measure = float(self.get_parameter('r_measure').value)

        self.roll = AxisAccumulator('roll', AngleKalmanFilter(q_angle=q_angle, q_bias=q_bias, r_measure=r_measure))
        self.pitch = AxisAccumulator('pitch', AngleKalmanFilter(q_angle=q_angle, q_bias=q_bias, r_measure=r_measure))

        self.last_imu_stamp = None
        self.first_sample_time = None
        self.last_progress_bucket = -1
        self.saved = False

        self.subscription = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            10,
        )

        self.get_logger().info(
            f'Idle IMU Kalman calibration started. Warmup={self.warmup_duration_sec:.1f}s, '
            f'accumulation={self.accumulation_duration_sec:.1f}s, output={self.calibration_file}'
        )
        self.get_logger().info(
            f'Keep the robot still while sampling from {self.imu_topic}. '
            f'Tuning seed: q_angle={q_angle}, q_bias={q_bias}, r_measure={r_measure}'
        )

    def imu_callback(self, msg: Imu):
        if self.saved:
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.first_sample_time is None:
            self.first_sample_time = now_sec

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        accel_roll = math.degrees(math.atan2(ay, az))
        accel_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))

        gyro_roll_rate = math.degrees(msg.angular_velocity.x)
        gyro_pitch_rate = math.degrees(msg.angular_velocity.y)

        if self.last_imu_stamp is None:
            dt = 0.1
            self.roll.filter.angle = accel_roll
            self.pitch.filter.angle = accel_pitch
        else:
            dt = (
                msg.header.stamp.sec - self.last_imu_stamp.sec
                + (msg.header.stamp.nanosec - self.last_imu_stamp.nanosec) * 1e-9
            )
            dt = max(1e-4, min(dt, 1.0))
        self.last_imu_stamp = msg.header.stamp

        self.roll.filter.update(accel_roll, gyro_roll_rate, dt)
        self.pitch.filter.update(accel_pitch, gyro_pitch_rate, dt)

        elapsed_sec = now_sec - self.first_sample_time
        if self.progress_log_period_sec > 0.0:
            progress_bucket = int(elapsed_sec / self.progress_log_period_sec)
            if progress_bucket != self.last_progress_bucket:
                self.last_progress_bucket = progress_bucket
                self.log_progress(elapsed_sec)

        if elapsed_sec < self.warmup_duration_sec:
            return

        if elapsed_sec <= self.warmup_duration_sec + self.accumulation_duration_sec:
            self.roll.add_sample()
            self.pitch.add_sample()
            return

        self.save_and_shutdown()

    def log_progress(self, elapsed_sec):
        if elapsed_sec < self.warmup_duration_sec:
            remaining = max(0.0, self.warmup_duration_sec - elapsed_sec)
            self.get_logger().info(f'Warmup in progress, {remaining:.1f}s remaining before accumulation')
            return

        accumulation_elapsed = elapsed_sec - self.warmup_duration_sec
        accumulation_remaining = max(0.0, self.accumulation_duration_sec - accumulation_elapsed)
        self.get_logger().info(
            f'Accumulating gains, {accumulation_remaining:.1f}s remaining '
            f'({self.roll.sample_count} samples)'
        )

    def save_and_shutdown(self):
        sample_count = min(self.roll.sample_count, self.pitch.sample_count)
        if sample_count <= 0:
            self.get_logger().error('No samples were accumulated. Nothing was saved.')
            self.saved = True
            rclpy.shutdown()
            return

        calibration = default_kalman_calibration()
        calibration['calibrated'] = True
        calibration['generated_at'] = datetime.now(timezone.utc).isoformat()
        calibration['warmup_duration_sec'] = self.warmup_duration_sec
        calibration['accumulation_duration_sec'] = self.accumulation_duration_sec
        calibration['sample_count'] = sample_count
        calibration['roll'] = self.roll.build_config()
        calibration['pitch'] = self.pitch.build_config()

        save_kalman_calibration(self.calibration_file, calibration)
        self.saved = True

        self.get_logger().info(f'Saved IMU Kalman calibration to {self.calibration_file}')
        self.get_logger().info(
            'Roll gain='
            f'({calibration["roll"]["kalman_gain_angle"]:.6f}, '
            f'{calibration["roll"]["kalman_gain_bias"]:.6f}), '
            'pitch gain='
            f'({calibration["pitch"]["kalman_gain_angle"]:.6f}, '
            f'{calibration["pitch"]["kalman_gain_bias"]:.6f})'
        )
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = KalmanCalibrationNode()
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
