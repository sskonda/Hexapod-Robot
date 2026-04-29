#!/usr/bin/env python3

"""Publish raw MPU6050 accel/gyro data on a separate fallback IMU topic."""

import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def set_diagonal_covariance(covariance, diagonal_value):
    covariance[0] = diagonal_value
    covariance[4] = diagonal_value
    covariance[8] = diagonal_value


class MPU6050Publisher(Node):
    def __init__(self):
        super().__init__('mpu6050_publisher')

        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('topic', '/imu/mpu6050')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('init_retry_period_sec', 1.0)
        self.declare_parameter('angular_velocity_covariance', 0.02)
        self.declare_parameter('linear_acceleration_covariance', 0.04)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.topic = str(self.get_parameter('topic').value)
        self.publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.i2c_address = int(self.get_parameter('i2c_address').value)
        self.i2c_bus = int(self.get_parameter('i2c_bus').value)
        self.init_retry_period_sec = max(
            0.2,
            float(self.get_parameter('init_retry_period_sec').value),
        )
        self.angular_velocity_covariance = float(
            self.get_parameter('angular_velocity_covariance').value
        )
        self.linear_acceleration_covariance = float(
            self.get_parameter('linear_acceleration_covariance').value
        )

        vendor_path = os.path.join(os.path.dirname(__file__), 'vendor', 'Libs', 'mpu6050')
        if vendor_path not in sys.path:
            sys.path.append(vendor_path)

        from mpu6050.mpu6050 import mpu6050

        self.sensor_class = mpu6050
        self.sensor = None
        self.last_init_attempt_monotonic = 0.0
        self.last_warning_text = ''
        self.publisher_ = self.create_publisher(Imu, self.topic, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_imu)
        self._attempt_initialize_sensor(force=True)

    def _attempt_initialize_sensor(self, force=False):
        now = time.monotonic()
        if (
            not force
            and now - self.last_init_attempt_monotonic < self.init_retry_period_sec
        ):
            return False
        self.last_init_attempt_monotonic = now

        try:
            self.sensor = self.sensor_class(self.i2c_address, bus=self.i2c_bus)
        except Exception as exc:
            self.sensor = None
            self._warn_throttled(
                'MPU6050 fallback IMU initialization failed '
                f'(addr=0x{self.i2c_address:02X}, bus={self.i2c_bus}): {exc}. '
                f'Retrying in {self.init_retry_period_sec:.1f} s.'
            )
            return False

        self.last_warning_text = ''
        self.get_logger().info(
            f'MPU6050 fallback publisher started on {self.topic} with frame {self.frame_id} '
            f'(addr=0x{self.i2c_address:02X}, bus={self.i2c_bus})'
        )
        return True

    def publish_imu(self):
        if self.sensor is None and not self._attempt_initialize_sensor():
            return

        try:
            accel = self.sensor.get_accel_data()
            gyro = self.sensor.get_gyro_data()
        except Exception as exc:
            self.sensor = None
            self._warn_throttled(
                f'MPU6050 fallback IMU read failed: {exc}. Reinitializing sensor.'
            )
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.linear_acceleration.x = float(accel['x'])
        msg.linear_acceleration.y = float(accel['y'])
        msg.linear_acceleration.z = float(accel['z'])

        msg.angular_velocity.x = math.radians(float(gyro['x']))
        msg.angular_velocity.y = math.radians(float(gyro['y']))
        msg.angular_velocity.z = math.radians(float(gyro['z']))

        # The MPU6050 does not provide absolute yaw on its own; locomotion
        # integrates the gyro z-rate as a relative heading fallback.
        msg.orientation_covariance[0] = -1.0
        set_diagonal_covariance(
            msg.angular_velocity_covariance,
            self.angular_velocity_covariance,
        )
        set_diagonal_covariance(
            msg.linear_acceleration_covariance,
            self.linear_acceleration_covariance,
        )

        self.publisher_.publish(msg)

    def _warn_throttled(self, text):
        if text == self.last_warning_text:
            return
        self.last_warning_text = text
        self.get_logger().warn(text)


def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Publisher()
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
