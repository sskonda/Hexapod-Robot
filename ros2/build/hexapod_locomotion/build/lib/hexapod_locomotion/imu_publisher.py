#!/usr/bin/env python3

import math
import os
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

sys.path.append(os.path.join(os.path.dirname(__file__), 'vendor'))
from Libs.mpu6050.mpu6050.mpu6050 import mpu6050


class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        self.publisher_ = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.timer = self.create_timer(0.1, self.publish_imu)

        self.sensor = mpu6050(0x68)
        self.frame_id = 'imu_link'

        self.get_logger().info('IMU publisher started on /imu/data_raw')

    def publish_imu(self):
        accel = self.sensor.get_accel_data()
        gyro = self.sensor.get_gyro_data()

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.linear_acceleration.x = accel['x']
        msg.linear_acceleration.y = accel['y']
        msg.linear_acceleration.z = accel['z']

        msg.angular_velocity.x = math.radians(gyro['x'])
        msg.angular_velocity.y = math.radians(gyro['y'])
        msg.angular_velocity.z = math.radians(gyro['z'])

        msg.orientation_covariance[0] = -1.0

        msg.angular_velocity_covariance[0] = 0.02
        msg.angular_velocity_covariance[4] = 0.02
        msg.angular_velocity_covariance[8] = 0.02

        msg.linear_acceleration_covariance[0] = 0.04
        msg.linear_acceleration_covariance[4] = 0.04
        msg.linear_acceleration_covariance[8] = 0.04

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
