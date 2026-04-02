#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class LocomotionNode(Node):
    def __init__(self):
        super().__init__('locomotion')

        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            10
        )

        self.get_logger().info('Locomotion node subscribed to /imu/data_raw')

    def imu_callback(self, msg: Imu):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gz = msg.angular_velocity.z

        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        self.get_logger().info(
            f'roll={math.degrees(roll):.2f} deg, '
            f'pitch={math.degrees(pitch):.2f} deg, '
            f'gz={math.degrees(gz):.2f} deg/s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = LocomotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
