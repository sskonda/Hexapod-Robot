#!/usr/bin/env python3

from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory

from .calibration_store import JOINT_NAMES, load_offsets


class ServoDriver(Node):
    def __init__(self):
        super().__init__('servo_driver')

        share_dir = Path(get_package_share_directory('hexapod_locomotion'))
        default_yaml = str(share_dir / 'config' / 'servo_calibration.yaml')

        self.declare_parameter('calibration_file', default_yaml)
        self.declare_parameter('apply_offsets', True)
        self.declare_parameter('dry_run', True)

        self.calibration_file = self.get_parameter('calibration_file').value
        self.apply_offsets = bool(self.get_parameter('apply_offsets').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)

        self.offsets = load_offsets(self.calibration_file)
        self.warned_not_implemented = False

        self.subscription = self.create_subscription(
            JointState,
            '/servo_targets',
            self.target_callback,
            10
        )

        self.get_logger().info(f'servo_driver listening on /servo_targets')
        self.get_logger().info(f'calibration_file={self.calibration_file}')
        self.get_logger().info(f'apply_offsets={self.apply_offsets}, dry_run={self.dry_run}')

    def target_callback(self, msg: JointState):
        if len(msg.name) != len(msg.position):
            self.get_logger().error('JointState name and position length mismatch')
            return

        commanded = dict(zip(msg.name, msg.position))
        applied = {}

        for joint in JOINT_NAMES:
            if joint not in commanded:
                continue

            value = float(commanded[joint])
            if self.apply_offsets:
                value += self.offsets.get(joint, 0.0)
            applied[joint] = value

        if self.dry_run:
            short = ', '.join([f'{k}={v:.2f}' for k, v in applied.items()])
            self.get_logger().info(f'DRY RUN servo targets -> {short}')
            return

        self.write_hardware(applied)

    def write_hardware(self, applied_targets: dict):
        if not self.warned_not_implemented:
            self.get_logger().warn(
                'Hardware write is not implemented yet. '
                'Replace write_hardware() with Freenove servo/PCA9685 calls.'
            )
            self.warned_not_implemented = True


def main(args=None):
    rclpy.init(args=args)
    node = ServoDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
