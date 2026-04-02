#!/usr/bin/env python3

from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory

from .calibration import ServoController
from .calibration_store import JOINT_CHANNELS, JOINT_NAMES, load_offsets


class ServoDriver(Node):
    def __init__(self):
        super().__init__('servo_driver')

        share_dir = Path(get_package_share_directory('hexapod_locomotion'))
        default_yaml = str(share_dir / 'config' / 'servo_calibration.yaml')

        self.declare_parameter('calibration_file', default_yaml)
        self.declare_parameter('apply_offsets', True)
        self.declare_parameter('dry_run', True)
        self.declare_parameter('relax_on_exit', False)
        self.declare_parameter('enable_power_control', True)
        self.declare_parameter('servo_power_disable_gpio', 4)
        self.declare_parameter('disable_power_on_exit', False)

        self.calibration_file = self.get_parameter('calibration_file').value
        self.apply_offsets = bool(self.get_parameter('apply_offsets').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.relax_on_exit = bool(self.get_parameter('relax_on_exit').value)
        self.enable_power_control = bool(self.get_parameter('enable_power_control').value)
        self.servo_power_disable_gpio = int(self.get_parameter('servo_power_disable_gpio').value)
        self.disable_power_on_exit = bool(self.get_parameter('disable_power_on_exit').value)
        self.dry_run_reason = 'dry_run parameter enabled' if self.dry_run else ''

        self.offsets = load_offsets(self.calibration_file)
        self.servo_power_disable = None
        self.configure_power_control()
        self.servo = self._create_servo()

        self.subscription = self.create_subscription(
            JointState,
            '/servo_targets',
            self.target_callback,
            10
        )

        self.get_logger().info(f'servo_driver listening on /servo_targets')
        self.get_logger().info(f'calibration_file={self.calibration_file}')
        self.get_logger().info(f'apply_offsets={self.apply_offsets}, dry_run={self.dry_run}')
        if self.dry_run:
            self.get_logger().warn(
                'servo_driver is in DRY RUN mode, so the robot will not move. '
                'Set dry_run:=false or launch with servo_dry_run:=false for hardware output.'
            )
            if self.dry_run_reason:
                self.get_logger().warn(f'dry-run reason: {self.dry_run_reason}')

    def configure_power_control(self):
        if not self.enable_power_control or self.dry_run:
            return

        try:
            from gpiozero import OutputDevice

            self.servo_power_disable = OutputDevice(self.servo_power_disable_gpio)
            # Matches Sample_Code/Server/control.py where driving GPIO 4 low enables servo power.
            self.servo_power_disable.off()
            self.get_logger().info(
                f'Servo power enabled using GPIO {self.servo_power_disable_gpio}'
            )
        except Exception as exc:
            self.get_logger().warn(
                f'Unable to configure servo power GPIO {self.servo_power_disable_gpio}: {exc}'
            )

    def _create_servo(self):
        if self.dry_run:
            return None

        try:
            return ServoController()
        except Exception as exc:
            self.dry_run_reason = f'hardware init failed: {exc}'
            self.get_logger().error(
                'Hardware init failed, falling back to dry-run mode. '
                'Check I2C access, the PCA9685 boards, and Python hardware dependencies. '
                f'Original error: {exc}'
            )
            self.dry_run = True
            return None

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
        if self.servo is None:
            self.get_logger().warn('Servo controller is unavailable; skipping hardware write')
            return

        for joint in JOINT_NAMES:
            if joint not in applied_targets:
                continue
            self.servo.set_servo_angle(JOINT_CHANNELS[joint], applied_targets[joint])

    def destroy_node(self):
        if self.servo is not None:
            self.servo.close(relax=self.relax_on_exit)
            self.servo = None
        if self.servo_power_disable is not None and self.disable_power_on_exit:
            self.servo_power_disable.on()
        self.servo_power_disable = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoDriver()
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
