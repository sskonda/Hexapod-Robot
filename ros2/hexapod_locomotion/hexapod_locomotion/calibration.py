#!/usr/bin/env python3

import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .calibration_store import (
    JOINT_CHANNELS,
    JOINT_NAMES,
    commanded_angles,
    load_offsets,
    offsets_from_leg_coordinates,
    save_offsets,
)


def map_value(value, from_low, from_high, to_low, to_high):
    return (to_high - to_low) * (value - from_low) / (from_high - from_low) + to_low


_tty_out = open('/dev/tty', 'w')


def tty_print(text):
    _tty_out.write(text)
    _tty_out.flush()


def clear_screen():
    tty_print('\x1b[2J\x1b[H')


class PCA9685:
    MODE1 = 0x00
    PRESCALE = 0xFE
    LED0_ON_L = 0x06
    LED0_ON_H = 0x07
    LED0_OFF_L = 0x08
    LED0_OFF_H = 0x09

    def __init__(self, address=0x40):
        import math
        import smbus

        self._math = math
        self.bus = smbus.SMBus(1)
        self.address = address
        self.write(self.MODE1, 0x00)

    def write(self, reg, value):
        self.bus.write_byte_data(self.address, reg, value)

    def read(self, reg):
        return self.bus.read_byte_data(self.address, reg)

    def set_pwm_freq(self, freq):
        prescaleval = 25000000.0
        prescaleval /= 4096.0
        prescaleval /= float(freq)
        prescaleval -= 1.0
        prescale = self._math.floor(prescaleval + 0.5)

        oldmode = self.read(self.MODE1)
        newmode = (oldmode & 0x7F) | 0x10
        self.write(self.MODE1, newmode)
        self.write(self.PRESCALE, int(self._math.floor(prescale)))
        self.write(self.MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.MODE1, oldmode | 0x80)

    def set_pwm(self, channel, on, off):
        self.write(self.LED0_ON_L + 4 * channel, on & 0xFF)
        self.write(self.LED0_ON_H + 4 * channel, on >> 8)
        self.write(self.LED0_OFF_L + 4 * channel, off & 0xFF)
        self.write(self.LED0_OFF_H + 4 * channel, off >> 8)

    def close(self):
        self.bus.close()


class ServoController:
    def __init__(self):
        self.pwm_40 = PCA9685(0x40)
        self.pwm_41 = PCA9685(0x41)
        self.pwm_40.set_pwm_freq(50)
        time.sleep(0.01)
        self.pwm_41.set_pwm_freq(50)
        time.sleep(0.01)

    def set_servo_angle(self, channel, angle):
        duty_cycle = map_value(angle, 0, 180, 500, 2500)
        duty_cycle = map_value(duty_cycle, 0, 20000, 0, 4095)
        if channel < 16:
            self.pwm_41.set_pwm(channel, 0, int(duty_cycle))
        else:
            self.pwm_40.set_pwm(channel - 16, 0, int(duty_cycle))

    def relax(self):
        for i in range(8):
            self.pwm_41.set_pwm(i + 8, 4096, 4096)
            self.pwm_40.set_pwm(i, 4096, 4096)
            self.pwm_40.set_pwm(i + 8, 4096, 4096)

    def close(self, relax=True):
        if relax:
            self.relax()
        self.pwm_40.close()
        self.pwm_41.close()


class KeyReader:
    def __enter__(self):
        import termios
        import tty
        import os

        self._termios = termios
        self._os = os
        self._tty = open('/dev/tty', 'rb', buffering=0)
        self._fd = self._tty.fileno()
        self._old_settings = termios.tcgetattr(self._fd)
        tty.setraw(self._fd)
        termios.tcflush(self._fd, termios.TCIFLUSH)
        return self

    def __exit__(self, exc_type, exc, tb):
        self._termios.tcsetattr(self._fd, self._termios.TCSADRAIN, self._old_settings)
        self._tty.close()

    def read_key(self):
        b = self._os.read(self._fd, 1)
        return b.decode('latin-1') if b else ''


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


def default_offsets():
    leg_local = []
    for i, point in enumerate(BASE_FOOTPOINTS):
        angle = math.radians(LEG_MOUNT_ANGLES_DEG[i])
        leg_local.append([
            point[0] * math.cos(angle) + point[1] * math.sin(angle) - LEG_X_OFFSETS_MM[i],
            -point[0] * math.sin(angle) + point[1] * math.cos(angle),
            point[2] - LEG_Z_OFFSET_MM,
        ])
    return offsets_from_leg_coordinates(leg_local)


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration')

        self.declare_parameter('calibration_file', '/home/snail/ros2_ws/install/hexapod_locomotion/share/hexapod_locomotion/config/servo_calibration.yaml')
        self.declare_parameter('dry_run', False)
        self.declare_parameter('step_small', 1.0)
        self.declare_parameter('step_large', 5.0)
        self.declare_parameter('relax_on_exit', True)

        self.calibration_file = self.get_parameter('calibration_file').value
        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.step_small = float(self.get_parameter('step_small').value)
        self.step_large = float(self.get_parameter('step_large').value)
        self.relax_on_exit = bool(self.get_parameter('relax_on_exit').value)

        loaded = load_offsets(self.calibration_file)
        if all(v == 0.0 for v in loaded.values()):
            self.offsets = default_offsets()
        else:
            self.offsets = loaded
        self.selected_index = 0
        self.status_message = 'Ready'
        self.publisher = self.create_publisher(JointState, '/servo_targets', 10)
        self.servo = self._create_servo()

        self.get_logger().info(f'Calibration file: {self.calibration_file}')
        self.get_logger().info(f'dry_run={self.dry_run}')

    def _create_servo(self):
        if self.dry_run:
            return None

        try:
            return ServoController()
        except Exception as exc:
            self.dry_run = True
            self.status_message = f'Hardware init failed, running dry-run: {exc}'
            return None

    def selected_joint(self):
        return JOINT_NAMES[self.selected_index]

    def current_angles(self):
        return commanded_angles(self.offsets)

    def publish_pose(self):
        angles = self.current_angles()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [angles[name] for name in JOINT_NAMES]
        self.publisher.publish(msg)

    def apply_pose(self):
        angles = self.current_angles()
        self.publish_pose()
        if self.servo is None:
            return

        for joint in JOINT_NAMES:
            self.servo.set_servo_angle(JOINT_CHANNELS[joint], angles[joint])

    def move_selection(self, delta):
        self.selected_index = (self.selected_index + delta) % len(JOINT_NAMES)

    def adjust_selected(self, delta):
        joint = self.selected_joint()
        self.offsets[joint] = self.offsets.get(joint, 0.0) + delta
        self.apply_pose()

    def zero_selected(self):
        joint = self.selected_joint()
        self.offsets[joint] = 0.0
        self.apply_pose()

    def reset_all(self):
        for joint in JOINT_NAMES:
            self.offsets[joint] = 0.0
        self.apply_pose()

    def save(self):
        save_offsets(self.calibration_file, self.offsets)
        self.status_message = f'Saved calibration to {self.calibration_file}'

    def render(self):
        angles = self.current_angles()
        clear_screen()
        lines = [
            'Hexapod Servo Calibration',
            '',
            (
                f'Keys: [ / ] select  a/d +/-{self.step_small:g}  '
                f'z/c +/-{self.step_large:g}  0 zero joint  r reset all  '
                's save  q save+quit  x quit'
            ),
            '',
        ]
        for index, joint in enumerate(JOINT_NAMES):
            marker = '>' if index == self.selected_index else ' '
            channel = JOINT_CHANNELS[joint]
            offset = self.offsets.get(joint, 0.0)
            angle = angles[joint]
            lines.append(
                f'{marker} {joint:<11} ch={channel:<2} offset={offset:>6.1f} target={angle:>6.1f}'
            )
        if self.dry_run:
            lines.extend(['', 'Dry-run mode is active. No hardware commands are being sent.'])
        lines.extend(['', f'Status: {self.status_message}'])
        tty_print('\r\n'.join(lines) + '\r\n')

    def run(self):
        self.apply_pose()
        self.render()

        with KeyReader() as reader:
            while rclpy.ok():
                key = reader.read_key()
                if not key:
                    continue

                if key == '[':
                    self.move_selection(-1)
                elif key == ']':
                    self.move_selection(1)
                elif key == 'a':
                    self.adjust_selected(-self.step_small)
                elif key == 'd':
                    self.adjust_selected(self.step_small)
                elif key == 'z':
                    self.adjust_selected(-self.step_large)
                elif key == 'c':
                    self.adjust_selected(self.step_large)
                elif key == '0':
                    self.zero_selected()
                elif key == 'r':
                    self.reset_all()
                elif key == 's':
                    self.save()
                elif key == 'q':
                    self.save()
                    break
                elif key == 'x':
                    self.status_message = 'Exited without saving'
                    break
                else:
                    self.status_message = f'Unknown key: {repr(key)}'

                self.render()

    def shutdown(self):
        if self.servo is not None:
            self.servo.close(relax=self.relax_on_exit)


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
