#!/usr/bin/env python3

"""UART BNO055 publisher for the hexapod robot.

This node publishes:
  - ``sensor_msgs/msg/Imu`` on ``/imu/data_raw`` with accel, gyro, and a
    tilt-compensated orientation estimate derived from accel + magnetometer
  - ``sensor_msgs/msg/MagneticField`` on ``/imu/mag`` with the raw
    magnetometer measurement

The goal is to preserve the MPU6050-era ROS interface while upgrading heading
hold to use the BNO055 magnetometer explicitly for yaw drift correction.
"""

import math
import struct
import time
import traceback

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

try:
    import serial
except ImportError:  # pragma: no cover - exercised only on systems without pyserial.
    serial = None


def vector_is_valid(values, expected_length):
    return (
        values is not None
        and len(values) == expected_length
        and all(value is not None for value in values)
    )


def set_diagonal_covariance(covariance, diagonal_value):
    covariance[0] = diagonal_value
    covariance[4] = diagonal_value
    covariance[8] = diagonal_value


def normalize_angle(angle_rad):
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def quaternion_from_euler(roll_rad, pitch_rad, yaw_rad):
    half_roll = roll_rad * 0.5
    half_pitch = pitch_rad * 0.5
    half_yaw = yaw_rad * 0.5

    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def orientation_from_accel_and_mag(accel_m_s2, magnetic_field_t):
    ax, ay, az = accel_m_s2
    mx, my, mz = magnetic_field_t

    accel_norm = math.sqrt(ax * ax + ay * ay + az * az)
    mag_norm = math.sqrt(mx * mx + my * my + mz * mz)
    if accel_norm < 1e-6 or mag_norm < 1e-12:
        return None

    roll_rad = math.atan2(ay, az)
    pitch_rad = math.atan2(-ax, math.sqrt(ay * ay + az * az))

    cos_roll = math.cos(roll_rad)
    sin_roll = math.sin(roll_rad)
    cos_pitch = math.cos(pitch_rad)
    sin_pitch = math.sin(pitch_rad)

    mag_x = mx * cos_pitch + mz * sin_pitch
    mag_y = (
        mx * sin_roll * sin_pitch
        + my * cos_roll
        - mz * sin_roll * cos_pitch
    )
    yaw_rad = normalize_angle(math.atan2(-mag_y, mag_x))

    return roll_rad, pitch_rad, yaw_rad


class BNO055ProtocolError(RuntimeError):
    """Raised when the BNO055 UART protocol reports a failure."""


class BNO055UART:
    READ_START_BYTE = 0xAA
    READ_COMMAND = 0x01
    WRITE_COMMAND = 0x00
    READ_RESPONSE = 0xBB
    ACK_RESPONSE = 0xEE

    CHIP_ID_REGISTER = 0x00
    PAGE_ID_REGISTER = 0x07
    ACCEL_DATA_X_LSB_REGISTER = 0x08
    MAG_DATA_X_LSB_REGISTER = 0x0E
    GYRO_DATA_X_LSB_REGISTER = 0x14
    CALIB_STAT_REGISTER = 0x35
    UNIT_SEL_REGISTER = 0x3B
    OPR_MODE_REGISTER = 0x3D
    PWR_MODE_REGISTER = 0x3E
    SYS_TRIGGER_REGISTER = 0x3F

    CHIP_ID = 0xA0
    CONFIG_MODE = 0x00
    NORMAL_POWER_MODE = 0x00

    MODE_VALUES = {
        'CONFIG_MODE': 0x00,
        'ACCONLY_MODE': 0x01,
        'MAGONLY_MODE': 0x02,
        'GYRONLY_MODE': 0x03,
        'ACCMAG_MODE': 0x04,
        'ACCGYRO_MODE': 0x05,
        'MAGGYRO_MODE': 0x06,
        'AMG_MODE': 0x07,
        'IMUPLUS_MODE': 0x08,
        'COMPASS_MODE': 0x09,
        'M4G_MODE': 0x0A,
        'NDOF_FMC_OFF_MODE': 0x0B,
        'NDOF_MODE': 0x0C,
    }

    RESPONSE_CODES = {
        0x01: 'write success',
        0x02: 'read failure',
        0x03: 'write failure',
        0x04: 'invalid register map address',
        0x05: 'register map write disabled',
        0x06: 'wrong start byte',
        0x07: 'bus overrun',
        0x08: 'maximum length error',
        0x09: 'minimum length error',
        0x0A: 'receive timeout',
    }

    def __init__(
        self,
        port,
        baud_rate,
        operation_mode,
        use_external_crystal,
        timeout_sec=0.2,
    ):
        if serial is None:
            raise RuntimeError('Missing pyserial dependency. Install python3-serial on the robot.')

        self.port = port
        self.serial = serial.Serial(
            port=port,
            baudrate=baud_rate,
            timeout=timeout_sec,
            write_timeout=timeout_sec,
        )

        time.sleep(0.1)
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

        chip_id = self.read_u8(self.CHIP_ID_REGISTER)
        if chip_id != self.CHIP_ID:
            raise RuntimeError(
                f'BNO055 on {self.port} returned CHIP_ID 0x{chip_id:02X} '
                f'instead of 0x{self.CHIP_ID:02X}.'
            )

        self.configure(operation_mode, use_external_crystal)

    def close(self):
        if getattr(self, 'serial', None) is not None and self.serial.is_open:
            self.serial.close()

    def configure(self, operation_mode, use_external_crystal):
        mode_value = self.MODE_VALUES.get(operation_mode)
        if mode_value is None:
            supported = ', '.join(sorted(self.MODE_VALUES))
            raise ValueError(
                f'Unsupported BNO055 mode "{operation_mode}". Supported modes: {supported}'
            )

        self.write_u8(self.PAGE_ID_REGISTER, 0x00)
        self.write_u8(self.OPR_MODE_REGISTER, self.CONFIG_MODE)
        time.sleep(0.03)

        self.write_u8(self.PWR_MODE_REGISTER, self.NORMAL_POWER_MODE)
        time.sleep(0.01)

        # Keep the sensor on the default engineering units:
        # accel in m/s^2, gyro in deg/s, Euler in degrees, temperature in C.
        self.write_u8(self.UNIT_SEL_REGISTER, 0x00)

        self.write_u8(self.SYS_TRIGGER_REGISTER, 0x80 if use_external_crystal else 0x00)
        time.sleep(0.05 if use_external_crystal else 0.01)

        self.write_u8(self.OPR_MODE_REGISTER, mode_value)
        time.sleep(0.03)

    def read_u8(self, register):
        return self.read_registers(register, 1)[0]

    def write_u8(self, register, value):
        self.write_registers(register, bytes([value & 0xFF]))

    def read_registers(self, register, length):
        self.serial.reset_input_buffer()
        packet = bytes([
            self.READ_START_BYTE,
            self.READ_COMMAND,
            register & 0xFF,
            length & 0xFF,
        ])
        self.serial.write(packet)
        self.serial.flush()

        header = self._read_exact(2)
        if header[0] == self.READ_RESPONSE:
            response_length = header[1]
            payload = self._read_exact(response_length)
            if response_length != length:
                raise BNO055ProtocolError(
                    f'Expected {length} bytes from register 0x{register:02X}, '
                    f'but received {response_length}.'
                )
            return payload

        if header[0] == self.ACK_RESPONSE:
            error_code = header[1]
            raise BNO055ProtocolError(
                f'Read from register 0x{register:02X} failed: {self.describe_response(error_code)}'
            )

        raise BNO055ProtocolError(
            f'Unexpected BNO055 response header 0x{header[0]:02X} '
            f'while reading register 0x{register:02X}.'
        )

    def write_registers(self, register, payload):
        self.serial.reset_input_buffer()
        packet = bytes([
            self.READ_START_BYTE,
            self.WRITE_COMMAND,
            register & 0xFF,
            len(payload) & 0xFF,
        ]) + payload
        self.serial.write(packet)
        self.serial.flush()

        response = self._read_exact(2)
        if response[0] != self.ACK_RESPONSE or response[1] != 0x01:
            error_code = response[1] if len(response) > 1 else 0x00
            raise BNO055ProtocolError(
                f'Write to register 0x{register:02X} failed: {self.describe_response(error_code)}'
            )

    def describe_response(self, code):
        return self.RESPONSE_CODES.get(code, f'unknown response code 0x{code:02X}')

    def _read_exact(self, length):
        data = self.serial.read(length)
        if len(data) != length:
            raise BNO055ProtocolError(
                f'Timeout while reading {length} byte(s) from BNO055 on {self.port}.'
            )
        return data

    def _read_vector(self, register):
        return struct.unpack('<hhh', self.read_registers(register, 6))

    def read_acceleration_m_s2(self):
        raw_values = self._read_vector(self.ACCEL_DATA_X_LSB_REGISTER)
        return tuple(value / 100.0 for value in raw_values)

    def read_gyro_rad_s(self):
        raw_values = self._read_vector(self.GYRO_DATA_X_LSB_REGISTER)
        return tuple(math.radians(value / 16.0) for value in raw_values)

    def read_magnetic_field_t(self):
        raw_values = self._read_vector(self.MAG_DATA_X_LSB_REGISTER)
        return tuple((value / 16.0) * 1e-6 for value in raw_values)

    def read_calibration_status(self):
        value = self.read_u8(self.CALIB_STAT_REGISTER)
        return (
            (value >> 6) & 0x03,
            (value >> 4) & 0x03,
            (value >> 2) & 0x03,
            value & 0x03,
        )


class BNO055Publisher(Node):
    def __init__(self):
        super().__init__('bno055_publisher')

        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('topic', '/imu/data_raw')
        self.declare_parameter('mag_topic', '/imu/mag')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('uart_port', '/dev/ttyAMA5')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('mode', 'AMG_MODE')
        self.declare_parameter('use_external_crystal', True)
        self.declare_parameter('orientation_covariance', 0.05)
        self.declare_parameter('angular_velocity_covariance', 0.02)
        self.declare_parameter('linear_acceleration_covariance', 0.04)
        self.declare_parameter('magnetic_field_covariance', 1e-6)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.topic = str(self.get_parameter('topic').value)
        self.mag_topic = str(self.get_parameter('mag_topic').value)
        self.publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.uart_port = str(self.get_parameter('uart_port').value)
        self.baud_rate = int(self.get_parameter('baud_rate').value)
        self.mode_name = str(self.get_parameter('mode').value).strip()
        self.use_external_crystal = bool(self.get_parameter('use_external_crystal').value)
        self.orientation_covariance = float(self.get_parameter('orientation_covariance').value)
        self.angular_velocity_covariance = float(
            self.get_parameter('angular_velocity_covariance').value
        )
        self.linear_acceleration_covariance = float(
            self.get_parameter('linear_acceleration_covariance').value
        )
        self.magnetic_field_covariance = float(
            self.get_parameter('magnetic_field_covariance').value
        )
        self.last_warning_text = ''

        self.sensor = BNO055UART(
            port=self.uart_port,
            baud_rate=self.baud_rate,
            operation_mode=self.mode_name,
            use_external_crystal=self.use_external_crystal,
        )
        self.imu_publisher = self.create_publisher(Imu, self.topic, 10)
        self.mag_publisher = self.create_publisher(MagneticField, self.mag_topic, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_measurements)

        calibration = self.sensor.read_calibration_status()
        self.get_logger().info(
            f'BNO055 publisher started on {self.topic} and {self.mag_topic} '
            f'from {self.uart_port} @ {self.baud_rate} baud, mode {self.mode_name}'
        )
        self.get_logger().info(
            'Initial calibration status '
            f'(sys={calibration[0]}, gyro={calibration[1]}, accel={calibration[2]}, mag={calibration[3]})'
        )

    def destroy_node(self):
        if getattr(self, 'sensor', None) is not None:
            self.sensor.close()
        return super().destroy_node()

    def publish_measurements(self):
        try:
            accel = self.sensor.read_acceleration_m_s2()
            gyro = self.sensor.read_gyro_rad_s()
            magnetic_field = self.sensor.read_magnetic_field_t()
            orientation = orientation_from_accel_and_mag(accel, magnetic_field)
        except Exception as exc:
            self._warn_throttled(f'BNO055 read failed: {exc}')
            return

        if not vector_is_valid(accel, 3) or not vector_is_valid(gyro, 3):
            self._warn_throttled('BNO055 acceleration or gyro data is unavailable.')
            return

        stamp = self.get_clock().now().to_msg()

        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.frame_id
        imu_msg.linear_acceleration.x = float(accel[0])
        imu_msg.linear_acceleration.y = float(accel[1])
        imu_msg.linear_acceleration.z = float(accel[2])
        imu_msg.angular_velocity.x = float(gyro[0])
        imu_msg.angular_velocity.y = float(gyro[1])
        imu_msg.angular_velocity.z = float(gyro[2])
        set_diagonal_covariance(
            imu_msg.angular_velocity_covariance,
            self.angular_velocity_covariance,
        )
        set_diagonal_covariance(
            imu_msg.linear_acceleration_covariance,
            self.linear_acceleration_covariance,
        )

        if orientation is None:
            imu_msg.orientation_covariance[0] = -1.0
        else:
            orientation_quaternion = quaternion_from_euler(*orientation)
            imu_msg.orientation.x = float(orientation_quaternion[0])
            imu_msg.orientation.y = float(orientation_quaternion[1])
            imu_msg.orientation.z = float(orientation_quaternion[2])
            imu_msg.orientation.w = float(orientation_quaternion[3])
            set_diagonal_covariance(
                imu_msg.orientation_covariance,
                self.orientation_covariance,
            )

        self.imu_publisher.publish(imu_msg)

        if vector_is_valid(magnetic_field, 3):
            mag_msg = MagneticField()
            mag_msg.header.stamp = stamp
            mag_msg.header.frame_id = self.frame_id
            mag_msg.magnetic_field.x = float(magnetic_field[0])
            mag_msg.magnetic_field.y = float(magnetic_field[1])
            mag_msg.magnetic_field.z = float(magnetic_field[2])
            set_diagonal_covariance(
                mag_msg.magnetic_field_covariance,
                self.magnetic_field_covariance,
            )
            self.mag_publisher.publish(mag_msg)

    def _warn_throttled(self, text):
        if text == self.last_warning_text:
            return
        self.last_warning_text = text
        self.get_logger().warn(text)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = BNO055Publisher()
    except Exception as exc:
        logger = rclpy.logging.get_logger('bno055_publisher')
        logger.error(str(exc))
        logger.debug(traceback.format_exc())
        if rclpy.ok():
            rclpy.shutdown()
        raise

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
