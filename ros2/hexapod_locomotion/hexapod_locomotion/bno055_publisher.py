#!/usr/bin/env python3

"""UART BNO055 publisher for the hexapod robot.

This node publishes:
  - ``sensor_msgs/msg/Imu`` on ``/imu/data_raw`` with accel, gyro, and a
    tilt-compensated orientation estimate whose yaw is fused from gyro +
    magnetometer
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
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

from .kalman_filter import YawComplementaryFilter
from .yaw_control import StartupStillnessGate, resolve_parameter_value, vector_norm

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
    RAW_IMU_BLOCK_LENGTH = 18
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
    RETRYABLE_READ_RESPONSE_CODES = {0x07, 0x0A}

    def __init__(
        self,
        port,
        baud_rate,
        operation_mode,
        use_external_crystal,
        timeout_sec=0.2,
        read_retry_count=3,
        retry_backoff_sec=0.01,
    ):
        if serial is None:
            raise RuntimeError('Missing pyserial dependency. Install python3-serial on the robot.')

        self.port = port
        self.read_retry_count = max(0, int(read_retry_count))
        self.retry_backoff_sec = max(0.0, float(retry_backoff_sec))
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
        for attempt in range(self.read_retry_count + 1):
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
                if (
                    error_code in self.RETRYABLE_READ_RESPONSE_CODES
                    and attempt < self.read_retry_count
                ):
                    time.sleep(self.retry_backoff_sec * (attempt + 1))
                    continue
                raise BNO055ProtocolError(
                    f'Read from register 0x{register:02X} failed: {self.describe_response(error_code)}'
                )

            raise BNO055ProtocolError(
                f'Unexpected BNO055 response header 0x{header[0]:02X} '
                f'while reading register 0x{register:02X}.'
            )

        raise BNO055ProtocolError(
            f'Read from register 0x{register:02X} failed after retries.'
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

    def read_imu_measurements(self):
        """Read accel, magnetometer, and gyro in one burst transaction."""
        raw_values = struct.unpack(
            '<hhhhhhhhh',
            self.read_registers(
                self.ACCEL_DATA_X_LSB_REGISTER,
                self.RAW_IMU_BLOCK_LENGTH,
            ),
        )
        accel_raw = raw_values[0:3]
        mag_raw = raw_values[3:6]
        gyro_raw = raw_values[6:9]
        return (
            tuple(value / 100.0 for value in accel_raw),
            tuple((value / 16.0) * 1e-6 for value in mag_raw),
            tuple(math.radians(value / 16.0) for value in gyro_raw),
        )

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
    DEFAULT_YAW_FILTER_TIME_CONSTANT_SEC = 0.5

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
        self.declare_parameter('read_retry_count', 3)
        self.declare_parameter('retry_backoff_sec', 0.01)
        self.declare_parameter('orientation_covariance', 0.05)
        self.declare_parameter('angular_velocity_covariance', 0.02)
        self.declare_parameter('linear_acceleration_covariance', 0.04)
        self.declare_parameter('magnetic_field_covariance', 1e-6)
        self.declare_parameter('min_mag_calibration_for_yaw', 3)
        self.declare_parameter('calibration_status_period_sec', 0.5)
        self.declare_parameter(
            'imu_yaw_filter_time_constant_sec',
            self.DEFAULT_YAW_FILTER_TIME_CONSTANT_SEC,
        )
        self.declare_parameter(
            'yaw_filter_time_constant_sec',
            self.DEFAULT_YAW_FILTER_TIME_CONSTANT_SEC,
        )
        self.declare_parameter('imu_startup_still_time_sec', 15.0)
        self.declare_parameter('imu_startup_accel_still_tolerance_m_s2', 0.75)
        self.declare_parameter('imu_startup_gyro_still_tolerance_rad_s', 0.12)
        self.declare_parameter('imu_startup_motion_grace_sec', 0.5)
        self.declare_parameter('imu_startup_status_log_interval_sec', 2.0)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.topic = str(self.get_parameter('topic').value)
        self.mag_topic = str(self.get_parameter('mag_topic').value)
        self.publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.uart_port = str(self.get_parameter('uart_port').value)
        self.baud_rate = int(self.get_parameter('baud_rate').value)
        self.mode_name = str(self.get_parameter('mode').value).strip()
        self.use_external_crystal = bool(self.get_parameter('use_external_crystal').value)
        self.read_retry_count = max(0, int(self.get_parameter('read_retry_count').value))
        self.retry_backoff_sec = max(0.0, float(self.get_parameter('retry_backoff_sec').value))
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
        self.min_mag_calibration_for_yaw = min(
            3,
            max(0, int(self.get_parameter('min_mag_calibration_for_yaw').value)),
        )
        self.calibration_status_period_sec = max(
            0.05,
            float(self.get_parameter('calibration_status_period_sec').value),
        )
        self.yaw_filter_time_constant_sec, yaw_param_conflict = resolve_parameter_value(
            float(self.get_parameter('imu_yaw_filter_time_constant_sec').value),
            float(self.get_parameter('yaw_filter_time_constant_sec').value),
            self.DEFAULT_YAW_FILTER_TIME_CONSTANT_SEC,
        )
        self.yaw_filter_time_constant_sec = max(
            1e-3,
            self.yaw_filter_time_constant_sec,
        )
        self.startup_still_time_sec = max(
            0.0,
            float(self.get_parameter('imu_startup_still_time_sec').value),
        )
        self.startup_accel_still_tolerance_m_s2 = max(
            0.0,
            float(self.get_parameter('imu_startup_accel_still_tolerance_m_s2').value),
        )
        self.startup_gyro_still_tolerance_rad_s = max(
            0.0,
            float(self.get_parameter('imu_startup_gyro_still_tolerance_rad_s').value),
        )
        self.startup_motion_grace_sec = max(
            0.0,
            float(self.get_parameter('imu_startup_motion_grace_sec').value),
        )
        self.startup_status_log_interval_sec = max(
            0.5,
            float(self.get_parameter('imu_startup_status_log_interval_sec').value),
        )
        self.last_warning_text = ''
        self.last_measurement_monotonic = None
        self.last_startup_status_log_monotonic = 0.0
        self.last_calibration_read_monotonic = 0.0
        self.last_calibration_status = (0, 0, 0, 0)
        self.last_yaw_source = None
        self.startup_still_gate = StartupStillnessGate(
            required_still_time_sec=self.startup_still_time_sec,
            accel_tolerance_m_s2=self.startup_accel_still_tolerance_m_s2,
            gyro_tolerance_rad_s=self.startup_gyro_still_tolerance_rad_s,
            motion_grace_sec=self.startup_motion_grace_sec,
        )
        self.startup_settle_logged = self.startup_still_gate.ready

        self.sensor = BNO055UART(
            port=self.uart_port,
            baud_rate=self.baud_rate,
            operation_mode=self.mode_name,
            use_external_crystal=self.use_external_crystal,
            read_retry_count=self.read_retry_count,
            retry_backoff_sec=self.retry_backoff_sec,
        )
        self.yaw_filter = YawComplementaryFilter(
            time_constant_sec=self.yaw_filter_time_constant_sec,
        )
        self.imu_publisher = self.create_publisher(Imu, self.topic, 10)
        self.mag_publisher = self.create_publisher(MagneticField, self.mag_topic, 10)
        self.add_on_set_parameters_callback(self.parameter_update_callback)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_measurements)

        calibration = self._read_calibration_status_cached(time.monotonic(), force=True)
        self.get_logger().info(
            f'BNO055 publisher started on {self.topic} and {self.mag_topic} '
            f'from {self.uart_port} @ {self.baud_rate} baud, mode {self.mode_name}'
        )
        self.get_logger().info(
            'Initial calibration status '
            f'(sys={calibration[0]}, gyro={calibration[1]}, accel={calibration[2]}, mag={calibration[3]})'
        )
        if yaw_param_conflict:
            self.get_logger().warn(
                'Both imu_yaw_filter_time_constant_sec and yaw_filter_time_constant_sec were set. '
                'Using imu_yaw_filter_time_constant_sec.'
            )
        if self.startup_still_time_sec > 0.0:
            self.get_logger().info(
                'IMU startup settle enabled: keep the robot still for '
                f'{self.startup_still_time_sec:.1f} s before yaw heading hold activates.'
            )
        if self.min_mag_calibration_for_yaw > 0:
            self.get_logger().info(
                'Magnetometer yaw correction requires mag calibration >= '
                f'{self.min_mag_calibration_for_yaw}; below that, yaw will be gyro-only.'
            )

    def parameter_update_callback(self, parameters):
        changed_fields = []

        for parameter in parameters:
            if parameter.name in (
                'imu_yaw_filter_time_constant_sec',
                'yaw_filter_time_constant_sec',
            ):
                value = float(parameter.value)
                if value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='yaw filter time constant must be greater than zero.',
                    )
                self.yaw_filter_time_constant_sec = value
                self.yaw_filter.time_constant_sec = value
                changed_fields.append(
                    f'{parameter.name}={self.yaw_filter_time_constant_sec:.3f}'
                )
            elif parameter.name == 'min_mag_calibration_for_yaw':
                value = int(parameter.value)
                if value < 0 or value > 3:
                    return SetParametersResult(
                        successful=False,
                        reason='min_mag_calibration_for_yaw must be in the range 0..3.',
                    )
                self.min_mag_calibration_for_yaw = value
                changed_fields.append(
                    f'min_mag_calibration_for_yaw={self.min_mag_calibration_for_yaw}'
                )

        if changed_fields:
            self.get_logger().info(
                'Updated BNO055 yaw tuning: ' + ', '.join(changed_fields)
            )

        return SetParametersResult(successful=True)

    def destroy_node(self):
        if getattr(self, 'sensor', None) is not None:
            self.sensor.close()
        return super().destroy_node()

    def publish_measurements(self):
        try:
            accel, magnetic_field, gyro = self.sensor.read_imu_measurements()
            orientation = orientation_from_accel_and_mag(accel, magnetic_field)
        except Exception as exc:
            self._warn_throttled(f'BNO055 read failed: {exc}')
            return

        if not vector_is_valid(accel, 3) or not vector_is_valid(gyro, 3):
            self._warn_throttled('BNO055 acceleration or gyro data is unavailable.')
            return

        measurement_time = time.monotonic()
        if self.last_measurement_monotonic is None:
            dt = 1.0 / self.publish_rate_hz
        else:
            dt = measurement_time - self.last_measurement_monotonic
            dt = max(1e-4, min(dt, 1.0))
        self.last_measurement_monotonic = measurement_time
        accumulated_still_time_sec = self.startup_still_gate.accumulated_still_time_sec
        startup_ready = self.startup_still_gate.update(dt, accel, gyro)
        calibration = self._read_calibration_status_cached(measurement_time)

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

        if not startup_ready:
            if accumulated_still_time_sec > 0.0 and not self.startup_still_gate.is_still:
                self._warn_throttled(
                    'BNO055 startup settle was interrupted by motion. '
                    'Hold the robot still to restart the settle timer.'
                )
            if orientation is None:
                self.yaw_filter.reset()
            else:
                self.yaw_filter.reset(orientation[2])
            imu_msg.orientation_covariance[0] = -1.0
            self._log_startup_status(measurement_time, accel, gyro)
        elif orientation is None:
            imu_msg.orientation_covariance[0] = -1.0
        else:
            if not self.startup_settle_logged:
                self.get_logger().info(
                    'IMU startup settle complete; yaw heading hold enabled. '
                    f'Calibration status (sys={calibration[0]}, gyro={calibration[1]}, '
                    f'accel={calibration[2]}, mag={calibration[3]}).'
                )
                self.startup_settle_logged = True
            roll_rad, pitch_rad, measured_yaw_rad = orientation
            if calibration[3] >= self.min_mag_calibration_for_yaw:
                filtered_yaw_rad = self.yaw_filter.update(
                    measured_yaw_rad,
                    gyro[2],
                    dt,
                )
                self._log_yaw_source_once('mag')
            else:
                filtered_yaw_rad = self.yaw_filter.predict(
                    gyro[2],
                    dt,
                    fallback_yaw_rad=measured_yaw_rad,
                )
                self._log_yaw_source_once('gyro_only', calibration)
            orientation_quaternion = quaternion_from_euler(
                roll_rad,
                pitch_rad,
                filtered_yaw_rad,
            )
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

    def _read_calibration_status_cached(self, measurement_time, force=False):
        if (
            force
            or measurement_time - self.last_calibration_read_monotonic
            >= self.calibration_status_period_sec
        ):
            self.last_calibration_status = self.sensor.read_calibration_status()
            self.last_calibration_read_monotonic = measurement_time

        return self.last_calibration_status

    def _log_yaw_source_once(self, yaw_source, calibration=None):
        if yaw_source == self.last_yaw_source:
            return

        self.last_yaw_source = yaw_source
        if yaw_source == 'mag':
            self.get_logger().info(
                'Using magnetometer-corrected yaw because mag calibration is high enough.'
            )
            return

        if calibration is None:
            calibration = self.last_calibration_status
        self.get_logger().warn(
            'Magnetometer calibration is too low for absolute yaw correction '
            f'(mag={calibration[3]}, required={self.min_mag_calibration_for_yaw}). '
            'Publishing gyro-only relative yaw to avoid magnetic heading jumps.'
        )

    def _log_startup_status(self, measurement_time, accel, gyro):
        if self.startup_still_gate.ready:
            return

        if (
            measurement_time - self.last_startup_status_log_monotonic
            < self.startup_status_log_interval_sec
        ):
            return

        self.last_startup_status_log_monotonic = measurement_time
        calibration = self._read_calibration_status_cached(measurement_time, force=True)
        self.get_logger().info(
            'IMU startup settle in progress: '
            f'{self.startup_still_gate.remaining_time_sec:.1f} s still remaining, '
            f'gyro_norm={vector_norm(gyro):.3f} rad/s, '
            f'accel_error={abs(vector_norm(accel) - 9.80665):.3f} m/s^2, '
            f'calib=(sys={calibration[0]}, gyro={calibration[1]}, '
            f'accel={calibration[2]}, mag={calibration[3]}).'
        )
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
