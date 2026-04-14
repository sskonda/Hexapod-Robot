#!/usr/bin/env python3

"""BNO055-based IMU publisher for the hexapod robot.

This node mirrors the current MPU6050 publisher's ROS interface:
  - publishes ``sensor_msgs/msg/Imu`` on ``/imu/data_raw`` by default
  - fills raw accelerometer data in m/s^2
  - fills raw gyroscope data in rad/s
  - leaves orientation unavailable by default to match the existing stack

The default BNO055 mode is ``IMUPLUS_MODE`` so the sensor uses accel + gyro
fusion without relying on the magnetometer for absolute yaw. That matches the
current locomotion stack's main need: a cleaner gyro Z signal for yaw-rate
correction.
"""

import traceback

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

try:
    import board
    import adafruit_bno055
except ImportError:  # pragma: no cover - exercised only on systems without the dependency.
    board = None
    adafruit_bno055 = None


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


class BNO055Publisher(Node):
    def __init__(self):
        super().__init__('bno055_publisher')

        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('topic', '/imu/data_raw')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('i2c_address', 0x28)
        self.declare_parameter('mode', 'IMUPLUS_MODE')
        self.declare_parameter('use_external_crystal', True)
        self.declare_parameter('angular_velocity_covariance', 0.02)
        self.declare_parameter('linear_acceleration_covariance', 0.04)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.topic = str(self.get_parameter('topic').value)
        self.publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.i2c_address = int(self.get_parameter('i2c_address').value)
        self.mode_name = str(self.get_parameter('mode').value).strip()
        self.use_external_crystal = bool(self.get_parameter('use_external_crystal').value)
        self.angular_velocity_covariance = float(
            self.get_parameter('angular_velocity_covariance').value
        )
        self.linear_acceleration_covariance = float(
            self.get_parameter('linear_acceleration_covariance').value
        )
        self.last_warning_text = ''

        if board is None or adafruit_bno055 is None:
            raise RuntimeError(
                'Missing BNO055 Python dependencies. Install adafruit-blinka and '
                'adafruit-circuitpython-bno055 on the robot.'
            )

        self.sensor = self._create_sensor()
        self.publisher_ = self.create_publisher(Imu, self.topic, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_imu)

        calibration = self.sensor.calibration_status
        self.get_logger().info(
            f'BNO055 publisher started on {self.topic} with frame {self.frame_id}, '
            f'address 0x{self.i2c_address:02X}, mode {self.mode_name}'
        )
        self.get_logger().info(
            'Initial calibration status '
            f'(sys={calibration[0]}, gyro={calibration[1]}, accel={calibration[2]}, mag={calibration[3]})'
        )

    def _create_sensor(self):
        try:
            i2c = board.I2C()
            sensor = adafruit_bno055.BNO055_I2C(i2c, address=self.i2c_address)
            mode_value = getattr(adafruit_bno055, self.mode_name, None)
            if mode_value is None:
                raise ValueError(
                    f'Unsupported BNO055 mode "{self.mode_name}". '
                    'Use a valid adafruit_bno055 mode constant such as IMUPLUS_MODE or NDOF_MODE.'
                )
            sensor.mode = mode_value
            sensor.use_external_crystal = self.use_external_crystal
            return sensor
        except Exception as exc:
            raise RuntimeError(f'Unable to initialize BNO055 on I2C address 0x{self.i2c_address:02X}: {exc}') from exc

    def publish_imu(self):
        try:
            accel = self.sensor.acceleration
            gyro = self.sensor.gyro
        except Exception as exc:
            self._warn_throttled(f'BNO055 read failed: {exc}')
            return

        if not vector_is_valid(accel, 3) or not vector_is_valid(gyro, 3):
            self._warn_throttled(
                'BNO055 acceleration or gyro data is unavailable. '
                f'Check the mode ({self.mode_name}) and sensor health.'
            )
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.linear_acceleration.x = float(accel[0])
        msg.linear_acceleration.y = float(accel[1])
        msg.linear_acceleration.z = float(accel[2])

        # Adafruit's BNO055 driver already reports gyro values in rad/s.
        msg.angular_velocity.x = float(gyro[0])
        msg.angular_velocity.y = float(gyro[1])
        msg.angular_velocity.z = float(gyro[2])

        # Match the current MPU6050 publisher: publish accel + gyro only and
        # leave orientation unavailable to downstream consumers by default.
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
