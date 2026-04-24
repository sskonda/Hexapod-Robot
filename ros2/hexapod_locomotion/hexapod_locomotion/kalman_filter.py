import math

from .yaw_control import clamp, normalize_angle


class AngleKalmanFilter:
    """
    Two-state Kalman filter for tilt angle estimation from IMU data.

    State vector: [angle (deg), gyro_bias (deg/s)]

    The gyroscope rate is used in the prediction step (fast, low noise in the short
    term) and the accelerometer-derived tilt angle is used in the update step (good
    long-term reference, but noisy under vibration). This combination suppresses
    high-frequency servo vibration while still tracking slow postural changes.
    """

    def __init__(self, q_angle=0.001, q_bias=0.003, r_measure=0.03):
        self.q_angle = q_angle
        self.q_bias = q_bias
        self.r_measure = r_measure

        self.angle = 0.0
        self.bias = 0.0
        self.P = [[0.0, 0.0], [0.0, 0.0]]

    def update(self, accel_angle, gyro_rate, dt):
        rate = gyro_rate - self.bias
        self.angle += dt * rate

        self.P[0][0] += dt * (dt * self.P[1][1] - self.P[0][1] - self.P[1][0] + self.q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.q_bias * dt

        innovation = accel_angle - self.angle
        innovation_covariance = self.P[0][0] + self.r_measure

        kalman_gain_0 = self.P[0][0] / innovation_covariance
        kalman_gain_1 = self.P[1][0] / innovation_covariance

        self.angle += kalman_gain_0 * innovation
        self.bias += kalman_gain_1 * innovation

        covariance_00 = self.P[0][0]
        covariance_01 = self.P[0][1]
        self.P[0][0] -= kalman_gain_0 * covariance_00
        self.P[0][1] -= kalman_gain_0 * covariance_01
        self.P[1][0] -= kalman_gain_1 * covariance_00
        self.P[1][1] -= kalman_gain_1 * covariance_01

        return self.angle


class YawComplementaryFilter:
    """
    Complementary yaw filter with a slow gyro-bias observer.

    The gyroscope provides short-term yaw dynamics, while trusted absolute yaw
    corrections slowly remove drift. The internal bias estimate prevents the
    filter from integrating a persistent gyro offset indefinitely.
    """

    def __init__(
        self,
        time_constant_sec=0.5,
        bias_gain=0.05,
        max_bias_rad_s=0.25,
    ):
        self.time_constant_sec = max(1e-3, float(time_constant_sec))
        self.bias_gain = max(0.0, float(bias_gain))
        self.max_bias_rad_s = max(0.0, float(max_bias_rad_s))
        self.yaw_rad = 0.0
        self.gyro_bias_rad_s = 0.0
        self.last_innovation_rad = 0.0
        self.initialized = False

    def reset(self, yaw_rad=None, gyro_bias_rad_s=0.0):
        self.gyro_bias_rad_s = clamp(
            float(gyro_bias_rad_s),
            -self.max_bias_rad_s,
            self.max_bias_rad_s,
        )
        self.last_innovation_rad = 0.0

        if yaw_rad is None:
            self.yaw_rad = 0.0
            self.initialized = False
            return

        self.yaw_rad = normalize_angle(float(yaw_rad))
        self.initialized = True

    def peek_prediction(self, gyro_yaw_rate_rad_s, dt, fallback_yaw_rad=None):
        dt = max(0.0, float(dt))

        if not self.initialized:
            return normalize_angle(0.0 if fallback_yaw_rad is None else fallback_yaw_rad)

        corrected_rate_rad_s = float(gyro_yaw_rate_rad_s) - self.gyro_bias_rad_s
        return normalize_angle(self.yaw_rad + corrected_rate_rad_s * dt)

    def update(self, measured_yaw_rad, gyro_yaw_rate_rad_s, dt):
        measured_yaw_rad = normalize_angle(float(measured_yaw_rad))
        dt = max(0.0, float(dt))

        if not self.initialized:
            self.yaw_rad = measured_yaw_rad
            self.last_innovation_rad = 0.0
            self.initialized = True
            return self.yaw_rad

        predicted_yaw_rad = self.peek_prediction(gyro_yaw_rate_rad_s, dt)
        correction_gain = dt / (self.time_constant_sec + dt)
        innovation = normalize_angle(measured_yaw_rad - predicted_yaw_rad)
        self.last_innovation_rad = innovation

        self.yaw_rad = normalize_angle(predicted_yaw_rad + correction_gain * innovation)
        self.gyro_bias_rad_s = clamp(
            self.gyro_bias_rad_s - self.bias_gain * innovation * dt,
            -self.max_bias_rad_s,
            self.max_bias_rad_s,
        )
        return self.yaw_rad

    def predict(self, gyro_yaw_rate_rad_s, dt, fallback_yaw_rad=None):
        dt = max(0.0, float(dt))
        predicted_yaw_rad = self.peek_prediction(
            gyro_yaw_rate_rad_s,
            dt,
            fallback_yaw_rad=fallback_yaw_rad,
        )

        self.yaw_rad = predicted_yaw_rad
        self.last_innovation_rad = 0.0
        self.initialized = True
        return self.yaw_rad
