DEFAULT_Q_ANGLE = 0.001
DEFAULT_Q_BIAS = 0.003
DEFAULT_R_MEASURE = 0.03


class AngleKalmanFilter:
    """
    Two-state Kalman filter for tilt angle estimation from IMU data.

    State vector: [angle (deg), gyro_bias (deg/s)]

    The gyroscope rate is used in the prediction step (fast, low noise in the short
    term) and the accelerometer-derived tilt angle is used in the update step (good
    long-term reference, but noisy under vibration).  This combination suppresses
    high-frequency servo vibration while still tracking slow postural changes.

    Tuning parameters (all in consistent degree/second units):
        q_angle   -- process noise for the angle state.  Increase if the robot
                     changes orientation faster than the gyro can track.
        q_bias    -- process noise for the bias state.  Increase if the gyro
                     drift changes rapidly (e.g. temperature swings).
        r_measure -- measurement noise for the accelerometer angle.  Increase
                     to trust the accelerometer less (smoother but laggier);
                     decrease to follow accelerometer readings more closely.
    """

    def __init__(
        self,
        q_angle=DEFAULT_Q_ANGLE,
        q_bias=DEFAULT_Q_BIAS,
        r_measure=DEFAULT_R_MEASURE,
        initial_angle=0.0,
        initial_bias=0.0,
        fixed_kalman_gain=None,
    ):
        self.q_angle = float(q_angle)
        self.q_bias = float(q_bias)
        self.r_measure = float(r_measure)

        self.angle = float(initial_angle)  # estimated tilt angle (degrees)
        self.bias = float(initial_bias)    # estimated gyro bias (degrees/s)
        self.fixed_kalman_gain = None
        self.last_gain = (0.0, 0.0)
        if fixed_kalman_gain is not None:
            self.set_fixed_kalman_gain(*fixed_kalman_gain)

        # 2x2 error covariance matrix: [[P00, P01], [P10, P11]]
        self.P = [[0.0, 0.0], [0.0, 0.0]]

    def set_fixed_kalman_gain(self, angle_gain, bias_gain):
        self.fixed_kalman_gain = (float(angle_gain), float(bias_gain))

    def clear_fixed_kalman_gain(self):
        self.fixed_kalman_gain = None

    def update(self, accel_angle, gyro_rate, dt):
        """
        Run one filter step and return the updated angle estimate.

        Args:
            accel_angle: tilt angle derived from accelerometer (degrees)
            gyro_rate:   angular rate from gyroscope for this axis (degrees/s)
            dt:          elapsed time since the last call (seconds)

        Returns:
            Filtered angle estimate (degrees)
        """
        # --- Predict ---
        # Integrate gyro rate (corrected for estimated bias) to get angle prediction.
        rate = gyro_rate - self.bias
        self.angle += dt * rate

        # Propagate error covariance: P = F*P*F^T + Q
        # F = [[1, -dt], [0, 1]],  Q = diag(q_angle*dt, q_bias*dt)
        self.P[0][0] += dt * (dt * self.P[1][1] - self.P[0][1] - self.P[1][0] + self.q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.q_bias * dt

        # --- Update ---
        y = accel_angle - self.angle  # innovation (measurement residual)
        if self.fixed_kalman_gain is None:
            S = self.P[0][0] + self.r_measure  # innovation covariance

            # Kalman gain: K = P * H^T / S,  H = [1, 0]
            K0 = self.P[0][0] / S
            K1 = self.P[1][0] / S

            # Correct state estimates.
            self.angle += K0 * y
            self.bias += K1 * y

            # Update error covariance: P = (I - K*H) * P
            P00_tmp = self.P[0][0]
            P01_tmp = self.P[0][1]
            self.P[0][0] -= K0 * P00_tmp
            self.P[0][1] -= K0 * P01_tmp
            self.P[1][0] -= K1 * P00_tmp
            self.P[1][1] -= K1 * P01_tmp
        else:
            K0, K1 = self.fixed_kalman_gain
            self.angle += K0 * y
            self.bias += K1 * y

        self.last_gain = (K0, K1)

        return self.angle
