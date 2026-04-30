#!/usr/bin/env python3

"""Shared helpers for yaw control and IMU startup settling."""

import math


STANDARD_GRAVITY_M_S2 = 9.80665


def apply_angular_deadband(error_rad: float, deadband_rad: float) -> float:
    """Return zero inside the deadband and a continuous error outside it."""
    error_rad = float(error_rad)
    deadband_rad = max(0.0, float(deadband_rad))

    if abs(error_rad) <= deadband_rad:
        return 0.0

    return math.copysign(abs(error_rad) - deadband_rad, error_rad)


def normalize_angle(angle_rad: float) -> float:
    """Wrap an angle to the [-pi, pi] interval."""
    return math.atan2(math.sin(float(angle_rad)), math.cos(float(angle_rad)))


def vector_norm(values) -> float:
    return math.sqrt(sum(float(value) * float(value) for value in values))


def imu_is_still(
    accel_m_s2,
    gyro_rad_s,
    accel_tolerance_m_s2: float = 0.75,
    gyro_tolerance_rad_s: float = 0.12,
    gravity_m_s2: float = STANDARD_GRAVITY_M_S2,
) -> bool:
    """Check whether the IMU is approximately motionless."""
    if (
        accel_m_s2 is None
        or gyro_rad_s is None
        or len(accel_m_s2) != 3
        or len(gyro_rad_s) != 3
    ):
        return False

    accel_error_m_s2 = abs(vector_norm(accel_m_s2) - float(gravity_m_s2))
    gyro_norm_rad_s = vector_norm(gyro_rad_s)
    return (
        accel_error_m_s2 <= max(0.0, float(accel_tolerance_m_s2))
        and gyro_norm_rad_s <= max(0.0, float(gyro_tolerance_rad_s))
    )


def resolve_parameter_value(
    preferred_value: float,
    legacy_value: float,
    default_value: float,
    tolerance: float = 1e-9,
) -> tuple[float, bool]:
    """Resolve a preferred parameter alias against a legacy name.

    Returns ``(resolved_value, conflict)`` where conflict is true when both
    names were overridden with different values.
    """
    preferred_value = float(preferred_value)
    legacy_value = float(legacy_value)
    default_value = float(default_value)

    preferred_overridden = not math.isclose(
        preferred_value,
        default_value,
        rel_tol=0.0,
        abs_tol=tolerance,
    )
    legacy_overridden = not math.isclose(
        legacy_value,
        default_value,
        rel_tol=0.0,
        abs_tol=tolerance,
    )

    conflict = (
        preferred_overridden
        and legacy_overridden
        and not math.isclose(preferred_value, legacy_value, rel_tol=0.0, abs_tol=tolerance)
    )

    if preferred_overridden:
        return preferred_value, conflict

    if legacy_overridden:
        return legacy_value, False

    return preferred_value, False


class StartupStillnessGate:
    """Require a continuous still period before enabling yaw heading hold."""

    def __init__(
        self,
        required_still_time_sec: float,
        accel_tolerance_m_s2: float = 0.75,
        gyro_tolerance_rad_s: float = 0.12,
        motion_grace_sec: float = 0.5,
        gravity_m_s2: float = STANDARD_GRAVITY_M_S2,
    ):
        self.required_still_time_sec = max(0.0, float(required_still_time_sec))
        self.accel_tolerance_m_s2 = max(0.0, float(accel_tolerance_m_s2))
        self.gyro_tolerance_rad_s = max(0.0, float(gyro_tolerance_rad_s))
        self.motion_grace_sec = max(0.0, float(motion_grace_sec))
        self.gravity_m_s2 = float(gravity_m_s2)
        self.accumulated_still_time_sec = 0.0
        self.accumulated_motion_time_sec = 0.0
        self.is_still = False
        self.ready = self.required_still_time_sec <= 0.0

        if self.ready:
            self.accumulated_still_time_sec = self.required_still_time_sec
            self.is_still = True

    @property
    def remaining_time_sec(self) -> float:
        return max(0.0, self.required_still_time_sec - self.accumulated_still_time_sec)

    def update(self, dt: float, accel_m_s2, gyro_rad_s) -> bool:
        if self.ready:
            self.is_still = True
            return True

        self.is_still = imu_is_still(
            accel_m_s2,
            gyro_rad_s,
            accel_tolerance_m_s2=self.accel_tolerance_m_s2,
            gyro_tolerance_rad_s=self.gyro_tolerance_rad_s,
            gravity_m_s2=self.gravity_m_s2,
        )

        if self.is_still:
            self.accumulated_motion_time_sec = 0.0
            self.accumulated_still_time_sec = min(
                self.required_still_time_sec,
                self.accumulated_still_time_sec + max(0.0, float(dt)),
            )
            self.ready = self.accumulated_still_time_sec >= self.required_still_time_sec
        else:
            self.accumulated_motion_time_sec += max(0.0, float(dt))
            if self.accumulated_motion_time_sec >= self.motion_grace_sec:
                self.accumulated_still_time_sec = 0.0

        return self.ready


class HeadingHoldPid:
    """Yaw-rate PID for holding a target heading while translating."""

    def __init__(
        self,
        kp: float,
        ki: float = 0.0,
        kd: float = 0.0,
        integral_limit_rad_s: float = 0.0,
    ):
        self.kp = max(0.0, float(kp))
        self.ki = max(0.0, float(ki))
        self.kd = max(0.0, float(kd))
        self.integral_limit_rad_s = max(0.0, float(integral_limit_rad_s))
        self.integral_correction_rad_s = 0.0

    @property
    def enabled(self) -> bool:
        return (
            self.kp > 1e-9
            or self.ki > 1e-9
            or self.kd > 1e-9
        )

    def set_gains(self, kp: float, ki: float, kd: float):
        self.kp = max(0.0, float(kp))
        self.ki = max(0.0, float(ki))
        self.kd = max(0.0, float(kd))

    def set_integral_limit(self, integral_limit_rad_s: float):
        self.integral_limit_rad_s = max(0.0, float(integral_limit_rad_s))
        if self.integral_limit_rad_s <= 0.0:
            self.integral_correction_rad_s = 0.0
        else:
            self.integral_correction_rad_s = max(
                -self.integral_limit_rad_s,
                min(self.integral_limit_rad_s, self.integral_correction_rad_s),
            )

    def reset(self):
        self.integral_correction_rad_s = 0.0

    def update(
        self,
        target_yaw_rad: float,
        current_yaw_rad: float,
        current_yaw_rate_rad_s: float,
        dt: float,
        deadband_rad: float = 0.0,
        output_limit_rad_s: float | None = None,
    ) -> float:
        if not self.enabled:
            return 0.0

        error_rad = normalize_angle(float(target_yaw_rad) - float(current_yaw_rad))
        error_rad = apply_angular_deadband(error_rad, deadband_rad)
        proportional = self.kp * error_rad
        derivative = -self.kd * float(current_yaw_rate_rad_s)

        previous_integral_correction = self.integral_correction_rad_s
        dt = max(0.0, float(dt))
        if dt > 0.0 and self.ki > 0.0 and abs(error_rad) > 1e-6:
            self.integral_correction_rad_s += self.ki * error_rad * dt
            if self.integral_limit_rad_s > 0.0:
                self.integral_correction_rad_s = max(
                    -self.integral_limit_rad_s,
                    min(self.integral_limit_rad_s, self.integral_correction_rad_s),
                )
        elif abs(error_rad) <= 1e-6:
            self.integral_correction_rad_s *= max(0.0, 1.0 - min(1.0, dt))

        correction_rad_s = proportional + self.integral_correction_rad_s + derivative
        if output_limit_rad_s is None or output_limit_rad_s <= 0.0:
            return correction_rad_s

        output_limit_rad_s = float(output_limit_rad_s)
        clamped_correction_rad_s = max(
            -output_limit_rad_s,
            min(output_limit_rad_s, correction_rad_s),
        )
        if abs(clamped_correction_rad_s - correction_rad_s) > 1e-9:
            if (
                clamped_correction_rad_s > 0.0 and error_rad > 0.0
            ) or (
                clamped_correction_rad_s < 0.0 and error_rad < 0.0
            ):
                self.integral_correction_rad_s = previous_integral_correction
                correction_rad_s = proportional + self.integral_correction_rad_s + derivative
                clamped_correction_rad_s = max(
                    -output_limit_rad_s,
                    min(output_limit_rad_s, correction_rad_s),
                )

        return clamped_correction_rad_s
