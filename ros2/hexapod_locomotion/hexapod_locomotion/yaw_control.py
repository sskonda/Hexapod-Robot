#!/usr/bin/env python3

"""Shared helpers for yaw control, IMU frame alignment, and startup settling."""

import math
from dataclasses import dataclass


STANDARD_GRAVITY_M_S2 = 9.80665


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def apply_angular_deadband(error_rad: float, deadband_rad: float) -> float:
    """Return zero inside the deadband and a continuous error outside it."""
    error_rad = float(error_rad)
    deadband_rad = max(0.0, float(deadband_rad))

    if abs(error_rad) <= deadband_rad:
        return 0.0

    return math.copysign(abs(error_rad) - deadband_rad, error_rad)


def heading_from_vector(
    x_value: float,
    y_value: float,
    fallback_yaw_rad: float | None = None,
    min_norm: float = 1e-9,
) -> float | None:
    """Return a yaw angle for a 2D heading vector, or a fallback if too small."""
    x_value = float(x_value)
    y_value = float(y_value)
    if math.hypot(x_value, y_value) < max(0.0, float(min_norm)):
        return fallback_yaw_rad
    return math.atan2(y_value, x_value)


def vector_norm(values) -> float:
    return math.sqrt(sum(float(value) * float(value) for value in values))


def quaternion_from_euler(roll_rad: float, pitch_rad: float, yaw_rad: float):
    half_roll = float(roll_rad) * 0.5
    half_pitch = float(pitch_rad) * 0.5
    half_yaw = float(yaw_rad) * 0.5

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


def quaternion_conjugate(quaternion):
    x_value, y_value, z_value, w_value = quaternion
    return (-float(x_value), -float(y_value), -float(z_value), float(w_value))


def quaternion_normalize(quaternion):
    x_value, y_value, z_value, w_value = quaternion
    norm = math.sqrt(
        x_value * x_value
        + y_value * y_value
        + z_value * z_value
        + w_value * w_value
    )
    if norm < 1e-12:
        return 0.0, 0.0, 0.0, 1.0

    return (
        float(x_value) / norm,
        float(y_value) / norm,
        float(z_value) / norm,
        float(w_value) / norm,
    )


def quaternion_multiply(first, second):
    ax, ay, az, aw = first
    bx, by, bz, bw = second
    return quaternion_normalize((
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ))


def rotate_vector_by_quaternion(vector, quaternion):
    x_value, y_value, z_value, w_value = quaternion_normalize(quaternion)
    vx, vy, vz = vector

    # Quaternion sandwich product optimized for pure-vector input.
    tx = 2.0 * (y_value * vz - z_value * vy)
    ty = 2.0 * (z_value * vx - x_value * vz)
    tz = 2.0 * (x_value * vy - y_value * vx)

    return (
        float(vx) + w_value * tx + (y_value * tz - z_value * ty),
        float(vy) + w_value * ty + (z_value * tx - x_value * tz),
        float(vz) + w_value * tz + (x_value * ty - y_value * tx),
    )


def quaternion_to_yaw(x_value: float, y_value: float, z_value: float, w_value: float) -> float:
    siny_cosp = 2.0 * (w_value * z_value + x_value * y_value)
    cosy_cosp = 1.0 - 2.0 * (y_value * y_value + z_value * z_value)
    return math.atan2(siny_cosp, cosy_cosp)


def orientation_quaternion_is_available(msg) -> bool:
    covariance = getattr(msg, 'orientation_covariance', None)
    if covariance is None or len(covariance) < 1 or covariance[0] < 0.0:
        return False

    return any(abs(value) > 1e-6 for value in (
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w,
    ))


def orientation_yaw_variance(orientation_covariance) -> float:
    if orientation_covariance is None or len(orientation_covariance) < 9:
        return math.inf

    yaw_variance = float(orientation_covariance[8])
    if yaw_variance < 0.0:
        return math.inf

    return yaw_variance


def heading_is_trusted(
    orientation_covariance,
    max_yaw_variance_rad2: float = 1.0,
) -> bool:
    return orientation_yaw_variance(orientation_covariance) <= max(
        0.0,
        float(max_yaw_variance_rad2),
    )


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
    legacy_default_value: float | None = None,
) -> tuple[float, bool]:
    """Resolve a preferred parameter alias against a legacy name.

    Returns ``(resolved_value, conflict)`` where conflict is true when both
    names were overridden with different values.
    """
    preferred_value = float(preferred_value)
    legacy_value = float(legacy_value)
    default_value = float(default_value)
    if legacy_default_value is None:
        legacy_default_value = default_value
    legacy_default_value = float(legacy_default_value)

    preferred_overridden = not math.isclose(
        preferred_value,
        default_value,
        rel_tol=0.0,
        abs_tol=tolerance,
    )
    legacy_overridden = not math.isclose(
        legacy_value,
        legacy_default_value,
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


@dataclass(frozen=True)
class YawHoldOutput:
    yaw_rate_rad_s: float
    target_yaw_rad: float | None
    yaw_error_rad: float
    deadbanded_error_rad: float
    proportional_term_rad_s: float
    integral_term_rad_s: float
    active: bool
    saturated: bool
    reason: str


class YawHoldController:
    """PI yaw hold with angular deadband, saturation, and anti-windup."""

    def __init__(
        self,
        kp: float,
        ki: float = 0.0,
        deadband_rad: float = 0.0,
        integrator_limit: float = 0.0,
        correction_limit_rad_s: float = math.inf,
    ):
        self.kp = max(0.0, float(kp))
        self.ki = max(0.0, float(ki))
        self.deadband_rad = max(0.0, float(deadband_rad))
        self.integrator_limit = max(0.0, float(integrator_limit))
        self.correction_limit_rad_s = max(0.0, float(correction_limit_rad_s))
        self.target_yaw_rad: float | None = None
        self.integral_state = 0.0

    @property
    def enabled(self) -> bool:
        return self.kp > 0.0 or self.ki > 0.0

    def configure(
        self,
        kp: float | None = None,
        ki: float | None = None,
        deadband_rad: float | None = None,
        integrator_limit: float | None = None,
        correction_limit_rad_s: float | None = None,
    ) -> None:
        if kp is not None:
            self.kp = max(0.0, float(kp))
        if ki is not None:
            self.ki = max(0.0, float(ki))
        if deadband_rad is not None:
            self.deadband_rad = max(0.0, float(deadband_rad))
        if integrator_limit is not None:
            self.integrator_limit = max(0.0, float(integrator_limit))
            self.integral_state = clamp(
                self.integral_state,
                -self.integrator_limit,
                self.integrator_limit,
            )
        if correction_limit_rad_s is not None:
            self.correction_limit_rad_s = max(0.0, float(correction_limit_rad_s))

    def reset(self, target_yaw_rad: float | None = None) -> None:
        self.target_yaw_rad = (
            None if target_yaw_rad is None else normalize_angle(float(target_yaw_rad))
        )
        self.integral_state = 0.0

    def output_for_reason(self, reason: str) -> YawHoldOutput:
        return YawHoldOutput(
            yaw_rate_rad_s=0.0,
            target_yaw_rad=self.target_yaw_rad,
            yaw_error_rad=0.0,
            deadbanded_error_rad=0.0,
            proportional_term_rad_s=0.0,
            integral_term_rad_s=0.0,
            active=False,
            saturated=False,
            reason=reason,
        )

    def update(
        self,
        current_yaw_rad: float,
        dt_sec: float,
        target_yaw_rad: float | None = None,
        correction_limit_rad_s: float | None = None,
        latch_target_if_unset: bool = True,
        reset_integral_on_target_change: bool = True,
    ) -> YawHoldOutput:
        if not self.enabled:
            self.reset()
            return self.output_for_reason('disabled')

        if target_yaw_rad is not None:
            normalized_target = normalize_angle(float(target_yaw_rad))
            target_changed = (
                self.target_yaw_rad is None
                or abs(normalize_angle(normalized_target - self.target_yaw_rad)) > 1e-9
            )
            self.target_yaw_rad = normalized_target
            if target_changed and reset_integral_on_target_change:
                self.integral_state = 0.0
        elif self.target_yaw_rad is None:
            if not latch_target_if_unset:
                return self.output_for_reason('no_target')
            self.target_yaw_rad = normalize_angle(float(current_yaw_rad))
            self.integral_state = 0.0

        current_yaw_rad = normalize_angle(float(current_yaw_rad))
        dt_sec = max(0.0, float(dt_sec))
        correction_limit = self.correction_limit_rad_s
        if correction_limit_rad_s is not None:
            correction_limit = max(0.0, float(correction_limit_rad_s))

        yaw_error = normalize_angle(self.target_yaw_rad - current_yaw_rad)
        deadbanded_error = apply_angular_deadband(yaw_error, self.deadband_rad)
        proportional_term = self.kp * deadbanded_error
        integral_candidate = clamp(
            self.integral_state + deadbanded_error * dt_sec,
            -self.integrator_limit,
            self.integrator_limit,
        )
        unsaturated_output = proportional_term + self.ki * integral_candidate
        saturated_output = clamp(
            unsaturated_output,
            -correction_limit,
            correction_limit,
        )
        saturated = not math.isclose(
            unsaturated_output,
            saturated_output,
            rel_tol=0.0,
            abs_tol=1e-9,
        )

        if self.ki > 0.0:
            if (
                not saturated
                or (unsaturated_output > correction_limit and deadbanded_error < 0.0)
                or (unsaturated_output < -correction_limit and deadbanded_error > 0.0)
            ):
                self.integral_state = integral_candidate

        integral_term = self.ki * self.integral_state
        yaw_rate = clamp(
            proportional_term + integral_term,
            -correction_limit,
            correction_limit,
        )

        return YawHoldOutput(
            yaw_rate_rad_s=yaw_rate,
            target_yaw_rad=self.target_yaw_rad,
            yaw_error_rad=yaw_error,
            deadbanded_error_rad=deadbanded_error,
            proportional_term_rad_s=proportional_term,
            integral_term_rad_s=integral_term,
            active=True,
            saturated=saturated,
            reason='tracking',
        )


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
