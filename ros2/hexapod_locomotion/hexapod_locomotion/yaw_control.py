#!/usr/bin/env python3

"""Shared helpers for yaw control and IMU startup settling."""

import math


STANDARD_GRAVITY_M_S2 = 9.80665


def normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(float(angle_rad)), math.cos(float(angle_rad)))


def quaternion_from_euler(roll_rad, pitch_rad, yaw_rad):
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


def quaternion_normalize(quaternion):
    norm = vector_norm(quaternion)
    if norm < 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return tuple(float(value) / norm for value in quaternion)


def apply_angular_deadband(error_rad: float, deadband_rad: float) -> float:
    """Return zero inside the deadband and a continuous error outside it."""
    error_rad = float(error_rad)
    deadband_rad = max(0.0, float(deadband_rad))

    if abs(error_rad) <= deadband_rad:
        return 0.0

    return math.copysign(abs(error_rad) - deadband_rad, error_rad)


def update_startup_yaw_reference(
    reference_yaw_rad,
    current_yaw_rad,
    reference_locked,
    imu_is_still,
):
    current_yaw_rad = normalize_angle(current_yaw_rad)
    if reference_yaw_rad is None or (not reference_locked and imu_is_still):
        return current_yaw_rad

    return normalize_angle(reference_yaw_rad)


def relative_yaw_from_reference(yaw_rad, reference_yaw_rad):
    if reference_yaw_rad is None:
        return 0.0

    return normalize_angle(float(yaw_rad) - float(reference_yaw_rad))


def vector_norm(values) -> float:
    return math.sqrt(sum(float(value) * float(value) for value in values))


def update_vector_running_average(current_average, sample_values, sample_count):
    """Update a per-axis running average and return ``(average, count)``."""
    if sample_values is None:
        return current_average, int(sample_count)

    if current_average is None or int(sample_count) <= 0:
        return tuple(float(value) for value in sample_values), 1

    next_count = int(sample_count) + 1
    next_average = tuple(
        current_average[index]
        + (float(sample_values[index]) - current_average[index]) / next_count
        for index in range(len(sample_values))
    )
    return next_average, next_count


def vector_components_within_tolerance(reference_values, sample_values, tolerance):
    """Return true when every axis stays within the absolute tolerance."""
    if (
        reference_values is None
        or sample_values is None
        or len(reference_values) != len(sample_values)
    ):
        return False

    tolerance = max(0.0, float(tolerance))
    return all(
        abs(float(sample_values[index]) - float(reference_values[index])) <= tolerance
        for index in range(len(reference_values))
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
