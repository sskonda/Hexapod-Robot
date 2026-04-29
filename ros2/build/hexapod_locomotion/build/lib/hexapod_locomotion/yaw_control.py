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
