#!/usr/bin/env python3

"""Shared helpers for yaw control and IMU startup settling."""

import math
from dataclasses import dataclass


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


def quaternion_to_yaw(x_value, y_value, z_value, w_value):
    siny_cosp = 2.0 * (float(w_value) * float(z_value) + float(x_value) * float(y_value))
    cosy_cosp = 1.0 - 2.0 * (
        float(y_value) * float(y_value) + float(z_value) * float(z_value)
    )
    return math.atan2(siny_cosp, cosy_cosp)


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


def calibration_status_meets_thresholds(
    calibration,
    min_sys: int = 0,
    min_gyro: int = 0,
    min_accel: int = 0,
    min_mag: int = 0,
) -> bool:
    if calibration is None or len(calibration) != 4:
        return False

    sys_calib, gyro_calib, accel_calib, mag_calib = (
        int(value) for value in calibration
    )
    return (
        sys_calib >= int(min_sys)
        and gyro_calib >= int(min_gyro)
        and accel_calib >= int(min_accel)
        and mag_calib >= int(min_mag)
    )


def fused_yaw_untrusted_reason(
    calibration,
    reference_locked: bool,
    min_sys: int,
    min_gyro: int,
    min_accel: int,
    min_mag: int,
    ignore_sys_after_lock: bool = True,
) -> str:
    if calibration is None or len(calibration) != 4:
        return 'calibration_missing'

    sys_calib, gyro_calib, accel_calib, mag_calib = (
        int(value) for value in calibration
    )
    if gyro_calib < int(min_gyro):
        return 'gyro_calibration_low'
    if accel_calib < int(min_accel):
        return 'accel_calibration_low'
    if mag_calib < int(min_mag):
        return 'mag_calibration_low'
    if (not bool(reference_locked) or not bool(ignore_sys_after_lock)) and sys_calib < int(min_sys):
        return 'sys_calibration_low'
    return 'calibration_healthy'


def fused_yaw_is_trusted(
    calibration,
    reference_locked: bool,
    min_sys: int,
    min_gyro: int,
    min_accel: int,
    min_mag: int,
    ignore_sys_after_lock: bool = True,
) -> bool:
    return fused_yaw_untrusted_reason(
        calibration,
        reference_locked=reference_locked,
        min_sys=min_sys,
        min_gyro=min_gyro,
        min_accel=min_accel,
        min_mag=min_mag,
        ignore_sys_after_lock=ignore_sys_after_lock,
    ) == 'calibration_healthy'


def vector_norm(values) -> float:
    return math.sqrt(sum(float(value) * float(value) for value in values))


def yaw_is_trusted_from_covariance(
    orientation_covariance,
    max_trusted_yaw_covariance_rad2: float,
) -> bool:
    if orientation_covariance is None or len(orientation_covariance) < 9:
        return False

    yaw_covariance = float(orientation_covariance[8])
    if not math.isfinite(yaw_covariance):
        return False

    return yaw_covariance >= 0.0 and yaw_covariance <= max(
        0.0,
        float(max_trusted_yaw_covariance_rad2),
    )


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


@dataclass(frozen=True)
class HeadingHoldPidStep:
    yaw_error_rad: float
    deadbanded_yaw_error_rad: float
    proportional_term: float
    integral_candidate: float
    integral_term: float
    derivative_term: float
    unsaturated_correction: float
    correction: float
    correction_limit: float
    correction_is_saturated: bool
    integral_should_update: bool


def compute_heading_hold_pid(
    target_yaw_rad: float,
    current_yaw_rad: float,
    current_yaw_rate_rps: float,
    integral_state_rad: float,
    control_dt_sec: float,
    kp: float,
    ki: float,
    kd: float,
    deadband_rad: float,
    integral_limit: float,
    correction_limit: float,
) -> HeadingHoldPidStep:
    yaw_error_rad = normalize_angle(float(target_yaw_rad) - float(current_yaw_rad))
    deadbanded_yaw_error_rad = apply_angular_deadband(yaw_error_rad, deadband_rad)
    proportional_term = float(kp) * deadbanded_yaw_error_rad
    derivative_term = -float(kd) * float(current_yaw_rate_rps)
    integral_candidate = max(
        -float(integral_limit),
        min(
            float(integral_limit),
            float(integral_state_rad) + deadbanded_yaw_error_rad * max(0.0, float(control_dt_sec)),
        ),
    )
    integral_term = float(ki) * integral_candidate
    unsaturated_correction = proportional_term + integral_term + derivative_term
    correction = max(
        -float(correction_limit),
        min(float(correction_limit), unsaturated_correction),
    )
    correction_is_saturated = not math.isclose(
        unsaturated_correction,
        correction,
        rel_tol=0.0,
        abs_tol=1e-9,
    )
    integral_should_update = (
        float(ki) > 0.0
        and (
            not correction_is_saturated
            or (
                unsaturated_correction > float(correction_limit)
                and deadbanded_yaw_error_rad < 0.0
            )
            or (
                unsaturated_correction < -float(correction_limit)
                and deadbanded_yaw_error_rad > 0.0
            )
        )
    )
    return HeadingHoldPidStep(
        yaw_error_rad=yaw_error_rad,
        deadbanded_yaw_error_rad=deadbanded_yaw_error_rad,
        proportional_term=proportional_term,
        integral_candidate=integral_candidate,
        integral_term=integral_term,
        derivative_term=derivative_term,
        unsaturated_correction=unsaturated_correction,
        correction=correction,
        correction_limit=float(correction_limit),
        correction_is_saturated=correction_is_saturated,
        integral_should_update=integral_should_update,
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
