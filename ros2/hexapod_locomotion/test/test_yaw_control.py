import math

import pytest

from hexapod_locomotion.yaw_control import (
    StartupStillnessGate,
    apply_angular_deadband,
    compute_heading_hold_pid,
    fused_yaw_is_trusted,
    fused_yaw_untrusted_reason,
    imu_is_still,
    relative_yaw_from_reference,
    resolve_parameter_value,
    update_startup_yaw_reference,
    update_vector_running_average,
    vector_components_within_tolerance,
    yaw_is_trusted_from_covariance,
)


def test_apply_angular_deadband_returns_zero_inside_deadband():
    error_rad = math.radians(3.0)
    deadband_rad = math.radians(5.0)

    assert apply_angular_deadband(error_rad, deadband_rad) == pytest.approx(0.0)
    assert apply_angular_deadband(-error_rad, deadband_rad) == pytest.approx(0.0)


def test_apply_angular_deadband_preserves_continuity_outside_deadband():
    adjusted = apply_angular_deadband(
        math.radians(12.0),
        math.radians(5.0),
    )

    assert adjusted == pytest.approx(math.radians(7.0))


def test_imu_is_still_checks_accel_and_gyro_thresholds():
    assert imu_is_still((0.0, 0.0, 9.81), (0.01, 0.0, 0.0))
    assert not imu_is_still((0.0, 0.0, 11.0), (0.01, 0.0, 0.0))
    assert not imu_is_still((0.0, 0.0, 9.81), (0.25, 0.0, 0.0))


def test_startup_stillness_gate_tolerates_short_motion_spikes():
    gate = StartupStillnessGate(required_still_time_sec=1.0, motion_grace_sec=0.2)

    assert not gate.update(0.4, (0.0, 0.0, 9.81), (0.0, 0.0, 0.0))
    assert gate.remaining_time_sec == pytest.approx(0.6)

    assert not gate.update(0.1, (0.0, 0.0, 9.81), (0.3, 0.0, 0.0))
    assert gate.remaining_time_sec == pytest.approx(0.6)

    assert not gate.update(0.1, (0.0, 0.0, 9.81), (0.3, 0.0, 0.0))
    assert gate.remaining_time_sec == pytest.approx(1.0)

    assert not gate.update(0.6, (0.0, 0.0, 9.81), (0.0, 0.0, 0.0))
    assert gate.remaining_time_sec == pytest.approx(0.4)

    assert gate.update(0.4, (0.0, 0.0, 9.81), (0.0, 0.0, 0.0))
    assert gate.ready


def test_resolve_parameter_value_prefers_new_name_and_reports_conflict():
    resolved, conflict = resolve_parameter_value(0.8, 0.5, 0.5)

    assert resolved == pytest.approx(0.8)
    assert not conflict

    resolved, conflict = resolve_parameter_value(0.8, 0.6, 0.5)

    assert resolved == pytest.approx(0.8)
    assert conflict


def test_resolve_parameter_value_respects_legacy_default_value():
    resolved, conflict = resolve_parameter_value(
        0.45,
        0.0,
        0.45,
        legacy_default_value=0.0,
    )

    assert resolved == pytest.approx(0.45)
    assert not conflict

    resolved, conflict = resolve_parameter_value(
        0.45,
        0.2,
        0.45,
        legacy_default_value=0.0,
    )

    assert resolved == pytest.approx(0.2)
    assert not conflict


def test_update_startup_yaw_reference_initializes_and_tracks_while_still():
    reference = update_startup_yaw_reference(None, math.radians(15.0), False, True)
    assert reference == pytest.approx(math.radians(15.0))

    reference = update_startup_yaw_reference(reference, math.radians(20.0), False, True)
    assert reference == pytest.approx(math.radians(20.0))


def test_update_startup_yaw_reference_holds_value_once_robot_is_moving():
    reference = update_startup_yaw_reference(math.radians(12.0), math.radians(30.0), False, False)
    assert reference == pytest.approx(math.radians(12.0))

    locked_reference = update_startup_yaw_reference(
        math.radians(12.0),
        math.radians(30.0),
        True,
        True,
    )
    assert locked_reference == pytest.approx(math.radians(12.0))


def test_relative_yaw_from_reference_wraps_to_shortest_difference():
    relative_yaw = relative_yaw_from_reference(
        math.radians(-170.0),
        math.radians(170.0),
    )

    assert relative_yaw == pytest.approx(math.radians(20.0))


def test_update_vector_running_average_tracks_per_axis_mean():
    average, count = update_vector_running_average(None, (1.0, 2.0, 3.0), 0)
    assert average == pytest.approx((1.0, 2.0, 3.0))
    assert count == 1

    average, count = update_vector_running_average(average, (3.0, 4.0, 5.0), count)
    assert average == pytest.approx((2.0, 3.0, 4.0))
    assert count == 2


def test_vector_components_within_tolerance_requires_every_axis_to_match():
    assert vector_components_within_tolerance(
        (10.0, -5.0, 2.5),
        (11.9, -3.1, 0.6),
        2.0,
    )
    assert not vector_components_within_tolerance(
        (10.0, -5.0, 2.5),
        (12.1, -5.0, 2.5),
        2.0,
    )


def test_fused_yaw_trust_requires_sys_only_until_reference_locks():
    calibration = (0, 3, 2, 2)

    assert not fused_yaw_is_trusted(
        calibration,
        reference_locked=False,
        min_sys=1,
        min_gyro=3,
        min_accel=2,
        min_mag=2,
    )
    assert fused_yaw_is_trusted(
        calibration,
        reference_locked=True,
        min_sys=1,
        min_gyro=3,
        min_accel=2,
        min_mag=2,
    )


def test_fused_yaw_trust_still_requires_mag_after_reference_locks():
    calibration = (0, 3, 3, 1)

    assert not fused_yaw_is_trusted(
        calibration,
        reference_locked=True,
        min_sys=1,
        min_gyro=3,
        min_accel=2,
        min_mag=2,
    )
    assert fused_yaw_untrusted_reason(
        calibration,
        reference_locked=True,
        min_sys=1,
        min_gyro=3,
        min_accel=2,
        min_mag=2,
    ) == 'mag_calibration_low'


def test_yaw_covariance_gate_accepts_only_low_uncertainty_heading():
    trusted_covariance = [0.05] * 9
    trusted_covariance[8] = 0.05
    untrusted_covariance = [0.05] * 9
    untrusted_covariance[8] = 25.0

    assert yaw_is_trusted_from_covariance(trusted_covariance, 1.0)
    assert not yaw_is_trusted_from_covariance(untrusted_covariance, 1.0)


def test_heading_hold_pid_positive_yaw_error_commands_positive_correction():
    step = compute_heading_hold_pid(
        target_yaw_rad=math.radians(15.0),
        current_yaw_rad=0.0,
        current_yaw_rate_rps=0.0,
        integral_state_rad=0.0,
        control_dt_sec=0.02,
        kp=0.5,
        ki=0.0,
        kd=0.0,
        deadband_rad=0.0,
        integral_limit=1.0,
        correction_limit=1.0,
    )

    assert step.yaw_error_rad > 0.0
    assert step.proportional_term > 0.0
    assert step.correction > 0.0


def test_heading_hold_pid_derivative_opposes_measured_positive_yaw_rate():
    step = compute_heading_hold_pid(
        target_yaw_rad=0.0,
        current_yaw_rad=0.0,
        current_yaw_rate_rps=0.5,
        integral_state_rad=0.0,
        control_dt_sec=0.02,
        kp=0.0,
        ki=0.0,
        kd=0.2,
        deadband_rad=0.0,
        integral_limit=1.0,
        correction_limit=1.0,
    )

    assert step.derivative_term == pytest.approx(-0.1)
    assert step.correction < 0.0
