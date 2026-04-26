import math

import pytest

from hexapod_locomotion.yaw_control import (
    StartupStillnessGate,
    YawHoldController,
    apply_angular_deadband,
    heading_from_vector,
    imu_is_still,
    resolve_parameter_value,
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


def test_heading_from_vector_returns_yaw_or_fallback():
    assert heading_from_vector(0.0, 1.0) == pytest.approx(math.pi / 2.0)
    assert heading_from_vector(0.0, 0.0, fallback_yaw_rad=1.2) == pytest.approx(1.2)
    assert heading_from_vector(0.0, 0.0) is None


def test_yaw_hold_controller_latches_current_heading_and_corrects_drift():
    controller = YawHoldController(
        kp=1.0,
        ki=0.0,
        deadband_rad=0.0,
        integrator_limit=0.0,
        correction_limit_rad_s=0.5,
    )

    first = controller.update(current_yaw_rad=0.25, dt_sec=0.1)
    assert first.target_yaw_rad == pytest.approx(0.25)
    assert first.yaw_rate_rad_s == pytest.approx(0.0)

    correction = controller.update(current_yaw_rad=0.35, dt_sec=0.1)
    assert correction.yaw_error_rad == pytest.approx(-0.1)
    assert correction.yaw_rate_rad_s == pytest.approx(-0.1)


def test_yaw_hold_controller_tracks_explicit_path_heading_with_saturation():
    controller = YawHoldController(
        kp=2.0,
        ki=0.0,
        deadband_rad=0.0,
        integrator_limit=0.0,
        correction_limit_rad_s=0.2,
    )

    correction = controller.update(
        current_yaw_rad=0.0,
        dt_sec=0.1,
        target_yaw_rad=math.pi / 2.0,
    )

    assert correction.target_yaw_rad == pytest.approx(math.pi / 2.0)
    assert correction.yaw_rate_rad_s == pytest.approx(0.2)
    assert correction.saturated


def test_yaw_hold_controller_integral_anti_windup_allows_unwinding():
    controller = YawHoldController(
        kp=0.0,
        ki=1.0,
        deadband_rad=0.0,
        integrator_limit=10.0,
        correction_limit_rad_s=0.1,
    )

    controller.reset(target_yaw_rad=1.0)
    controller.update(current_yaw_rad=0.0, dt_sec=1.0)
    assert controller.integral_state == pytest.approx(0.0)

    controller.integral_state = 1.0
    controller.update(current_yaw_rad=1.2, dt_sec=1.0)
    assert controller.integral_state < 1.0
