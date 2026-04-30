import math

import pytest

from hexapod_locomotion.yaw_control import (
    HeadingHoldPid,
    StartupStillnessGate,
    apply_angular_deadband,
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


def test_heading_hold_pid_combines_pid_terms():
    controller = HeadingHoldPid(
        kp=1.0,
        ki=0.5,
        kd=0.25,
        integral_limit_rad_s=1.0,
    )

    correction = controller.update(
        target_yaw_rad=1.0,
        current_yaw_rad=0.0,
        current_yaw_rate_rad_s=0.4,
        dt=0.2,
    )

    assert correction == pytest.approx(1.0)


def test_heading_hold_pid_prevents_integral_windup_at_output_limit():
    controller = HeadingHoldPid(
        kp=2.0,
        ki=1.0,
        kd=0.0,
        integral_limit_rad_s=0.5,
    )

    correction = controller.update(
        target_yaw_rad=1.0,
        current_yaw_rad=0.0,
        current_yaw_rate_rad_s=0.0,
        dt=1.0,
        output_limit_rad_s=0.2,
    )

    assert correction == pytest.approx(0.2)
    assert controller.integral_correction_rad_s == pytest.approx(0.0)


def test_heading_hold_pid_reset_clears_integral_state():
    controller = HeadingHoldPid(
        kp=0.0,
        ki=1.0,
        kd=0.0,
        integral_limit_rad_s=0.5,
    )
    controller.update(
        target_yaw_rad=0.5,
        current_yaw_rad=0.0,
        current_yaw_rate_rad_s=0.0,
        dt=1.0,
    )

    controller.reset()

    assert controller.integral_correction_rad_s == pytest.approx(0.0)
