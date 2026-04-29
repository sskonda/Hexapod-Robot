import math

import pytest

from hexapod_locomotion.imu_fallback import (
    integrate_relative_yaw,
    should_use_mpu6050_yaw_fallback,
)


def test_integrate_relative_yaw_accumulates_and_wraps():
    yaw_rad = integrate_relative_yaw(math.pi - 0.1, 1.0, 0.2)
    assert yaw_rad == pytest.approx(-math.pi + 0.1)


def test_should_use_mpu6050_yaw_fallback_waits_for_timeout_and_sensor():
    assert not should_use_mpu6050_yaw_fallback(
        primary_yaw_valid=False,
        fallback_enabled=True,
        fallback_available=False,
        elapsed_sec=10.0,
        timeout_sec=5.0,
    )
    assert not should_use_mpu6050_yaw_fallback(
        primary_yaw_valid=False,
        fallback_enabled=True,
        fallback_available=True,
        elapsed_sec=4.9,
        timeout_sec=5.0,
    )
    assert should_use_mpu6050_yaw_fallback(
        primary_yaw_valid=False,
        fallback_enabled=True,
        fallback_available=True,
        elapsed_sec=5.0,
        timeout_sec=5.0,
    )


def test_should_use_mpu6050_yaw_fallback_stays_latched_once_selected():
    assert should_use_mpu6050_yaw_fallback(
        primary_yaw_valid=True,
        fallback_enabled=True,
        fallback_available=True,
        elapsed_sec=1.0,
        timeout_sec=5.0,
        fallback_latched=True,
    )
