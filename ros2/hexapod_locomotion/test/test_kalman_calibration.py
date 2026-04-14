import pytest

from hexapod_locomotion.kalman_calibration_store import (
    default_kalman_calibration,
    load_kalman_calibration,
    save_kalman_calibration,
)
from hexapod_locomotion.kalman_filter import AngleKalmanFilter


def test_fixed_gain_updates_state():
    kalman_filter = AngleKalmanFilter(fixed_kalman_gain=(0.25, 0.1))

    angle = kalman_filter.update(accel_angle=10.0, gyro_rate=0.0, dt=0.1)

    assert angle == pytest.approx(2.5)
    assert kalman_filter.bias == pytest.approx(1.0)
    assert kalman_filter.last_gain == pytest.approx((0.25, 0.1))


def test_kalman_calibration_round_trip(tmp_path):
    calibration = default_kalman_calibration()
    calibration['calibrated'] = True
    calibration['sample_count'] = 123
    calibration['roll']['use_fixed_gain'] = True
    calibration['roll']['kalman_gain_angle'] = 0.42
    calibration['roll']['kalman_gain_bias'] = 0.08
    calibration['pitch']['initial_bias_deg_s'] = -0.17

    calibration_path = tmp_path / 'imu_kalman_calibration.yaml'
    save_kalman_calibration(str(calibration_path), calibration)
    loaded = load_kalman_calibration(str(calibration_path))

    assert loaded['calibrated'] is True
    assert loaded['sample_count'] == 123
    assert loaded['roll']['use_fixed_gain'] is True
    assert loaded['roll']['kalman_gain_angle'] == pytest.approx(0.42)
    assert loaded['roll']['kalman_gain_bias'] == pytest.approx(0.08)
    assert loaded['pitch']['initial_bias_deg_s'] == pytest.approx(-0.17)
