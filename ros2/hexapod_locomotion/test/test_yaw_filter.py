import math

import pytest

from hexapod_locomotion.kalman_filter import YawComplementaryFilter, normalize_angle


def test_yaw_filter_initializes_from_measurement():
    yaw_filter = YawComplementaryFilter(time_constant_sec=0.5)

    estimate = yaw_filter.update(1.2, 0.0, 0.02)

    assert estimate == pytest.approx(1.2)


def test_yaw_filter_tracks_gyro_between_mag_corrections():
    yaw_filter = YawComplementaryFilter(time_constant_sec=10.0)
    yaw_filter.update(0.0, 0.0, 0.02)

    estimate = yaw_filter.update(0.0, 1.0, 0.1)

    assert estimate == pytest.approx(0.09900990099)


def test_yaw_filter_wraps_cleanly_across_pi_boundary():
    yaw_filter = YawComplementaryFilter(time_constant_sec=0.1)
    yaw_filter.update(math.pi - 0.05, 0.0, 0.02)

    estimate = yaw_filter.update(-math.pi + 0.05, 0.0, 0.1)

    assert abs(normalize_angle(estimate - math.pi)) < 0.06
