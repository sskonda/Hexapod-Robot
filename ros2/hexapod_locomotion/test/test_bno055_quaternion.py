import math

from hexapod_locomotion.bno055_publisher import (
    mode_uses_sensor_fused_orientation,
    quaternion_to_yaw,
    quaternion_xyzw_from_bno_raw,
)
import pytest


def test_quaternion_xyzw_from_bno_raw_reorders_and_normalizes():
    raw_wxyz = (
        int(round(math.cos(math.pi / 4.0) * (1 << 14))),
        0,
        0,
        int(round(math.sin(math.pi / 4.0) * (1 << 14))),
    )

    quaternion = quaternion_xyzw_from_bno_raw(raw_wxyz)

    assert quaternion[0] == pytest.approx(0.0, abs=1e-6)
    assert quaternion[1] == pytest.approx(0.0, abs=1e-6)
    assert quaternion[2] == pytest.approx(math.sin(math.pi / 4.0), rel=1e-4)
    assert quaternion[3] == pytest.approx(math.cos(math.pi / 4.0), rel=1e-4)


def test_quaternion_xyzw_from_bno_raw_rejects_zero_norm():
    assert quaternion_xyzw_from_bno_raw((0, 0, 0, 0)) is None


def test_quaternion_to_yaw_matches_quarter_turn():
    yaw_rad = quaternion_to_yaw(
        0.0,
        0.0,
        math.sin(math.pi / 4.0),
        math.cos(math.pi / 4.0),
    )

    assert yaw_rad == pytest.approx(math.pi / 2.0, rel=1e-6)


def test_mode_uses_sensor_fused_orientation_accepts_ndof_modes():
    assert mode_uses_sensor_fused_orientation('NDOF_MODE')
    assert mode_uses_sensor_fused_orientation('NDOF_FMC_OFF_MODE')
    assert not mode_uses_sensor_fused_orientation('AMG_MODE')
