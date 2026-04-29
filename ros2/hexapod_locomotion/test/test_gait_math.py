import pytest

from hexapod_locomotion.gait_math import (
    DEFAULT_TRIPOD_PLANAR_TRAVEL_SCALE,
    cycle_planar_travel_from_deltas,
    tripod_points_for_phase,
)


BASELINE_POINTS = [
    [137.1, 189.4, -25.0],
    [225.0, 0.0, -25.0],
    [137.1, -189.4, -25.0],
    [-137.1, -189.4, -25.0],
    [-225.0, 0.0, -25.0],
    [-137.1, 189.4, -25.0],
]


def uniform_planar_travel(dx_mm: float, dy_mm: float):
    return [[dx_mm, dy_mm] for _ in range(6)]


def test_tripod_cycle_wraps_without_leaving_legs_lifted():
    points_at_wrap = tripod_points_for_phase(
        BASELINE_POINTS,
        uniform_planar_travel(0.0, -22.0),
        40.0,
        1.0,
    )

    for baseline_point, wrapped_point in zip(BASELINE_POINTS, points_at_wrap):
        assert wrapped_point[2] == pytest.approx(baseline_point[2])


def test_tripod_cycle_boundary_is_periodic():
    start_points = tripod_points_for_phase(
        BASELINE_POINTS,
        uniform_planar_travel(26.4, 0.0),
        40.0,
        0.0,
    )
    wrap_points = tripod_points_for_phase(
        BASELINE_POINTS,
        uniform_planar_travel(26.4, 0.0),
        40.0,
        1.0,
    )

    for start_point, wrap_point in zip(start_points, wrap_points):
        assert wrap_point == pytest.approx(start_point)


def test_tripod_lifted_group_switches_between_half_cycles():
    early_points = tripod_points_for_phase(
        BASELINE_POINTS,
        uniform_planar_travel(0.0, -22.0),
        40.0,
        0.25,
    )
    late_points = tripod_points_for_phase(
        BASELINE_POINTS,
        uniform_planar_travel(0.0, -22.0),
        40.0,
        0.75,
    )

    for leg_index, baseline_point in enumerate(BASELINE_POINTS):
        if leg_index % 2 == 1:
            assert early_points[leg_index][2] > baseline_point[2]
            assert late_points[leg_index][2] == pytest.approx(baseline_point[2])
        else:
            assert early_points[leg_index][2] == pytest.approx(baseline_point[2])
            assert late_points[leg_index][2] > baseline_point[2]


def test_tripod_cycle_travel_scale_matches_legacy_excursion():
    planar_deltas = [[0.875, 0.0] for _ in range(6)]
    scaled_travel = cycle_planar_travel_from_deltas(
        planar_deltas,
        total_steps=40,
        travel_scale=DEFAULT_TRIPOD_PLANAR_TRAVEL_SCALE,
    )

    for travel in scaled_travel:
        assert travel[0] == pytest.approx(70.0)
        assert travel[1] == pytest.approx(0.0)
