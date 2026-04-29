#!/usr/bin/env python3

"""Pure gait helpers that do not depend on ROS."""

TRIPOD_GROUP_PHASE_OFFSETS = (0.5, 0.0, 0.5, 0.0, 0.5, 0.0)


def clamp_unit_interval(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def triangular_lift_scale(progress: float) -> float:
    """Return a 0..1..0 lift profile over a swing phase."""
    progress = clamp_unit_interval(progress)
    if progress <= 0.5:
        return progress * 2.0
    return (1.0 - progress) * 2.0


def tripod_leg_pose(
    baseline_point,
    cycle_planar_travel_mm,
    step_height_mm: float,
    phase_progress: float,
    phase_offset: float,
):
    """Compute a closed tripod-gait foot pose from the cycle phase.

    ``phase_progress`` is expected in the range ``[0, 1]``.  During the first
    half of the local phase the leg swings from ``-0.5 * travel`` to
    ``+0.5 * travel`` while lifted. During the second half it returns on the
    ground from ``+0.5 * travel`` to ``-0.5 * travel``.  This keeps every leg
    on the ground at the cycle boundary and removes the end-of-cycle reset.
    """

    local_phase = (float(phase_progress) + float(phase_offset)) % 1.0
    if local_phase < 0.5:
        swing_progress = local_phase / 0.5
        planar_scale = swing_progress - 0.5
        lift_mm = float(step_height_mm) * triangular_lift_scale(swing_progress)
    else:
        stance_progress = (local_phase - 0.5) / 0.5
        planar_scale = 0.5 - stance_progress
        lift_mm = 0.0

    return [
        float(baseline_point[0]) + float(cycle_planar_travel_mm[0]) * planar_scale,
        float(baseline_point[1]) + float(cycle_planar_travel_mm[1]) * planar_scale,
        float(baseline_point[2]) + lift_mm,
    ]


def tripod_points_for_phase(
    baseline_points,
    cycle_planar_travel_mm,
    step_height_mm: float,
    phase_progress: float,
):
    return [
        tripod_leg_pose(
            baseline_point,
            cycle_planar_travel_mm[leg_index],
            step_height_mm,
            phase_progress,
            TRIPOD_GROUP_PHASE_OFFSETS[leg_index],
        )
        for leg_index, baseline_point in enumerate(baseline_points)
    ]
