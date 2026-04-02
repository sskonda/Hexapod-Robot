#!/usr/bin/env python3

from pathlib import Path

try:
    from .calibration_store import (
        JOINT_NAMES,
        offsets_from_leg_coordinates,
        save_offsets,
        servo_angles_from_leg_coordinates,
    )
except ImportError:
    from calibration_store import (
        JOINT_NAMES,
        offsets_from_leg_coordinates,
        save_offsets,
        servo_angles_from_leg_coordinates,
    )

# Replace these six [x, y, z] rows with values from your calibration graph.
LEG_COORDINATE_MATRIX = [
    [99.0, 29.0, -17.0],
    [102.0, -28.0, -22.0],
    [113.0, 2.0, -7.0],
    [124.0, -15.0, -18.0],
    [115.0, 8.0, -7.0],
    [135.0, -25.0, 0.0],
]

# Change this if you want to write somewhere else.
OUTPUT_YAML = Path('ros2/hexapod_locomotion/config/servo_calibration.yaml')


def main():
    offsets = offsets_from_leg_coordinates(LEG_COORDINATE_MATRIX)

    print('Per-leg servo angles:')
    for leg_index, coordinates in enumerate(LEG_COORDINATE_MATRIX, start=1):
        angles = servo_angles_from_leg_coordinates(leg_index - 1, coordinates)
        print(
            f'  leg{leg_index}: '
            f'x={coordinates[0]:.1f} y={coordinates[1]:.1f} z={coordinates[2]:.1f} -> '
            f"coxa={angles['coxa']:.1f} femur={angles['femur']:.1f} tibia={angles['tibia']:.1f}"
        )

    print('\nservo_offsets:')
    for joint in JOINT_NAMES:
        print(f'  {joint}: {offsets[joint]:.1f}')

    save_offsets(str(OUTPUT_YAML), offsets)
    print(f'\nSaved YAML to {OUTPUT_YAML}')


if __name__ == '__main__':
    main()
