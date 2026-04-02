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
    [140.0, 0.0, 0.0],
    [140.0, 0.0, 0.0],
    [140.0, 0.0, 0.0],
    [140.0, 0.0, 0.0],
    [140.0, 0.0, 0.0],
    [140.0, 0.0, 0.0],
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
