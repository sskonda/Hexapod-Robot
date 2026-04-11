#!/usr/bin/env python3

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

try:
    from .calibration_store import (
        JOINT_NAMES,
        offsets_from_leg_coordinates,
        save_base_footpoints,
        save_offsets,
        servo_angles_from_leg_coordinates,
    )
except ImportError:
    from calibration_store import (
        JOINT_NAMES,
        offsets_from_leg_coordinates,
        save_base_footpoints,
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
OUTPUT_YAML = (
    Path(get_package_share_directory('hexapod_locomotion'))
    / 'config'
    / 'servo_calibration.yaml'
)

LEG_MOUNT_ANGLES_DEG = [54.0, 0.0, -54.0, -126.0, 180.0, 126.0]
LEG_X_OFFSETS_MM = [94.0, 85.0, 94.0, 94.0, 85.0, 94.0]
LEG_Z_OFFSET_MM = 50.0


def body_footpoints_from_leg_coordinates(leg_coordinates):
    base_footpoints = []
    for leg_index, point in enumerate(leg_coordinates):
        leg_x = float(point[0])
        leg_y = float(point[1])
        leg_z = float(point[2])
        angle = math.radians(LEG_MOUNT_ANGLES_DEG[leg_index])
        qx = leg_x + LEG_X_OFFSETS_MM[leg_index]

        body_x = qx * math.cos(angle) - leg_y * math.sin(angle)
        body_y = qx * math.sin(angle) + leg_y * math.cos(angle)
        body_z = leg_z + LEG_Z_OFFSET_MM

        base_footpoints.append([body_x, body_y, body_z])

    return base_footpoints


def main():
    offsets = offsets_from_leg_coordinates(LEG_COORDINATE_MATRIX)
    base_footpoints = body_footpoints_from_leg_coordinates(LEG_COORDINATE_MATRIX)

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

    print('\nbase_footpoints:')
    for leg_index, point in enumerate(base_footpoints, start=1):
        print(
            f'  leg{leg_index}: '
            f'x={point[0]:.1f} y={point[1]:.1f} z={point[2]:.1f}'
        )

    save_offsets(str(OUTPUT_YAML), offsets)
    save_base_footpoints(str(OUTPUT_YAML), base_footpoints)
    print(f'\nSaved YAML to {OUTPUT_YAML}')


if __name__ == '__main__':
    main()
