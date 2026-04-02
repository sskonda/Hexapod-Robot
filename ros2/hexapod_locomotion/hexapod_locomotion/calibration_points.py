#!/usr/bin/env python3

import argparse
from pathlib import Path

try:
    from .calibration_store import (
        JOINT_NAMES,
        neutral_ik_angles,
        offsets_from_leg_coordinates,
        save_offsets,
        servo_angles_from_leg_coordinates,
    )
except ImportError:
    from calibration_store import (
        JOINT_NAMES,
        neutral_ik_angles,
        offsets_from_leg_coordinates,
        save_offsets,
        servo_angles_from_leg_coordinates,
    )


def load_points_file(points_file):
    rows = []
    with Path(points_file).open('r', encoding='utf-8') as handle:
        for line_number, raw_line in enumerate(handle, start=1):
            line = raw_line.strip()
            if not line:
                continue
            parts = line.replace(',', ' ').split()
            if len(parts) != 3:
                raise ValueError(
                    f'Line {line_number} in {points_file} must contain exactly 3 values'
                )
            rows.append(tuple(float(value) for value in parts))

    if len(rows) != 6:
        raise ValueError(f'{points_file} must contain exactly 6 coordinate rows')

    return rows


def main():
    parser = argparse.ArgumentParser(
        description='Convert per-leg XYZ calibration coordinates into servo offset YAML.'
    )
    parser.add_argument(
        'points_file',
        help='Path to a 6-line file of leg coordinates in the same format as Sample_Code/Server/point.txt',
    )
    parser.add_argument(
        '--output',
        help='Optional YAML file to write. If omitted, the offsets are printed.',
    )
    args = parser.parse_args()

    leg_coordinates = load_points_file(args.points_file)
    offsets = offsets_from_leg_coordinates(leg_coordinates)

    neutral = neutral_ik_angles()
    print(
        'Neutral IK angles from control.py [140, 0, 0]: '
        f"coxa={neutral['coxa']:.0f}, femur={neutral['femur']:.0f}, tibia={neutral['tibia']:.0f}"
    )
    print('Servo-space neutral angles:')
    print('  legs 1-3: coxa=90, femur=157, tibia=116')
    print('  legs 4-6: coxa=90, femur=23, tibia=64')
    print('')
    print('Per-leg servo targets from the supplied coordinates:')
    for leg_index, coordinates in enumerate(leg_coordinates, start=1):
        angles = servo_angles_from_leg_coordinates(leg_index - 1, coordinates)
        print(
            f'  leg{leg_index}: '
            f"x={coordinates[0]:.1f} y={coordinates[1]:.1f} z={coordinates[2]:.1f} -> "
            f"coxa={angles['coxa']:.1f} femur={angles['femur']:.1f} tibia={angles['tibia']:.1f}"
        )

    print('')
    print('servo_offsets:')
    for joint in JOINT_NAMES:
        print(f'  {joint}: {offsets[joint]:.1f}')

    if args.output:
        save_offsets(args.output, offsets)
        print(f'\nSaved YAML to {args.output}')


if __name__ == '__main__':
    main()
