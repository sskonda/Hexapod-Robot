from pathlib import Path
import math
import yaml

JOINT_NAMES = [
    'leg1_coxa', 'leg1_femur', 'leg1_tibia',
    'leg2_coxa', 'leg2_femur', 'leg2_tibia',
    'leg3_coxa', 'leg3_femur', 'leg3_tibia',
    'leg4_coxa', 'leg4_femur', 'leg4_tibia',
    'leg5_coxa', 'leg5_femur', 'leg5_tibia',
    'leg6_coxa', 'leg6_femur', 'leg6_tibia',
]

JOINT_CHANNELS = {
    'leg1_coxa': 15,
    'leg1_femur': 14,
    'leg1_tibia': 13,
    'leg2_coxa': 12,
    'leg2_femur': 11,
    'leg2_tibia': 10,
    'leg3_coxa': 9,
    'leg3_femur': 8,
    'leg3_tibia': 31,
    'leg4_coxa': 22,
    'leg4_femur': 23,
    'leg4_tibia': 27,
    'leg5_coxa': 19,
    'leg5_femur': 20,
    'leg5_tibia': 21,
    'leg6_coxa': 16,
    'leg6_femur': 17,
    'leg6_tibia': 18,
}

LOCOMOTION_BASE_FOOTPOINTS = [
    [137.1, 189.4, 0.0],
    [225.0, 0.0, 0.0],
    [137.1, -189.4, 0.0],
    [-137.1, -189.4, 0.0],
    [-225.0, 0.0, 0.0],
    [-137.1, 189.4, 0.0],
]

LOCOMOTION_LEG_MOUNT_ANGLES_DEG = [54.0, 0.0, -54.0, -126.0, 180.0, 126.0]
LOCOMOTION_LEG_X_OFFSETS_MM = [94.0, 85.0, 94.0, 94.0, 85.0, 94.0]
LOCOMOTION_LEG_Z_OFFSET_MM = 50.0
LOCOMOTION_DEFAULT_BODY_HEIGHT_MM = -25.0

REFERENCE_ANGLES = {
    name: 90.0 for name in JOINT_NAMES
}
REFERENCE_ANGLES.update({
    'leg1_tibia': 10.0,
    'leg2_tibia': 10.0,
    'leg3_tibia': 10.0,
    'leg4_tibia': 170.0,
    'leg5_tibia': 170.0,
    'leg6_tibia': 170.0,
})


def zero_offsets():
    return {name: 0.0 for name in JOINT_NAMES}


def reference_angles():
    return {name: REFERENCE_ANGLES[name] for name in JOINT_NAMES}


def commanded_angles(offsets: dict):
    angles = {}
    for name in JOINT_NAMES:
        offset = float(offsets.get(name, 0.0))
        angles[name] = min(180.0, max(0.0, REFERENCE_ANGLES[name] + offset))
    return angles


def clamp_angle(angle):
    return min(180.0, max(0.0, float(angle)))


def restrict_value(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def coordinate_to_angle(x, y, z, l1=33.0, l2=90.0, l3=110.0):
    a = math.pi / 2 - math.atan2(z, y)
    x_4 = l1 * math.sin(a)
    x_5 = l1 * math.cos(a)
    l23 = math.sqrt((z - x_5) ** 2 + (y - x_4) ** 2 + x ** 2)
    w = restrict_value(x / l23, -1.0, 1.0)
    v = restrict_value((l2 * l2 + l23 * l23 - l3 * l3) / (2 * l2 * l23), -1.0, 1.0)
    u = restrict_value((l2 ** 2 + l3 ** 2 - l23 ** 2) / (2 * l3 * l2), -1.0, 1.0)
    b = math.asin(round(w, 2)) - math.acos(round(v, 2))
    c = math.pi - math.acos(round(u, 2))
    return (
        round(math.degrees(a)),
        round(math.degrees(b)),
        round(math.degrees(c)),
    )


def neutral_ik_angles():
    a, b, c = coordinate_to_angle(0.0, 140.0, 0.0)
    return {'coxa': float(a), 'femur': float(b), 'tibia': float(c)}


def servo_angles_from_leg_coordinates(leg_index: int, coordinates):
    x, y, z = coordinates
    coxa, femur, tibia = coordinate_to_angle(-float(z), float(x), float(y))

    if leg_index < 3:
        servo_femur = 90.0 - femur
        servo_tibia = float(tibia)
    else:
        servo_femur = 90.0 + femur
        servo_tibia = 180.0 - tibia

    return {
        'coxa': clamp_angle(coxa),
        'femur': clamp_angle(servo_femur),
        'tibia': clamp_angle(servo_tibia),
    }


def offsets_from_leg_coordinates(leg_coordinates):
    if len(leg_coordinates) != 6:
        raise ValueError('Expected coordinates for exactly 6 legs')

    offsets = zero_offsets()
    for leg_index, coordinates in enumerate(leg_coordinates, start=1):
        servo_angles = servo_angles_from_leg_coordinates(leg_index - 1, coordinates)
        offsets[f'leg{leg_index}_coxa'] = servo_angles['coxa'] - REFERENCE_ANGLES[f'leg{leg_index}_coxa']
        offsets[f'leg{leg_index}_femur'] = servo_angles['femur'] - REFERENCE_ANGLES[f'leg{leg_index}_femur']
        offsets[f'leg{leg_index}_tibia'] = servo_angles['tibia'] - REFERENCE_ANGLES[f'leg{leg_index}_tibia']

    return offsets


def locomotion_default_leg_coordinates(
    default_body_height_mm=LOCOMOTION_DEFAULT_BODY_HEIGHT_MM,
):
    leg_coordinates = []
    for leg_index, point in enumerate(LOCOMOTION_BASE_FOOTPOINTS):
        angle = math.radians(LOCOMOTION_LEG_MOUNT_ANGLES_DEG[leg_index])
        leg_coordinates.append([
            point[0] * math.cos(angle) + point[1] * math.sin(angle) - LOCOMOTION_LEG_X_OFFSETS_MM[leg_index],
            -point[0] * math.sin(angle) + point[1] * math.cos(angle),
            point[2] + float(default_body_height_mm) - LOCOMOTION_LEG_Z_OFFSET_MM,
        ])

    return leg_coordinates


def locomotion_default_offsets(default_body_height_mm=LOCOMOTION_DEFAULT_BODY_HEIGHT_MM):
    return offsets_from_leg_coordinates(
        locomotion_default_leg_coordinates(default_body_height_mm=default_body_height_mm)
    )


def normalize_offsets_for_locomotion(
    offsets: dict,
    default_body_height_mm=LOCOMOTION_DEFAULT_BODY_HEIGHT_MM,
):
    baseline_offsets = locomotion_default_offsets(default_body_height_mm=default_body_height_mm)
    raw_magnitude = sum(abs(float(offsets.get(name, 0.0))) for name in JOINT_NAMES)
    normalized = {
        name: float(offsets.get(name, 0.0)) - baseline_offsets[name]
        for name in JOINT_NAMES
    }
    normalized_magnitude = sum(abs(normalized[name]) for name in JOINT_NAMES)
    should_normalize = raw_magnitude > 1e-6 and normalized_magnitude + 1e-6 < raw_magnitude
    return normalized if should_normalize else {
        name: float(offsets.get(name, 0.0))
        for name in JOINT_NAMES
    }


def load_offsets(yaml_path: str):
    path = Path(yaml_path)
    if not path.exists():
        return zero_offsets()

    with path.open('r') as f:
        data = yaml.safe_load(f) or {}

    raw = data.get('servo_offsets', {})
    offsets = zero_offsets()
    for name in JOINT_NAMES:
        if name in raw:
            offsets[name] = float(raw[name])

    return offsets


def save_offsets(yaml_path: str, offsets: dict):
    path = Path(yaml_path)
    path.parent.mkdir(parents=True, exist_ok=True)

    cleaned = zero_offsets()
    for name in JOINT_NAMES:
        if name in offsets:
            cleaned[name] = float(offsets[name])

    with path.open('w') as f:
        yaml.safe_dump({'servo_offsets': cleaned}, f, sort_keys=False)
