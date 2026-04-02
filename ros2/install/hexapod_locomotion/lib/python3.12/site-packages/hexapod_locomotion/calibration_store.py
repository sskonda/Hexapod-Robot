from pathlib import Path
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
