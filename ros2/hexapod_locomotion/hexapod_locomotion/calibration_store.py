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


def zero_offsets():
    return {name: 0.0 for name in JOINT_NAMES}


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
