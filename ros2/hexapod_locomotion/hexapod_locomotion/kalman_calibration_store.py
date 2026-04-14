from copy import deepcopy
from pathlib import Path

import yaml

from .kalman_filter import DEFAULT_Q_ANGLE, DEFAULT_Q_BIAS, DEFAULT_R_MEASURE


AXES = ('roll', 'pitch')


def default_axis_calibration():
    return {
        'q_angle': DEFAULT_Q_ANGLE,
        'q_bias': DEFAULT_Q_BIAS,
        'r_measure': DEFAULT_R_MEASURE,
        'initial_bias_deg_s': 0.0,
        'use_fixed_gain': False,
        'kalman_gain_angle': 0.0,
        'kalman_gain_bias': 0.0,
    }


def default_kalman_calibration():
    return {
        'calibrated': False,
        'generated_at': '',
        'warmup_duration_sec': 0.0,
        'accumulation_duration_sec': 0.0,
        'sample_count': 0,
        'roll': default_axis_calibration(),
        'pitch': default_axis_calibration(),
    }


def _coerce_float(value, default):
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def load_kalman_calibration(yaml_path: str):
    calibration = default_kalman_calibration()
    path = Path(yaml_path)
    if not path.exists():
        return calibration

    with path.open('r', encoding='utf-8') as file:
        data = yaml.safe_load(file) or {}

    raw = data.get('imu_kalman_calibration', {}) or {}
    calibration['calibrated'] = bool(raw.get('calibrated', calibration['calibrated']))
    calibration['generated_at'] = str(raw.get('generated_at', calibration['generated_at']) or '')
    calibration['warmup_duration_sec'] = _coerce_float(
        raw.get('warmup_duration_sec'),
        calibration['warmup_duration_sec'],
    )
    calibration['accumulation_duration_sec'] = _coerce_float(
        raw.get('accumulation_duration_sec'),
        calibration['accumulation_duration_sec'],
    )
    calibration['sample_count'] = max(
        0,
        int(raw.get('sample_count', calibration['sample_count']) or 0),
    )

    for axis_name in AXES:
        axis_defaults = default_axis_calibration()
        axis_raw = raw.get(axis_name, {}) or {}
        axis_config = deepcopy(axis_defaults)
        axis_config['q_angle'] = _coerce_float(axis_raw.get('q_angle'), axis_defaults['q_angle'])
        axis_config['q_bias'] = _coerce_float(axis_raw.get('q_bias'), axis_defaults['q_bias'])
        axis_config['r_measure'] = _coerce_float(axis_raw.get('r_measure'), axis_defaults['r_measure'])
        axis_config['initial_bias_deg_s'] = _coerce_float(
            axis_raw.get('initial_bias_deg_s'),
            axis_defaults['initial_bias_deg_s'],
        )
        axis_config['use_fixed_gain'] = bool(
            axis_raw.get('use_fixed_gain', axis_defaults['use_fixed_gain'])
        )
        axis_config['kalman_gain_angle'] = _coerce_float(
            axis_raw.get('kalman_gain_angle'),
            axis_defaults['kalman_gain_angle'],
        )
        axis_config['kalman_gain_bias'] = _coerce_float(
            axis_raw.get('kalman_gain_bias'),
            axis_defaults['kalman_gain_bias'],
        )
        calibration[axis_name] = axis_config

    return calibration


def save_kalman_calibration(yaml_path: str, calibration: dict):
    path = Path(yaml_path)
    path.parent.mkdir(parents=True, exist_ok=True)

    cleaned = default_kalman_calibration()
    cleaned['calibrated'] = bool(calibration.get('calibrated', cleaned['calibrated']))
    cleaned['generated_at'] = str(calibration.get('generated_at', cleaned['generated_at']) or '')
    cleaned['warmup_duration_sec'] = _coerce_float(
        calibration.get('warmup_duration_sec'),
        cleaned['warmup_duration_sec'],
    )
    cleaned['accumulation_duration_sec'] = _coerce_float(
        calibration.get('accumulation_duration_sec'),
        cleaned['accumulation_duration_sec'],
    )
    cleaned['sample_count'] = max(0, int(calibration.get('sample_count', cleaned['sample_count']) or 0))

    for axis_name in AXES:
        axis_defaults = default_axis_calibration()
        axis_raw = calibration.get(axis_name, {}) or {}
        axis_cleaned = deepcopy(axis_defaults)
        axis_cleaned['q_angle'] = _coerce_float(axis_raw.get('q_angle'), axis_defaults['q_angle'])
        axis_cleaned['q_bias'] = _coerce_float(axis_raw.get('q_bias'), axis_defaults['q_bias'])
        axis_cleaned['r_measure'] = _coerce_float(axis_raw.get('r_measure'), axis_defaults['r_measure'])
        axis_cleaned['initial_bias_deg_s'] = _coerce_float(
            axis_raw.get('initial_bias_deg_s'),
            axis_defaults['initial_bias_deg_s'],
        )
        axis_cleaned['use_fixed_gain'] = bool(
            axis_raw.get('use_fixed_gain', axis_defaults['use_fixed_gain'])
        )
        axis_cleaned['kalman_gain_angle'] = _coerce_float(
            axis_raw.get('kalman_gain_angle'),
            axis_defaults['kalman_gain_angle'],
        )
        axis_cleaned['kalman_gain_bias'] = _coerce_float(
            axis_raw.get('kalman_gain_bias'),
            axis_defaults['kalman_gain_bias'],
        )
        cleaned[axis_name] = axis_cleaned

    with path.open('w', encoding='utf-8') as file:
        yaml.safe_dump({'imu_kalman_calibration': cleaned}, file, sort_keys=False)
