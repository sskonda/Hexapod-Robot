"""
Planar frame-conversion helpers shared by the maze-memory nodes.
"""

import math
from typing import Optional, Tuple

from rclpy.time import Time
from tf2_ros import Buffer


def quaternion_to_yaw(
    x_value: float,
    y_value: float,
    z_value: float,
    w_value: float,
) -> float:
    siny_cosp = 2.0 * (w_value * z_value + x_value * y_value)
    cosy_cosp = 1.0 - 2.0 * (y_value * y_value + z_value * z_value)
    return math.atan2(siny_cosp, cosy_cosp)


def transform_point_2d(
    x_m: float,
    y_m: float,
    translate_x_m: float,
    translate_y_m: float,
    yaw_rad: float,
) -> Tuple[float, float]:
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    return (
        translate_x_m + cos_yaw * x_m - sin_yaw * y_m,
        translate_y_m + sin_yaw * x_m + cos_yaw * y_m,
    )


def lookup_point_2d(
    tf_buffer: Buffer,
    target_frame: str,
    source_frame: str,
    x_m: float,
    y_m: float,
) -> Optional[Tuple[float, float]]:
    if not target_frame or not source_frame:
        return None
    if target_frame == source_frame:
        return x_m, y_m

    try:
        transform = tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            Time(),
        )
    except Exception:
        return None

    translation = transform.transform.translation
    rotation = transform.transform.rotation
    yaw_rad = quaternion_to_yaw(
        rotation.x,
        rotation.y,
        rotation.z,
        rotation.w,
    )
    return transform_point_2d(
        x_m,
        y_m,
        translation.x,
        translation.y,
        yaw_rad,
    )
