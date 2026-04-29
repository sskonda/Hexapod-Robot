#!/usr/bin/env python3

"""Helpers for IMU yaw fallback selection and gyro yaw integration."""

import math


def normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(float(angle_rad)), math.cos(float(angle_rad)))


def integrate_relative_yaw(current_yaw_rad: float, yaw_rate_rad_s: float, dt: float) -> float:
    return normalize_angle(float(current_yaw_rad) + float(yaw_rate_rad_s) * max(0.0, float(dt)))


def should_use_mpu6050_yaw_fallback(
    primary_yaw_valid: bool,
    fallback_enabled: bool,
    fallback_available: bool,
    elapsed_sec: float,
    timeout_sec: float,
    fallback_latched: bool = False,
) -> bool:
    if primary_yaw_valid and not fallback_latched:
        return False

    return (
        bool(fallback_enabled)
        and bool(fallback_available)
        and (
            bool(fallback_latched)
            or float(elapsed_sec) >= max(0.0, float(timeout_sec))
        )
    )
