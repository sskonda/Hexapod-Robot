#!/usr/bin/env python3
"""Lower-layer scan-based safety filter for commanded motion.

The gap-following explorer decides *where* the robot should go, but this node
acts as a last-line watchdog beneath the planner/follower.  It monitors the
current translation direction in ``cmd_vel`` and reduces or blocks the linear
command if the LiDAR no longer shows enough clearance in that sector.

To avoid transient false-clear readings from the hexapod's legs sweeping
through the LiDAR plane, the safety check uses the minimum clearance across the
current and previous scan.
"""

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def rotate_vector(x_value: float, y_value: float, yaw_rad: float) -> tuple[float, float]:
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    return (
        cos_yaw * x_value - sin_yaw * y_value,
        sin_yaw * x_value + cos_yaw * y_value,
    )


def valid_range(scan: LaserScan, index: int) -> Optional[float]:
    if index < 0 or index >= len(scan.ranges):
        return None

    range_value = scan.ranges[index]
    if math.isnan(range_value):
        return None
    if math.isinf(range_value):
        return scan.range_max
    if range_value < scan.range_min:
        return scan.range_min
    if range_value > scan.range_max:
        return scan.range_max
    return range_value


class ScanCmdVelSafety(Node):
    def __init__(self):
        super().__init__('scan_cmd_vel_safety')

        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('scan_yaw_offset_rad', 0.0)
        self.declare_parameter('cmd_vel_yaw_offset_rad', 0.0)
        self.declare_parameter('input_cmd_vel_topic', 'cmd_vel_nav')
        self.declare_parameter('output_cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('scan_timeout_sec', 0.5)
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('clearance_window_deg', 15.0)
        self.declare_parameter('stop_distance_m', 0.65)
        self.declare_parameter('slowdown_distance_m', 0.85)
        self.declare_parameter('side_clearance_window_deg', 20.0)
        self.declare_parameter('side_stop_distance_m', 0.38)
        self.declare_parameter('side_slowdown_distance_m', 0.52)
        self.declare_parameter('max_side_push_ratio', 0.75)
        self.declare_parameter('side_push_deadband_m', 0.05)
        self.declare_parameter('preserve_turning_when_blocked', False)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.scan_yaw_offset_rad = float(self.get_parameter('scan_yaw_offset_rad').value)
        self.cmd_vel_yaw_offset_rad = float(
            self.get_parameter('cmd_vel_yaw_offset_rad').value
        )
        self.input_cmd_vel_topic = str(self.get_parameter('input_cmd_vel_topic').value)
        self.output_cmd_vel_topic = str(self.get_parameter('output_cmd_vel_topic').value)
        self.control_rate_hz = max(1.0, float(self.get_parameter('control_rate_hz').value))
        self.scan_timeout_sec = max(0.1, float(self.get_parameter('scan_timeout_sec').value))
        self.cmd_timeout_sec = max(0.1, float(self.get_parameter('cmd_timeout_sec').value))
        self.clearance_window_rad = math.radians(
            max(1.0, float(self.get_parameter('clearance_window_deg').value))
        )
        self.stop_distance_m = max(0.05, float(self.get_parameter('stop_distance_m').value))
        self.slowdown_distance_m = max(
            self.stop_distance_m,
            float(self.get_parameter('slowdown_distance_m').value),
        )
        self.side_clearance_window_rad = math.radians(
            max(1.0, float(self.get_parameter('side_clearance_window_deg').value))
        )
        self.side_stop_distance_m = max(
            0.05,
            float(self.get_parameter('side_stop_distance_m').value),
        )
        self.side_slowdown_distance_m = max(
            self.side_stop_distance_m,
            float(self.get_parameter('side_slowdown_distance_m').value),
        )
        self.max_side_push_ratio = clamp(
            float(self.get_parameter('max_side_push_ratio').value),
            0.0,
            2.0,
        )
        self.side_push_deadband_m = max(
            0.0,
            float(self.get_parameter('side_push_deadband_m').value),
        )
        self.preserve_turning_when_blocked = bool(
            self.get_parameter('preserve_turning_when_blocked').value
        )

        self.latest_scan: Optional[LaserScan] = None
        self.previous_scan: Optional[LaserScan] = None
        self.latest_scan_time = None
        self.latest_cmd: Optional[Twist] = None
        self.latest_cmd_time = None
        self.last_clearance_state = 'clear'

        self.cmd_publisher = self.create_publisher(Twist, self.output_cmd_vel_topic, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.create_subscription(Twist, self.input_cmd_vel_topic, self.cmd_callback, 10)
        self.create_timer(1.0 / self.control_rate_hz, self.control_loop)

        self.get_logger().info(
            f'Scan cmd_vel safety ready — {self.input_cmd_vel_topic} -> {self.output_cmd_vel_topic}, '
            f'stop {self.stop_distance_m:.2f} m, slowdown {self.slowdown_distance_m:.2f} m'
        )
        self.get_logger().info(
            f'Safety uses scan yaw offset {math.degrees(self.scan_yaw_offset_rad):.1f} deg and '
            f'cmd_vel yaw offset {math.degrees(self.cmd_vel_yaw_offset_rad):.1f} deg.'
        )
        self.get_logger().info(
            f'Side-wall safety: stop {self.side_stop_distance_m:.2f} m, '
            f'slowdown {self.side_slowdown_distance_m:.2f} m, '
            f'max push ratio {self.max_side_push_ratio:.2f}.'
        )

    def scan_callback(self, msg: LaserScan):
        self.previous_scan = self.latest_scan
        self.latest_scan = msg
        self.latest_scan_time = self.get_clock().now()

    def cmd_callback(self, msg: Twist):
        self.latest_cmd = msg
        self.latest_cmd_time = self.get_clock().now()

    def control_loop(self):
        filtered_cmd = Twist()

        if self.latest_cmd is None or self.latest_cmd_time is None:
            self.cmd_publisher.publish(filtered_cmd)
            return

        now = self.get_clock().now()
        cmd_age_sec = (now - self.latest_cmd_time).nanoseconds * 1e-9
        if cmd_age_sec > self.cmd_timeout_sec:
            self.cmd_publisher.publish(filtered_cmd)
            return

        filtered_cmd.angular.z = self.latest_cmd.angular.z
        translation_speed = math.hypot(self.latest_cmd.linear.x, self.latest_cmd.linear.y)
        if translation_speed < 1e-6:
            self.cmd_publisher.publish(filtered_cmd)
            return

        if not self.scan_is_fresh():
            if not self.preserve_turning_when_blocked:
                filtered_cmd.angular.z = 0.0
            self._log_state('blocked', 0.0, 'stale LiDAR data')
            self.cmd_publisher.publish(filtered_cmd)
            return

        motion_vector_x, motion_vector_y = rotate_vector(
            self.latest_cmd.linear.x,
            self.latest_cmd.linear.y,
            self.cmd_vel_yaw_offset_rad,
        )
        motion_angle_rad = math.atan2(motion_vector_y, motion_vector_x)
        forward_clearance_m = self.clearance_for_angle(motion_angle_rad)
        left_clearance_m = self.clearance_for_angle(
            motion_angle_rad + math.pi / 2.0,
            window_rad=self.side_clearance_window_rad,
        )
        right_clearance_m = self.clearance_for_angle(
            motion_angle_rad - math.pi / 2.0,
            window_rad=self.side_clearance_window_rad,
        )
        forward_scale = self.translation_scale(forward_clearance_m)
        side_scale = min(
            self.clearance_scale(
                left_clearance_m,
                self.side_stop_distance_m,
                self.side_slowdown_distance_m,
            ),
            self.clearance_scale(
                right_clearance_m,
                self.side_stop_distance_m,
                self.side_slowdown_distance_m,
            ),
        )
        scale = min(forward_scale, side_scale)

        side_push_ratio = self.side_push_ratio(left_clearance_m, right_clearance_m)
        away_sign = 0.0
        if left_clearance_m + self.side_push_deadband_m < right_clearance_m:
            away_sign = -1.0
        elif right_clearance_m + self.side_push_deadband_m < left_clearance_m:
            away_sign = 1.0

        if scale > 0.0:
            forward_x = math.cos(motion_angle_rad)
            forward_y = math.sin(motion_angle_rad)
            corrected_x = forward_x
            corrected_y = forward_y
            if side_push_ratio > 0.0 and away_sign != 0.0:
                corrected_x += away_sign * side_push_ratio * -forward_y
                corrected_y += away_sign * side_push_ratio * forward_x
            corrected_norm = math.hypot(corrected_x, corrected_y)
            if corrected_norm > 1e-6:
                corrected_x /= corrected_norm
                corrected_y /= corrected_norm
            corrected_speed = translation_speed * scale
            corrected_cmd_x, corrected_cmd_y = rotate_vector(
                corrected_x * corrected_speed,
                corrected_y * corrected_speed,
                -self.cmd_vel_yaw_offset_rad,
            )
            filtered_cmd.linear.x = corrected_cmd_x
            filtered_cmd.linear.y = corrected_cmd_y

        if scale <= 0.0 and not self.preserve_turning_when_blocked:
            filtered_cmd.angular.z = 0.0

        if scale <= 0.0:
            self._log_state(
                'blocked',
                min(forward_clearance_m, left_clearance_m, right_clearance_m),
                (
                    f'stopping translation '
                    f'(front {forward_clearance_m:.2f} m, left {left_clearance_m:.2f} m, '
                    f'right {right_clearance_m:.2f} m)'
                ),
            )
        elif scale < 1.0:
            if side_push_ratio > 0.0 and away_sign != 0.0:
                push_direction = 'right' if away_sign < 0.0 else 'left'
                description = (
                    f'scaling translation to {scale:.2f} and nudging {push_direction} '
                    f'(front {forward_clearance_m:.2f} m, left {left_clearance_m:.2f} m, '
                    f'right {right_clearance_m:.2f} m)'
                )
            else:
                description = (
                    f'scaling translation to {scale:.2f} '
                    f'(front {forward_clearance_m:.2f} m, left {left_clearance_m:.2f} m, '
                    f'right {right_clearance_m:.2f} m)'
                )
            self._log_state('slow', min(forward_clearance_m, left_clearance_m, right_clearance_m), description)
        elif side_push_ratio > 0.0 and away_sign != 0.0:
            push_direction = 'right' if away_sign < 0.0 else 'left'
            self._log_state(
                'slow',
                min(forward_clearance_m, left_clearance_m, right_clearance_m),
                (
                    f'nudging {push_direction} away from side wall '
                    f'(front {forward_clearance_m:.2f} m, left {left_clearance_m:.2f} m, '
                    f'right {right_clearance_m:.2f} m)'
                ),
            )
        else:
            self._log_state(
                'clear',
                min(forward_clearance_m, left_clearance_m, right_clearance_m),
                'path clear',
            )

        self.cmd_publisher.publish(filtered_cmd)

    def scan_is_fresh(self) -> bool:
        if self.latest_scan is None or self.latest_scan_time is None:
            return False
        age_sec = (self.get_clock().now() - self.latest_scan_time).nanoseconds * 1e-9
        return age_sec <= self.scan_timeout_sec

    def translation_scale(self, clearance_m: float) -> float:
        return self.clearance_scale(
            clearance_m,
            self.stop_distance_m,
            self.slowdown_distance_m,
        )

    def clearance_scale(
        self,
        clearance_m: float,
        stop_distance_m: float,
        slowdown_distance_m: float,
    ) -> float:
        if clearance_m <= stop_distance_m:
            return 0.0
        if clearance_m >= slowdown_distance_m:
            return 1.0
        span = slowdown_distance_m - stop_distance_m
        if span <= 1e-6:
            return 0.0
        return clamp((clearance_m - stop_distance_m) / span, 0.0, 1.0)

    def side_push_ratio(self, left_clearance_m: float, right_clearance_m: float) -> float:
        closer_clearance_m = min(left_clearance_m, right_clearance_m)
        if closer_clearance_m >= self.side_slowdown_distance_m:
            return 0.0
        span = self.side_slowdown_distance_m - self.side_stop_distance_m
        if span <= 1e-6:
            return 0.0
        closeness = clamp(
            (self.side_slowdown_distance_m - closer_clearance_m) / span,
            0.0,
            1.0,
        )
        imbalance = clamp(
            (
                abs(left_clearance_m - right_clearance_m) - self.side_push_deadband_m
            ) / max(self.side_slowdown_distance_m, 1e-6),
            0.0,
            1.0,
        )
        return self.max_side_push_ratio * closeness * imbalance

    def clearance_for_angle(self, target_angle_rad: float, window_rad: Optional[float] = None) -> float:
        current = self.scan_window_clearance(
            self.latest_scan,
            target_angle_rad,
            window_rad=window_rad,
        )
        previous = self.scan_window_clearance(
            self.previous_scan,
            target_angle_rad,
            window_rad=window_rad,
        )

        if current is None:
            return 0.0 if previous is None else previous
        if previous is None:
            return current
        return min(current, previous)

    def scan_window_clearance(
        self,
        scan: Optional[LaserScan],
        target_angle_rad: float,
        window_rad: Optional[float] = None,
    ) -> Optional[float]:
        if scan is None or len(scan.ranges) == 0 or abs(scan.angle_increment) < 1e-9:
            return None

        angle_increment = abs(scan.angle_increment)
        window_width_rad = self.clearance_window_rad if window_rad is None else window_rad
        window_beams = max(1, int(window_width_rad / angle_increment))
        target_scan_angle_rad = target_angle_rad - self.scan_yaw_offset_rad
        raw_index = (target_scan_angle_rad - scan.angle_min) / scan.angle_increment
        center_index = int(round(raw_index))
        center_index = max(window_beams, min(len(scan.ranges) - 1 - window_beams, center_index))

        minimum_clearance = None
        for index in range(center_index - window_beams, center_index + window_beams + 1):
            range_value = valid_range(scan, index)
            if range_value is None:
                continue
            if minimum_clearance is None or range_value < minimum_clearance:
                minimum_clearance = range_value
        return minimum_clearance

    def _log_state(self, state: str, clearance_m: float, description: str):
        if state == self.last_clearance_state:
            return
        self.last_clearance_state = state
        self.get_logger().info(
            f'Safety state {state}: clearance {clearance_m:.2f} m, {description}.'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ScanCmdVelSafety()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
