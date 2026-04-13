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

        motion_angle_rad = (
            math.atan2(self.latest_cmd.linear.y, self.latest_cmd.linear.x)
            + self.cmd_vel_yaw_offset_rad
        )
        clearance_m = self.clearance_for_angle(motion_angle_rad)
        scale = self.translation_scale(clearance_m)

        filtered_cmd.linear.x = self.latest_cmd.linear.x * scale
        filtered_cmd.linear.y = self.latest_cmd.linear.y * scale
        if scale <= 0.0 and not self.preserve_turning_when_blocked:
            filtered_cmd.angular.z = 0.0

        if scale <= 0.0:
            self._log_state('blocked', clearance_m, 'stopping translation')
        elif scale < 1.0:
            self._log_state('slow', clearance_m, f'scaling translation to {scale:.2f}')
        else:
            self._log_state('clear', clearance_m, 'path clear')

        self.cmd_publisher.publish(filtered_cmd)

    def scan_is_fresh(self) -> bool:
        if self.latest_scan is None or self.latest_scan_time is None:
            return False
        age_sec = (self.get_clock().now() - self.latest_scan_time).nanoseconds * 1e-9
        return age_sec <= self.scan_timeout_sec

    def translation_scale(self, clearance_m: float) -> float:
        if clearance_m <= self.stop_distance_m:
            return 0.0
        if clearance_m >= self.slowdown_distance_m:
            return 1.0
        span = self.slowdown_distance_m - self.stop_distance_m
        if span <= 1e-6:
            return 0.0
        return clamp((clearance_m - self.stop_distance_m) / span, 0.0, 1.0)

    def clearance_for_angle(self, target_angle_rad: float) -> float:
        current = self.scan_window_clearance(self.latest_scan, target_angle_rad)
        previous = self.scan_window_clearance(self.previous_scan, target_angle_rad)

        if current is None:
            return 0.0 if previous is None else previous
        if previous is None:
            return current
        return min(current, previous)

    def scan_window_clearance(self, scan: Optional[LaserScan], target_angle_rad: float) -> Optional[float]:
        if scan is None or len(scan.ranges) == 0 or abs(scan.angle_increment) < 1e-9:
            return None

        angle_increment = abs(scan.angle_increment)
        window_beams = max(1, int(self.clearance_window_rad / angle_increment))
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
