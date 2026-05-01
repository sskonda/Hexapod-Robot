#!/usr/bin/env python3
"""Simple LiDAR reactive explorer.

This node is intentionally small: it subscribes to a 2D LaserScan, chooses the
widest/deepest safe direction, and publishes cmd_vel for slow exploration while
slam_toolbox builds the map.
"""

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def quaternion_to_yaw(x_value: float, y_value: float, z_value: float, w_value: float) -> float:
    siny_cosp = 2.0 * (w_value * z_value + x_value * y_value)
    cosy_cosp = 1.0 - 2.0 * (y_value * y_value + z_value * z_value)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass(frozen=True)
class DirectionScore:
    angle_rad: float
    clearance_angle_rad: float
    mean_range_m: float
    min_range_m: float
    score: float


class LidarOpenSpaceExplorer(Node):
    def __init__(self):
        super().__init__('lidar_open_space_explorer')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('cmd_vel_rate_hz', 10.0)
        self.declare_parameter('scan_timeout_sec', 0.75)
        self.declare_parameter('enabled', True)
        self.declare_parameter('use_tf_for_scan_frame', True)
        self.declare_parameter('scan_yaw_offset_deg', 0.0)

        self.declare_parameter('max_speed_mps', 0.04)
        self.declare_parameter('min_speed_mps', 0.015)
        self.declare_parameter('max_lateral_speed_mps', 0.035)
        self.declare_parameter('max_turn_rate_rad_s', 0.18)
        self.declare_parameter('turn_gain', 0.8)
        self.declare_parameter('crab_motion', True)

        self.declare_parameter('obstacle_stop_distance_m', 0.28)
        self.declare_parameter('obstacle_slow_distance_m', 0.65)
        self.declare_parameter('desired_clearance_m', 0.55)
        self.declare_parameter('minimum_direction_range_m', 0.35)
        self.declare_parameter('max_usable_range_m', 4.0)

        self.declare_parameter('candidate_angle_min_deg', -150.0)
        self.declare_parameter('candidate_angle_max_deg', 150.0)
        self.declare_parameter('direction_window_deg', 18.0)
        self.declare_parameter('candidate_stride_deg', 5.0)
        self.declare_parameter('forward_bias_weight', 0.45)
        self.declare_parameter('distance_weight', 1.0)
        self.declare_parameter('clearance_angle_weight', 1.4)
        self.declare_parameter('reverse_allowed', False)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.latest_scan = None
        self.latest_scan_time = None
        self.last_choice = None

        self._load_parameters()

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.add_on_set_parameters_callback(self.parameter_update_callback)
        self.create_timer(1.0 / self.cmd_vel_rate_hz, self.control_loop)

        self.get_logger().info(
            'LiDAR open-space explorer ready: '
            f'scan={self.scan_topic}, cmd_vel={self.cmd_vel_topic}, '
            f'max_speed={self.max_speed_mps:.3f} m/s, crab_motion={self.crab_motion}'
        )

    def _load_parameters(self):
        self.cmd_vel_rate_hz = max(1.0, float(self.get_parameter('cmd_vel_rate_hz').value))
        self.scan_timeout_sec = max(0.1, float(self.get_parameter('scan_timeout_sec').value))
        self.enabled = as_bool(self.get_parameter('enabled').value)
        self.use_tf_for_scan_frame = as_bool(self.get_parameter('use_tf_for_scan_frame').value)
        self.scan_yaw_offset_rad = math.radians(
            float(self.get_parameter('scan_yaw_offset_deg').value)
        )

        self.max_speed_mps = max(0.0, float(self.get_parameter('max_speed_mps').value))
        self.min_speed_mps = max(0.0, float(self.get_parameter('min_speed_mps').value))
        self.max_lateral_speed_mps = max(
            0.0,
            float(self.get_parameter('max_lateral_speed_mps').value),
        )
        self.max_turn_rate_rad_s = max(
            0.0,
            float(self.get_parameter('max_turn_rate_rad_s').value),
        )
        self.turn_gain = max(0.0, float(self.get_parameter('turn_gain').value))
        self.crab_motion = as_bool(self.get_parameter('crab_motion').value)

        self.obstacle_stop_distance_m = max(
            0.0,
            float(self.get_parameter('obstacle_stop_distance_m').value),
        )
        self.obstacle_slow_distance_m = max(
            self.obstacle_stop_distance_m + 0.01,
            float(self.get_parameter('obstacle_slow_distance_m').value),
        )
        self.desired_clearance_m = max(
            self.obstacle_stop_distance_m,
            float(self.get_parameter('desired_clearance_m').value),
        )
        self.minimum_direction_range_m = max(
            self.obstacle_stop_distance_m,
            float(self.get_parameter('minimum_direction_range_m').value),
        )
        self.max_usable_range_m = max(
            self.minimum_direction_range_m,
            float(self.get_parameter('max_usable_range_m').value),
        )

        angle_min = math.radians(float(self.get_parameter('candidate_angle_min_deg').value))
        angle_max = math.radians(float(self.get_parameter('candidate_angle_max_deg').value))
        self.candidate_angle_min_rad = min(angle_min, angle_max)
        self.candidate_angle_max_rad = max(angle_min, angle_max)
        self.direction_window_rad = math.radians(
            max(1.0, float(self.get_parameter('direction_window_deg').value))
        )
        self.candidate_stride_rad = math.radians(
            max(1.0, float(self.get_parameter('candidate_stride_deg').value))
        )
        self.forward_bias_weight = max(
            0.0,
            float(self.get_parameter('forward_bias_weight').value),
        )
        self.distance_weight = max(0.0, float(self.get_parameter('distance_weight').value))
        self.clearance_angle_weight = max(
            0.0,
            float(self.get_parameter('clearance_angle_weight').value),
        )
        self.reverse_allowed = as_bool(self.get_parameter('reverse_allowed').value)

    def parameter_update_callback(self, parameters):
        for parameter in parameters:
            if parameter.name == 'base_frame':
                self.base_frame = str(parameter.value)
            elif parameter.name == 'enabled':
                self.enabled = as_bool(parameter.value)
            elif parameter.name == 'use_tf_for_scan_frame':
                self.use_tf_for_scan_frame = as_bool(parameter.value)
            elif parameter.name == 'scan_yaw_offset_deg':
                self.scan_yaw_offset_rad = math.radians(float(parameter.value))
            elif parameter.name == 'max_speed_mps':
                self.max_speed_mps = max(0.0, float(parameter.value))
            elif parameter.name == 'min_speed_mps':
                self.min_speed_mps = max(0.0, float(parameter.value))
            elif parameter.name == 'max_lateral_speed_mps':
                self.max_lateral_speed_mps = max(0.0, float(parameter.value))
            elif parameter.name == 'max_turn_rate_rad_s':
                self.max_turn_rate_rad_s = max(0.0, float(parameter.value))
            elif parameter.name == 'turn_gain':
                self.turn_gain = max(0.0, float(parameter.value))
            elif parameter.name == 'crab_motion':
                self.crab_motion = as_bool(parameter.value)
            elif parameter.name == 'obstacle_stop_distance_m':
                self.obstacle_stop_distance_m = max(0.0, float(parameter.value))
            elif parameter.name == 'obstacle_slow_distance_m':
                self.obstacle_slow_distance_m = max(
                    self.obstacle_stop_distance_m + 0.01,
                    float(parameter.value),
                )
            elif parameter.name == 'desired_clearance_m':
                self.desired_clearance_m = max(
                    self.obstacle_stop_distance_m,
                    float(parameter.value),
                )
            elif parameter.name == 'minimum_direction_range_m':
                self.minimum_direction_range_m = max(
                    self.obstacle_stop_distance_m,
                    float(parameter.value),
                )
            elif parameter.name == 'max_usable_range_m':
                self.max_usable_range_m = max(
                    self.minimum_direction_range_m,
                    float(parameter.value),
                )
            elif parameter.name == 'candidate_angle_min_deg':
                self.candidate_angle_min_rad = math.radians(float(parameter.value))
            elif parameter.name == 'candidate_angle_max_deg':
                self.candidate_angle_max_rad = math.radians(float(parameter.value))
            elif parameter.name == 'direction_window_deg':
                self.direction_window_rad = math.radians(max(1.0, float(parameter.value)))
            elif parameter.name == 'candidate_stride_deg':
                self.candidate_stride_rad = math.radians(max(1.0, float(parameter.value)))
            elif parameter.name == 'forward_bias_weight':
                self.forward_bias_weight = max(0.0, float(parameter.value))
            elif parameter.name == 'distance_weight':
                self.distance_weight = max(0.0, float(parameter.value))
            elif parameter.name == 'clearance_angle_weight':
                self.clearance_angle_weight = max(0.0, float(parameter.value))
            elif parameter.name == 'reverse_allowed':
                self.reverse_allowed = as_bool(parameter.value)

        if self.candidate_angle_min_rad > self.candidate_angle_max_rad:
            self.candidate_angle_min_rad, self.candidate_angle_max_rad = (
                self.candidate_angle_max_rad,
                self.candidate_angle_min_rad,
            )
        return SetParametersResult(successful=True)

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg
        self.latest_scan_time = self.get_clock().now()

    def control_loop(self):
        cmd = Twist()

        if not self.enabled or self.latest_scan is None or self.latest_scan_time is None:
            self.cmd_pub.publish(cmd)
            return

        scan_age_sec = (
            self.get_clock().now() - self.latest_scan_time
        ).nanoseconds * 1e-9
        if scan_age_sec > self.scan_timeout_sec:
            self.get_logger().warn(
                f'Scan timeout ({scan_age_sec:.2f}s old). Stopping.',
                throttle_duration_sec=2.0,
            )
            self.cmd_pub.publish(cmd)
            return

        best = self.choose_best_direction(self.latest_scan)
        if best is None:
            cmd.angular.z = self.max_turn_rate_rad_s
            self.cmd_pub.publish(cmd)
            return

        body_angle_rad = self.scan_angle_to_body_angle(self.latest_scan, best.angle_rad)
        if best.min_range_m < self.obstacle_stop_distance_m:
            cmd.angular.z = clamp(
                self.turn_gain * body_angle_rad,
                -self.max_turn_rate_rad_s,
                self.max_turn_rate_rad_s,
            )
            if abs(cmd.angular.z) < 1e-3:
                cmd.angular.z = self.max_turn_rate_rad_s
            self.cmd_pub.publish(cmd)
            return

        speed = self.speed_for_clearance(best.min_range_m)
        if self.crab_motion:
            cmd.linear.x = speed * math.cos(body_angle_rad)
            cmd.linear.y = speed * math.sin(body_angle_rad)
            cmd.linear.y = clamp(
                cmd.linear.y,
                -self.max_lateral_speed_mps,
                self.max_lateral_speed_mps,
            )
            if not self.reverse_allowed and cmd.linear.x < 0.0:
                cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            turn = clamp(
                self.turn_gain * body_angle_rad,
                -self.max_turn_rate_rad_s,
                self.max_turn_rate_rad_s,
            )
            cmd.linear.x = speed * max(0.0, math.cos(body_angle_rad))
            cmd.angular.z = turn

        self.last_choice = best
        self.cmd_pub.publish(cmd)

    def scan_angle_to_body_angle(self, scan: LaserScan, scan_angle_rad: float) -> float:
        if not self.use_tf_for_scan_frame:
            return scan_angle_rad + self.scan_yaw_offset_rad

        scan_frame = scan.header.frame_id.strip()
        if not scan_frame:
            return scan_angle_rad + self.scan_yaw_offset_rad

        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                scan_frame,
                Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'Could not transform {scan_frame} to {self.base_frame}: {exc}. '
                'Using scan_yaw_offset_deg fallback.',
                throttle_duration_sec=2.0,
            )
            return scan_angle_rad + self.scan_yaw_offset_rad

        rotation = transform.transform.rotation
        yaw_rad = quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w)
        return scan_angle_rad + yaw_rad

    def speed_for_clearance(self, clearance_m: float) -> float:
        if self.max_speed_mps <= 0.0:
            return 0.0
        if clearance_m <= self.obstacle_stop_distance_m:
            return 0.0
        if clearance_m >= self.obstacle_slow_distance_m:
            return self.max_speed_mps

        ratio = (
            (clearance_m - self.obstacle_stop_distance_m)
            / (self.obstacle_slow_distance_m - self.obstacle_stop_distance_m)
        )
        return self.min_speed_mps + ratio * (self.max_speed_mps - self.min_speed_mps)

    def choose_best_direction(self, scan: LaserScan):
        ranges = list(scan.ranges)
        if not ranges or scan.angle_increment == 0.0:
            return None

        best = None
        candidate_angle = self.candidate_angle_min_rad
        while candidate_angle <= self.candidate_angle_max_rad + 1e-9:
            score = self.score_direction(scan, ranges, candidate_angle)
            if score is not None and (best is None or score.score > best.score):
                best = score
            candidate_angle += self.candidate_stride_rad

        return best

    def score_direction(self, scan: LaserScan, ranges: list[float], angle_rad: float):
        sample_indices = self.indices_in_window(scan, angle_rad, self.direction_window_rad)
        if not sample_indices:
            return None

        valid_ranges = [
            self.sanitize_range(ranges[index], scan.range_min, scan.range_max)
            for index in sample_indices
        ]
        valid_ranges = [value for value in valid_ranges if value is not None]
        if not valid_ranges:
            return None

        min_range = min(valid_ranges)
        mean_range = sum(valid_ranges) / len(valid_ranges)
        if min_range < self.minimum_direction_range_m:
            return None

        clearance_angle = self.clearance_angle(scan, ranges, angle_rad)
        mean_range = min(mean_range, self.max_usable_range_m)
        forward_bias = max(0.0, math.cos(angle_rad))
        score = (
            self.distance_weight * mean_range
            + self.clearance_angle_weight * clearance_angle
            + self.forward_bias_weight * forward_bias
        )
        return DirectionScore(
            angle_rad=angle_rad,
            clearance_angle_rad=clearance_angle,
            mean_range_m=mean_range,
            min_range_m=min_range,
            score=score,
        )

    def clearance_angle(self, scan: LaserScan, ranges: list[float], center_angle_rad: float):
        center_index = self.index_for_angle(scan, center_angle_rad)
        if center_index is None:
            return 0.0

        left_count = self.count_clear_beams(scan, ranges, center_index, step=1)
        right_count = self.count_clear_beams(scan, ranges, center_index, step=-1)
        beam_count = max(0, left_count + right_count - 1)
        return beam_count * abs(scan.angle_increment)

    def count_clear_beams(self, scan: LaserScan, ranges: list[float], start_index: int, step: int):
        count = 0
        index = start_index
        while 0 <= index < len(ranges):
            angle = scan.angle_min + index * scan.angle_increment
            if angle < self.candidate_angle_min_rad or angle > self.candidate_angle_max_rad:
                break

            value = self.sanitize_range(ranges[index], scan.range_min, scan.range_max)
            if value is None or value < self.desired_clearance_m:
                break

            count += 1
            index += step
        return count

    def indices_in_window(self, scan: LaserScan, center_angle_rad: float, half_width_rad: float):
        start_angle = center_angle_rad - half_width_rad
        end_angle = center_angle_rad + half_width_rad
        start_index = self.index_for_angle(scan, start_angle)
        end_index = self.index_for_angle(scan, end_angle)
        if start_index is None or end_index is None:
            return []

        lo = max(0, min(start_index, end_index))
        hi = min(len(scan.ranges) - 1, max(start_index, end_index))
        return list(range(lo, hi + 1))

    def index_for_angle(self, scan: LaserScan, angle_rad: float):
        if angle_rad < scan.angle_min or angle_rad > scan.angle_max:
            return None
        index = int(round((angle_rad - scan.angle_min) / scan.angle_increment))
        if index < 0 or index >= len(scan.ranges):
            return None
        return index

    def sanitize_range(self, value: float, range_min: float, range_max: float):
        if math.isnan(value):
            return None
        if math.isinf(value):
            return self.max_usable_range_m if value > 0.0 else None
        if value < range_min:
            return None
        if value > range_max:
            return min(range_max, self.max_usable_range_m)
        return min(value, self.max_usable_range_m)


def main(args=None):
    rclpy.init(args=args)
    node = LidarOpenSpaceExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
