#!/usr/bin/env python3
"""Simple LiDAR reactive explorer.

This node is intentionally small: it subscribes to a 2D LaserScan, chooses the
widest/deepest safe direction, and publishes cmd_vel for slow exploration while
slam_toolbox builds the map.
"""

import math
from collections import deque
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


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


@dataclass(frozen=True)
class FrontierTarget:
    x_m: float
    y_m: float
    grid_x: int
    grid_y: int
    searched_cells: int
    path: tuple[tuple[float, float], ...]


@dataclass(frozen=True)
class SuppressedFrontier:
    x_m: float
    y_m: float
    expires_at_sec: float
    reason: str


class LidarOpenSpaceExplorer(Node):
    def __init__(self):
        super().__init__('lidar_open_space_explorer')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('cmd_vel_rate_hz', 10.0)
        self.declare_parameter('scan_timeout_sec', 0.75)
        self.declare_parameter('map_timeout_sec', 2.5)
        self.declare_parameter('enabled', True)
        self.declare_parameter('exploration_mode', 'frontier')
        self.declare_parameter('search_strategy', 'bfs')
        self.declare_parameter('reactive_fallback', True)
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
        self.declare_parameter('frontier_replan_period_sec', 2.0)
        self.declare_parameter('frontier_goal_tolerance_m', 0.18)
        self.declare_parameter('frontier_waypoint_spacing_m', 0.25)
        self.declare_parameter('frontier_min_clearance_m', 0.25)
        self.declare_parameter('frontier_free_threshold', 25)
        self.declare_parameter('frontier_occupied_threshold', 65)
        self.declare_parameter('frontier_suppression_duration_sec', 15.0)
        self.declare_parameter('frontier_suppression_radius_m', 0.60)
        self.declare_parameter('frontier_blocked_clearance_margin_m', 0.05)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.latest_scan = None
        self.latest_scan_time = None
        self.latest_map = None
        self.latest_map_time = None
        self.last_choice = None
        self.current_frontier = None
        self.current_path_index = 0
        self.last_frontier_plan_time = None
        self.suppressed_frontiers = []

        self._load_parameters()

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, 10)
        self.add_on_set_parameters_callback(self.parameter_update_callback)
        self.create_timer(1.0 / self.cmd_vel_rate_hz, self.control_loop)

        self.get_logger().info(
            'LiDAR open-space explorer ready: '
            f'scan={self.scan_topic}, map={self.map_topic}, cmd_vel={self.cmd_vel_topic}, '
            f'mode={self.exploration_mode}, search={self.search_strategy}, '
            f'max_speed={self.max_speed_mps:.3f} m/s'
        )

    def _load_parameters(self):
        self.cmd_vel_rate_hz = max(1.0, float(self.get_parameter('cmd_vel_rate_hz').value))
        self.scan_timeout_sec = max(0.1, float(self.get_parameter('scan_timeout_sec').value))
        self.map_timeout_sec = max(0.1, float(self.get_parameter('map_timeout_sec').value))
        self.enabled = as_bool(self.get_parameter('enabled').value)
        self.exploration_mode = self.normalized_mode(
            str(self.get_parameter('exploration_mode').value)
        )
        self.search_strategy = self.normalized_search_strategy(
            str(self.get_parameter('search_strategy').value)
        )
        self.reactive_fallback = as_bool(self.get_parameter('reactive_fallback').value)
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
        self.frontier_replan_period_sec = max(
            0.5,
            float(self.get_parameter('frontier_replan_period_sec').value),
        )
        self.frontier_goal_tolerance_m = max(
            0.03,
            float(self.get_parameter('frontier_goal_tolerance_m').value),
        )
        self.frontier_waypoint_spacing_m = max(
            self.frontier_goal_tolerance_m,
            float(self.get_parameter('frontier_waypoint_spacing_m').value),
        )
        self.frontier_min_clearance_m = max(
            0.0,
            float(self.get_parameter('frontier_min_clearance_m').value),
        )
        self.frontier_free_threshold = int(self.get_parameter('frontier_free_threshold').value)
        self.frontier_occupied_threshold = int(
            self.get_parameter('frontier_occupied_threshold').value
        )
        self.frontier_suppression_duration_sec = max(
            0.0,
            float(self.get_parameter('frontier_suppression_duration_sec').value),
        )
        self.frontier_suppression_radius_m = max(
            0.0,
            float(self.get_parameter('frontier_suppression_radius_m').value),
        )
        self.frontier_blocked_clearance_margin_m = max(
            0.0,
            float(self.get_parameter('frontier_blocked_clearance_margin_m').value),
        )

    def normalized_mode(self, value: str) -> str:
        mode = value.strip().lower()
        if mode not in ('frontier', 'reactive'):
            self.get_logger().warn(
                f'Unsupported exploration_mode "{value}". Falling back to frontier.',
                throttle_duration_sec=2.0,
            )
            return 'frontier'
        return mode

    def normalized_search_strategy(self, value: str) -> str:
        strategy = value.strip().lower()
        if strategy not in ('bfs', 'dfs'):
            self.get_logger().warn(
                f'Unsupported search_strategy "{value}". Falling back to bfs.',
                throttle_duration_sec=2.0,
            )
            return 'bfs'
        return strategy

    def parameter_update_callback(self, parameters):
        for parameter in parameters:
            if parameter.name == 'map_frame':
                self.map_frame = str(parameter.value)
            elif parameter.name == 'map_timeout_sec':
                self.map_timeout_sec = max(0.1, float(parameter.value))
            elif parameter.name == 'exploration_mode':
                self.exploration_mode = self.normalized_mode(str(parameter.value))
                self.current_frontier = None
                self.last_frontier_plan_time = None
            elif parameter.name == 'search_strategy':
                self.search_strategy = self.normalized_search_strategy(str(parameter.value))
                self.current_frontier = None
                self.last_frontier_plan_time = None
            elif parameter.name == 'reactive_fallback':
                self.reactive_fallback = as_bool(parameter.value)
            elif parameter.name == 'frontier_replan_period_sec':
                self.frontier_replan_period_sec = max(0.5, float(parameter.value))
            elif parameter.name == 'frontier_goal_tolerance_m':
                self.frontier_goal_tolerance_m = max(0.03, float(parameter.value))
            elif parameter.name == 'frontier_waypoint_spacing_m':
                self.frontier_waypoint_spacing_m = max(
                    self.frontier_goal_tolerance_m,
                    float(parameter.value),
                )
            elif parameter.name == 'frontier_min_clearance_m':
                self.frontier_min_clearance_m = max(0.0, float(parameter.value))
            elif parameter.name == 'frontier_free_threshold':
                self.frontier_free_threshold = int(parameter.value)
            elif parameter.name == 'frontier_occupied_threshold':
                self.frontier_occupied_threshold = int(parameter.value)
            elif parameter.name == 'frontier_suppression_duration_sec':
                self.frontier_suppression_duration_sec = max(0.0, float(parameter.value))
            elif parameter.name == 'frontier_suppression_radius_m':
                self.frontier_suppression_radius_m = max(0.0, float(parameter.value))
            elif parameter.name == 'frontier_blocked_clearance_margin_m':
                self.frontier_blocked_clearance_margin_m = max(
                    0.0,
                    float(parameter.value),
                )
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

    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg
        self.latest_map_time = self.get_clock().now()

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

        if self.exploration_mode == 'frontier':
            frontier_cmd = self.frontier_control()
            if frontier_cmd is not None:
                self.cmd_pub.publish(frontier_cmd)
                return
            if not self.reactive_fallback:
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

    def frontier_control(self):
        cmd = Twist()

        if self.latest_map is None or self.latest_map_time is None:
            self.get_logger().warn(
                'No /map received yet. Waiting before frontier exploration.',
                throttle_duration_sec=2.0,
            )
            return None

        map_age_sec = (self.get_clock().now() - self.latest_map_time).nanoseconds * 1e-9
        if map_age_sec > self.map_timeout_sec:
            self.get_logger().warn(
                f'Map timeout ({map_age_sec:.2f}s old). Holding position.',
                throttle_duration_sec=2.0,
            )
            return cmd

        if self.frontier_replan_due():
            self.plan_frontier_target()

        if self.current_frontier is None:
            self.get_logger().info(
                'No reachable frontier found. Exploration appears complete or blocked.',
                throttle_duration_sec=4.0,
            )
            return cmd

        while self.current_frontier is not None:
            waypoint = self.active_frontier_waypoint()
            if waypoint is None:
                self.clear_current_frontier()
                return cmd

            body_vector = self.map_target_vector_in_body(waypoint[0], waypoint[1])
            if body_vector is None:
                return None

            body_x, body_y = body_vector
            distance_m = math.hypot(body_x, body_y)
            if distance_m < self.frontier_goal_tolerance_m:
                if self.advance_frontier_waypoint():
                    continue
                self.suppress_current_frontier('reached')
                return cmd

            body_angle = math.atan2(body_y, body_x)
            scan_clearance = self.min_scan_range_near_body_angle(
                body_angle,
                self.direction_window_rad,
            )
            blocked_clearance_m = (
                self.obstacle_stop_distance_m
                + self.frontier_blocked_clearance_margin_m
            )
            if scan_clearance is not None and scan_clearance <= blocked_clearance_m:
                self.suppress_current_frontier('scan_blocked')
                return cmd

            speed = self.speed_for_clearance(
                scan_clearance if scan_clearance is not None else self.obstacle_slow_distance_m
            )
            if self.crab_motion:
                cmd.linear.x = speed * (body_x / distance_m)
                cmd.linear.y = speed * (body_y / distance_m)
                cmd.linear.y = clamp(
                    cmd.linear.y,
                    -self.max_lateral_speed_mps,
                    self.max_lateral_speed_mps,
                )
                if not self.reverse_allowed and cmd.linear.x < 0.0:
                    if self.advance_frontier_waypoint():
                        continue
                    self.suppress_current_frontier('requires_reverse')
                    return cmd
            else:
                cmd.linear.x = speed * max(0.0, math.cos(body_angle))
                cmd.angular.z = clamp(
                    self.turn_gain * body_angle,
                    -self.max_turn_rate_rad_s,
                    self.max_turn_rate_rad_s,
                )

            return cmd

        return cmd

    def plan_frontier_target(self):
        self.prune_suppressed_frontiers()
        self.current_frontier = self.find_frontier_target()
        self.current_path_index = 0
        self.last_frontier_plan_time = self.get_clock().now()
        if self.current_frontier is not None:
            self.get_logger().info(
                f'New {self.search_strategy.upper()} frontier target: '
                f'({self.current_frontier.x_m:.2f}, {self.current_frontier.y_m:.2f}) '
                f'after searching {self.current_frontier.searched_cells} cells '
                f'with {len(self.current_frontier.path)} waypoints.'
            )

    def advance_frontier_waypoint(self) -> bool:
        if self.current_frontier is None:
            return False
        next_index = self.current_path_index + 1
        if next_index >= len(self.current_frontier.path):
            return False
        self.current_path_index = next_index
        return True

    def clear_current_frontier(self):
        self.current_frontier = None
        self.current_path_index = 0
        self.last_frontier_plan_time = self.get_clock().now()

    def suppress_current_frontier(self, reason: str):
        frontier = self.current_frontier
        if frontier is None:
            self.clear_current_frontier()
            return

        self.prune_suppressed_frontiers()
        duration_sec = self.frontier_suppression_duration_sec
        suppressed_count = 0
        if duration_sec > 0.0 and self.frontier_suppression_radius_m > 0.0:
            expires_at_sec = self.now_sec() + duration_sec
            suppressed_count += self.add_suppressed_frontier_area(
                frontier.x_m,
                frontier.y_m,
                expires_at_sec,
                reason,
            )
            for x_m, y_m in frontier.path[self.current_path_index:]:
                suppressed_count += self.add_suppressed_frontier_area(
                    x_m,
                    y_m,
                    expires_at_sec,
                    reason,
                )
            self.get_logger().info(
                f'Suppressing frontier path near ({frontier.x_m:.2f}, '
                f'{frontier.y_m:.2f}) for {duration_sec:.1f}s after {reason} '
                f'({suppressed_count} map areas).',
                throttle_duration_sec=2.0,
            )
        self.clear_current_frontier()

    def add_suppressed_frontier_area(
        self,
        x_m: float,
        y_m: float,
        expires_at_sec: float,
        reason: str,
    ) -> int:
        dedupe_radius_m = max(0.05, self.frontier_suppression_radius_m * 0.5)
        dedupe_radius_sq = dedupe_radius_m * dedupe_radius_m
        for frontier in self.suppressed_frontiers:
            dx_m = x_m - frontier.x_m
            dy_m = y_m - frontier.y_m
            if dx_m * dx_m + dy_m * dy_m <= dedupe_radius_sq:
                return 0

        self.suppressed_frontiers.append(
            SuppressedFrontier(
                x_m=x_m,
                y_m=y_m,
                expires_at_sec=expires_at_sec,
                reason=reason,
            )
        )
        return 1

    def active_frontier_waypoint(self):
        if self.current_frontier is None or not self.current_frontier.path:
            return None
        index = min(self.current_path_index, len(self.current_frontier.path) - 1)
        return self.current_frontier.path[index]

    def frontier_replan_due(self) -> bool:
        if self.last_frontier_plan_time is None:
            return True
        elapsed = (self.get_clock().now() - self.last_frontier_plan_time).nanoseconds * 1e-9
        return elapsed >= self.frontier_replan_period_sec

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def prune_suppressed_frontiers(self):
        if not self.suppressed_frontiers:
            return
        now_sec = self.now_sec()
        self.suppressed_frontiers = [
            frontier
            for frontier in self.suppressed_frontiers
            if frontier.expires_at_sec > now_sec
        ]

    def frontier_is_suppressed(self, x_m: float, y_m: float) -> bool:
        if not self.suppressed_frontiers or self.frontier_suppression_radius_m <= 0.0:
            return False
        radius_sq = self.frontier_suppression_radius_m * self.frontier_suppression_radius_m
        for frontier in self.suppressed_frontiers:
            dx_m = x_m - frontier.x_m
            dy_m = y_m - frontier.y_m
            if dx_m * dx_m + dy_m * dy_m <= radius_sq:
                return True
        return False

    def frontier_path_is_suppressed(
        self,
        path: tuple[tuple[float, float], ...],
    ) -> bool:
        return any(self.frontier_is_suppressed(x_m, y_m) for x_m, y_m in path)

    def map_target_vector_in_body(self, target_x_m: float, target_y_m: float):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'Could not transform {self.base_frame} to {self.map_frame}: {exc}.',
                throttle_duration_sec=2.0,
            )
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        robot_yaw = quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w)
        dx_map = target_x_m - translation.x
        dy_map = target_y_m - translation.y
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        return (
            cos_yaw * dx_map + sin_yaw * dy_map,
            -sin_yaw * dx_map + cos_yaw * dy_map,
        )

    def min_scan_range_near_body_angle(self, body_angle_rad: float, window_rad: float):
        if self.latest_scan is None:
            return None

        yaw_offset = self.scan_to_body_yaw(self.latest_scan)
        best = None
        for index, raw_range in enumerate(self.latest_scan.ranges):
            value = self.sanitize_range(
                raw_range,
                self.latest_scan.range_min,
                self.latest_scan.range_max,
            )
            if value is None:
                continue

            scan_angle = self.latest_scan.angle_min + index * self.latest_scan.angle_increment
            beam_body_angle = scan_angle + yaw_offset
            if abs(normalize_angle(beam_body_angle - body_angle_rad)) <= window_rad:
                best = value if best is None else min(best, value)

        return best

    def scan_to_body_yaw(self, scan: LaserScan) -> float:
        if not self.use_tf_for_scan_frame:
            return self.scan_yaw_offset_rad

        scan_frame = scan.header.frame_id.strip()
        if not scan_frame:
            return self.scan_yaw_offset_rad

        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                scan_frame,
                Time(),
            )
        except TransformException:
            return self.scan_yaw_offset_rad

        rotation = transform.transform.rotation
        return quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w)

    def find_frontier_target(self):
        grid = self.latest_map
        if grid is None or grid.info.width <= 0 or grid.info.height <= 0:
            return None

        try:
            transform = self.tf_buffer.lookup_transform(
                grid.header.frame_id or self.map_frame,
                self.base_frame,
                Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'Cannot find robot pose in map for frontier search: {exc}.',
                throttle_duration_sec=2.0,
            )
            return None

        start = self.world_to_grid(
            grid,
            transform.transform.translation.x,
            transform.transform.translation.y,
        )
        if start is None:
            return None

        width = grid.info.width
        height = grid.info.height
        start_index = self.grid_index(start[0], start[1], width)
        if not self.is_traversable(grid, start_index):
            nearest = self.nearest_traversable_cell(grid, start[0], start[1])
            if nearest is None:
                return None
            start_index = self.grid_index(nearest[0], nearest[1], width)

        visited = bytearray(width * height)
        parent = [-1] * (width * height)
        pending = deque([start_index])
        visited[start_index] = 1
        parent[start_index] = start_index
        searched = 0

        while pending:
            current = pending.popleft() if self.search_strategy == 'bfs' else pending.pop()
            searched += 1

            if self.is_frontier_cell(grid, current):
                grid_x = current % width
                grid_y = current // width
                world_x, world_y = self.grid_to_world(grid, grid_x, grid_y)
                if not self.frontier_is_suppressed(world_x, world_y):
                    path = self.reconstruct_world_path(grid, parent, start_index, current)
                    if not self.frontier_path_is_suppressed(path):
                        return FrontierTarget(
                            x_m=world_x,
                            y_m=world_y,
                            grid_x=grid_x,
                            grid_y=grid_y,
                            searched_cells=searched,
                            path=path,
                        )

            for neighbor in self.neighbor_indices(current, width, height):
                if visited[neighbor] or not self.is_traversable(grid, neighbor):
                    continue
                visited[neighbor] = 1
                parent[neighbor] = current
                pending.append(neighbor)

        return None

    def reconstruct_world_path(
        self,
        grid: OccupancyGrid,
        parent: list[int],
        start_index: int,
        goal_index: int,
    ):
        path_indices = []
        current = goal_index
        while current != start_index and current >= 0:
            path_indices.append(current)
            current = parent[current]
        path_indices.reverse()

        if not path_indices:
            goal_x = goal_index % grid.info.width
            goal_y = goal_index // grid.info.width
            return (self.grid_to_world(grid, goal_x, goal_y),)

        waypoints = []
        last_waypoint = None
        for index in path_indices:
            grid_x = index % grid.info.width
            grid_y = index // grid.info.width
            world = self.grid_to_world(grid, grid_x, grid_y)
            if last_waypoint is None:
                waypoints.append(world)
                last_waypoint = world
                continue

            if math.hypot(world[0] - last_waypoint[0], world[1] - last_waypoint[1]) >= (
                self.frontier_waypoint_spacing_m
            ):
                waypoints.append(world)
                last_waypoint = world

        final_x = goal_index % grid.info.width
        final_y = goal_index // grid.info.width
        final_world = self.grid_to_world(grid, final_x, final_y)
        if not waypoints or waypoints[-1] != final_world:
            waypoints.append(final_world)
        return tuple(waypoints)

    def nearest_traversable_cell(self, grid: OccupancyGrid, start_x: int, start_y: int):
        width = grid.info.width
        height = grid.info.height
        start_index = self.grid_index(start_x, start_y, width)
        visited = bytearray(width * height)
        pending = deque([start_index])
        visited[start_index] = 1
        while pending:
            current = pending.popleft()
            if self.is_traversable(grid, current):
                return current % width, current // width
            for neighbor in self.neighbor_indices(current, width, height):
                if not visited[neighbor]:
                    visited[neighbor] = 1
                    pending.append(neighbor)
        return None

    def is_frontier_cell(self, grid: OccupancyGrid, index: int) -> bool:
        if not self.is_traversable(grid, index):
            return False
        width = grid.info.width
        height = grid.info.height
        for neighbor in self.neighbor_indices(index, width, height):
            if grid.data[neighbor] < 0:
                return True
        return False

    def is_traversable(self, grid: OccupancyGrid, index: int) -> bool:
        value = grid.data[index]
        if value < 0 or value > self.frontier_free_threshold:
            return False

        clearance_cells = int(math.ceil(self.frontier_min_clearance_m / grid.info.resolution))
        if clearance_cells <= 0:
            return True

        width = grid.info.width
        height = grid.info.height
        center_x = index % width
        center_y = index // width
        for y_value in range(
            max(0, center_y - clearance_cells),
            min(height, center_y + clearance_cells + 1),
        ):
            for x_value in range(
                max(0, center_x - clearance_cells),
                min(width, center_x + clearance_cells + 1),
            ):
                neighbor_value = grid.data[self.grid_index(x_value, y_value, width)]
                if neighbor_value >= self.frontier_occupied_threshold:
                    return False
        return True

    def neighbor_indices(self, index: int, width: int, height: int):
        x_value = index % width
        y_value = index // width
        for dy_value, dx_value in (
            (0, 1),
            (1, 0),
            (0, -1),
            (-1, 0),
            (1, 1),
            (1, -1),
            (-1, -1),
            (-1, 1),
        ):
            nx = x_value + dx_value
            ny = y_value + dy_value
            if 0 <= nx < width and 0 <= ny < height:
                yield self.grid_index(nx, ny, width)

    def grid_index(self, x_value: int, y_value: int, width: int) -> int:
        return y_value * width + x_value

    def world_to_grid(self, grid: OccupancyGrid, x_m: float, y_m: float):
        origin = grid.info.origin.position
        resolution = grid.info.resolution
        grid_x = int((x_m - origin.x) / resolution)
        grid_y = int((y_m - origin.y) / resolution)
        if 0 <= grid_x < grid.info.width and 0 <= grid_y < grid.info.height:
            return grid_x, grid_y
        return None

    def grid_to_world(self, grid: OccupancyGrid, grid_x: int, grid_y: int):
        origin = grid.info.origin.position
        resolution = grid.info.resolution
        return (
            origin.x + (grid_x + 0.5) * resolution,
            origin.y + (grid_y + 0.5) * resolution,
        )

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
