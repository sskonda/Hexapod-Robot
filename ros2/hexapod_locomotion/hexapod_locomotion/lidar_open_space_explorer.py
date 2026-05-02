#!/usr/bin/env python3
"""Simple LiDAR reactive explorer.

This node is intentionally small: it subscribes to a 2D LaserScan, chooses the
widest/deepest safe direction, and publishes cmd_vel for slow exploration while
slam_toolbox builds the map.
"""

import json
import math
import heapq
from collections import deque
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Point, Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


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
    target_reason: str = 'frontier'
    frontier_cells: tuple[int, ...] = ()
    frontier_area_cells: int = 0
    clearance_m: float = 0.0
    path_length_m: float = 0.0
    unknown_visible_cells: int = 0
    score: float = 0.0


@dataclass(frozen=True)
class SuppressedFrontier:
    x_m: float
    y_m: float
    expires_at_sec: float
    reason: str
    free_count: int = -1
    unknown_count: int = -1
    occupied_count: int = -1


@dataclass(frozen=True)
class RecentFrontierPath:
    path: tuple[tuple[float, float], ...]
    recorded_at_sec: float
    reason: str


@dataclass
class PlannerGrid:
    width: int
    height: int
    resolution: float
    free: bytearray
    danger: bytearray
    path_safe: bytearray
    goal_safe: bytearray
    safe: bytearray
    clearance_m: list[float]


@dataclass(frozen=True)
class ViewpointCandidate:
    index: int
    parent_index: int
    distance_to_frontier_cells: float
    path_length_m: float
    clearance_m: float
    unknown_visible_cells: int
    score: float


class LidarOpenSpaceExplorer(Node):
    def __init__(self):
        super().__init__('lidar_open_space_explorer')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('target_marker_topic', '/explorer/targets')
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

        # Conservative defaults for live SLAM on a walking hexapod.
        # Fast yaw/forward motion near walls makes scan matching and odometry
        # disagree, so keep motion slow and predictable by default.
        self.declare_parameter('max_speed_mps', 0.018)
        self.declare_parameter('min_speed_mps', 0.008)
        self.declare_parameter('max_lateral_speed_mps', 0.012)
        self.declare_parameter('max_turn_rate_rad_s', 0.040)
        self.declare_parameter('turn_gain', 0.30)
        self.declare_parameter('crab_motion', True)

        self.declare_parameter('obstacle_stop_distance_m', 0.40)
        self.declare_parameter('obstacle_slow_distance_m', 0.70)
        self.declare_parameter('side_stop_distance_m', 0.45)
        self.declare_parameter('side_check_angle_deg', 85.0)
        self.declare_parameter('side_check_window_deg', 28.0)
        self.declare_parameter('wall_escape_enter_distance_m', 0.45)
        self.declare_parameter('wall_escape_exit_distance_m', 0.60)
        self.declare_parameter('wall_escape_exit_hold_sec', 0.75)
        self.declare_parameter('wall_escape_lateral_speed_mps', 0.012)
        self.declare_parameter('wall_escape_turn_rate_rad_s', 0.020)
        self.declare_parameter('wall_escape_max_duration_sec', 5.0)
        self.declare_parameter('desired_clearance_m', 0.45)
        self.declare_parameter('minimum_direction_range_m', 0.32)
        self.declare_parameter('max_usable_range_m', 4.0)

        self.declare_parameter('candidate_angle_min_deg', -150.0)
        self.declare_parameter('candidate_angle_max_deg', 150.0)
        self.declare_parameter('direction_window_deg', 18.0)
        self.declare_parameter('candidate_stride_deg', 5.0)
        self.declare_parameter('forward_bias_weight', 0.45)
        self.declare_parameter('distance_weight', 1.0)
        self.declare_parameter('clearance_angle_weight', 1.4)
        self.declare_parameter('reverse_allowed', False)
        self.declare_parameter('frontier_replan_period_sec', 5.0)
        self.declare_parameter('frontier_goal_tolerance_m', 0.18)
        self.declare_parameter('frontier_waypoint_spacing_m', 0.25)
        self.declare_parameter('frontier_min_clearance_m', 0.15)
        self.declare_parameter('frontier_free_threshold', 25)
        self.declare_parameter('frontier_occupied_threshold', 65)
        self.declare_parameter('robot_radius_m', 0.30)
        self.declare_parameter('planner_safety_margin_m', 0.10)
        self.declare_parameter('path_clearance_m', 0.40)
        self.declare_parameter('goal_clearance_m', 0.33)
        self.declare_parameter('frontier_unknown_margin_cells', 2)
        self.declare_parameter('frontier_min_area_cells', 5)
        self.declare_parameter('min_frontier_cluster_size', 5)
        self.declare_parameter('frontier_goal_projection_radius_m', 0.90)
        self.declare_parameter('frontier_standoff_distance_m', 0.40)
        self.declare_parameter('max_projection_attempts_per_frontier', 50)
        self.declare_parameter('unknown_visibility_radius_m', 1.20)
        self.declare_parameter('unknown_visibility_min_cells', 1)
        self.declare_parameter('visited_radius_m', 0.25)
        self.declare_parameter('unvisited_goal_min_distance_m', 0.35)
        self.declare_parameter('unvisited_goal_max_distance_m', 3.0)
        self.declare_parameter('unvisited_goal_tolerance_m', 0.18)
        self.declare_parameter('suppress_only_after_motion_failure', True)
        self.declare_parameter('frontier_failure_memory_enabled', False)
        self.declare_parameter('frontier_suppression_duration_sec', 45.0)
        self.declare_parameter('frontier_suppression_radius_m', 0.45)
        self.declare_parameter('frontier_blocked_clearance_margin_m', 0.05)
        self.declare_parameter('frontier_progress_timeout_sec', 10.0)
        self.declare_parameter('frontier_progress_epsilon_m', 0.08)
        self.declare_parameter('recent_path_memory_size', 2)
        self.declare_parameter('recent_path_overlap_fraction', 0.6)
        self.declare_parameter('recent_path_point_tolerance_m', 0.18)
        self.declare_parameter('publish_target_markers', True)
        self.declare_parameter('target_marker_scale_m', 0.12)
        self.declare_parameter('bug_recovery_enabled', False)
        self.declare_parameter('bug_wall_side', 'auto')
        self.declare_parameter('bug_forward_speed_mps', 0.012)
        self.declare_parameter('bug_lateral_gain', 0.8)
        self.declare_parameter('bug_desired_wall_distance_m', 0.40)
        self.declare_parameter('bug_release_clearance_m', 0.55)
        self.declare_parameter('bug_release_hold_sec', 1.0)
        self.declare_parameter('bug_min_duration_sec', 3.0)
        self.declare_parameter('bug_max_duration_sec', 15.0)
        self.declare_parameter('bug_front_angle_deg', 30.0)
        self.declare_parameter('bug_side_angle_deg', 75.0)
        self.declare_parameter('qr_spin_enabled', True)
        self.declare_parameter('qr_spin_cmd_angular_z_rad_s', 0.08)
        self.declare_parameter('qr_spin_step_angle_deg', 25.0)
        self.declare_parameter('qr_spin_step_duration_sec', 0.0)
        self.declare_parameter('qr_spin_pause_duration_sec', 0.6)
        self.declare_parameter('qr_spin_total_angle_deg', 360.0)
        self.declare_parameter('qr_spin_cooldown_sec', 60.0)
        self.declare_parameter('qr_spin_max_count', 2)
        self.declare_parameter('qr_spin_front_dead_end_distance_m', 0.55)
        self.declare_parameter('qr_spin_side_dead_end_distance_m', 0.55)
        self.declare_parameter('qr_spin_qr_text_topic', '/qr_code/text')
        self.declare_parameter('qr_spin_pause_slam', True)
        self.declare_parameter('qr_spin_slam_node_name', '/slam_toolbox')
        self.declare_parameter('qr_spin_stop_on_qr_detected', True)
        self.declare_parameter('mission_return_home_after_qr_spin', True)
        self.declare_parameter('mission_home_marker_state_topic', '/qr_code/marker_state')
        self.declare_parameter('mission_home_marker_name', '')
        self.declare_parameter('mission_home_standoff_min_m', 0.35)
        self.declare_parameter('mission_home_standoff_max_m', 0.85)
        self.declare_parameter('mission_home_goal_tolerance_m', 0.22)
        self.declare_parameter('mission_return_home_replan_period_sec', 2.0)
        self.declare_parameter('mission_stop_when_home_reached', True)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.target_marker_topic = str(self.get_parameter('target_marker_topic').value)
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
        self.frontier_progress_best_distance_m = None
        self.frontier_progress_last_time_sec = 0.0
        self.frontier_progress_waypoint_index = -1
        self.suppressed_frontiers = []
        self.rejected_frontiers = []
        self.recent_frontier_paths = []
        self.visited_cells = bytearray()
        self.visited_map_signature = None
        self.bug_active = False
        self.bug_wall_side = 1
        self.bug_started_at_sec = 0.0
        self.bug_clear_since_sec = None
        self.bug_timed_out = False
        self.wall_escape_active = False
        self.wall_escape_side = 0
        self.wall_escape_started_sec = 0.0
        self.wall_escape_clear_since_sec = None
        self.mission_mode = 'explore'
        self.home_marker_name = None
        self.home_marker_xy = None
        self.home_target = None
        self.home_path_index = 0
        self.last_home_plan_time = None
        self.last_qr_text = ''
        self.last_qr_seen_sec = None
        self.qr_spin_active = False
        self.qr_spin_phase = 'idle'
        self.qr_spin_started_sec = 0.0
        self.qr_spin_phase_started_sec = 0.0
        self.qr_spin_accumulated_yaw_rad = 0.0
        self.qr_spin_accumulated_angle_rad = 0.0
        self.qr_spin_last_start_sec = -math.inf
        self.qr_spin_count = 0
        self.qr_spin_slam_was_paused = False
        self.slam_transition_future = None
        self.qr_spin_waiting_for_slam_pause = False
        self.qr_spin_slam_pause_requested_sec = 0.0

        self._load_parameters()

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.target_marker_topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, 10)
        self.create_subscription(String, self.qr_spin_qr_text_topic, self.qr_text_callback, 10)
        marker_state_qos = QoSProfile(depth=1)
        marker_state_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String,
            self.mission_home_marker_state_topic,
            self.marker_state_callback,
            marker_state_qos,
        )
        self.slam_change_state_client = self.create_client(
            ChangeState,
            self.slam_change_state_service_name(),
        )
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
        self.side_stop_distance_m = max(
            self.obstacle_stop_distance_m,
            float(self.get_parameter('side_stop_distance_m').value),
        )
        self.side_check_angle_rad = math.radians(
            max(5.0, min(120.0, float(self.get_parameter('side_check_angle_deg').value)))
        )
        self.side_check_window_rad = math.radians(
            max(1.0, min(60.0, float(self.get_parameter('side_check_window_deg').value)))
        )
        self.wall_escape_enter_distance_m = max(
            0.0,
            float(self.get_parameter('wall_escape_enter_distance_m').value),
        )
        self.wall_escape_exit_distance_m = max(
            self.wall_escape_enter_distance_m,
            float(self.get_parameter('wall_escape_exit_distance_m').value),
        )
        self.wall_escape_lateral_speed_mps = max(
            0.0,
            float(self.get_parameter('wall_escape_lateral_speed_mps').value),
        )
        self.wall_escape_turn_rate_rad_s = max(
            0.0,
            float(self.get_parameter('wall_escape_turn_rate_rad_s').value),
        )
        self.wall_escape_max_duration_sec = max(
            0.0,
            float(self.get_parameter('wall_escape_max_duration_sec').value),
        )
        self.wall_escape_exit_hold_sec = max(
            0.0,
            float(self.get_parameter('wall_escape_exit_hold_sec').value),
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
        self.robot_radius_m = max(0.0, float(self.get_parameter('robot_radius_m').value))
        self.planner_safety_margin_m = max(
            0.0,
            float(self.get_parameter('planner_safety_margin_m').value),
        )
        self.path_clearance_m = max(
            self.robot_radius_m,
            float(self.get_parameter('path_clearance_m').value),
        )
        self.goal_clearance_m = max(
            self.robot_radius_m,
            float(self.get_parameter('goal_clearance_m').value),
        )
        self.frontier_unknown_margin_cells = max(
            0,
            int(self.get_parameter('frontier_unknown_margin_cells').value),
        )
        legacy_min_frontier_cells = int(
            self.get_parameter('frontier_min_area_cells').value
        )
        cluster_min_frontier_cells = int(
            self.get_parameter('min_frontier_cluster_size').value
        )
        self.frontier_min_area_cells = max(
            1,
            cluster_min_frontier_cells
            if cluster_min_frontier_cells != 5
            else legacy_min_frontier_cells,
        )
        self.frontier_goal_projection_radius_m = max(
            self.frontier_goal_tolerance_m,
            float(self.get_parameter('frontier_goal_projection_radius_m').value),
        )
        self.frontier_standoff_distance_m = max(
            0.0,
            float(self.get_parameter('frontier_standoff_distance_m').value),
        )
        self.max_projection_attempts_per_frontier = max(
            1,
            int(self.get_parameter('max_projection_attempts_per_frontier').value),
        )
        self.unknown_visibility_radius_m = max(
            self.frontier_goal_tolerance_m,
            float(self.get_parameter('unknown_visibility_radius_m').value),
        )
        self.unknown_visibility_min_cells = max(
            0,
            int(self.get_parameter('unknown_visibility_min_cells').value),
        )
        self.visited_radius_m = max(
            0.0,
            float(self.get_parameter('visited_radius_m').value),
        )
        self.unvisited_goal_min_distance_m = max(
            0.0,
            float(self.get_parameter('unvisited_goal_min_distance_m').value),
        )
        self.unvisited_goal_max_distance_m = max(
            self.unvisited_goal_min_distance_m,
            float(self.get_parameter('unvisited_goal_max_distance_m').value),
        )
        self.unvisited_goal_tolerance_m = max(
            0.03,
            float(self.get_parameter('unvisited_goal_tolerance_m').value),
        )
        self.suppress_only_after_motion_failure = as_bool(
            self.get_parameter('suppress_only_after_motion_failure').value
        )
        self.frontier_failure_memory_enabled = as_bool(
            self.get_parameter('frontier_failure_memory_enabled').value
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
        self.frontier_progress_timeout_sec = max(
            1.0,
            float(self.get_parameter('frontier_progress_timeout_sec').value),
        )
        self.frontier_progress_epsilon_m = max(
            0.0,
            float(self.get_parameter('frontier_progress_epsilon_m').value),
        )
        self.recent_path_memory_size = max(
            0,
            int(self.get_parameter('recent_path_memory_size').value),
        )
        self.recent_path_overlap_fraction = clamp(
            float(self.get_parameter('recent_path_overlap_fraction').value),
            0.0,
            1.0,
        )
        self.recent_path_point_tolerance_m = max(
            0.01,
            float(self.get_parameter('recent_path_point_tolerance_m').value),
        )
        self.trim_recent_frontier_paths()
        self.publish_target_markers = as_bool(
            self.get_parameter('publish_target_markers').value
        )
        self.target_marker_scale_m = max(
            0.02,
            float(self.get_parameter('target_marker_scale_m').value),
        )
        self.bug_recovery_enabled = as_bool(
            self.get_parameter('bug_recovery_enabled').value
        )
        self.bug_wall_side_mode = self.normalized_bug_wall_side(
            str(self.get_parameter('bug_wall_side').value)
        )
        self.bug_forward_speed_mps = max(
            0.0,
            float(self.get_parameter('bug_forward_speed_mps').value),
        )
        self.bug_lateral_gain = max(0.0, float(self.get_parameter('bug_lateral_gain').value))
        self.bug_desired_wall_distance_m = max(
            self.obstacle_stop_distance_m,
            float(self.get_parameter('bug_desired_wall_distance_m').value),
        )
        self.bug_release_clearance_m = max(
            self.obstacle_stop_distance_m,
            float(self.get_parameter('bug_release_clearance_m').value),
        )
        self.bug_release_hold_sec = max(
            0.0,
            float(self.get_parameter('bug_release_hold_sec').value),
        )
        self.bug_min_duration_sec = max(
            0.0,
            float(self.get_parameter('bug_min_duration_sec').value),
        )
        self.bug_max_duration_sec = max(
            0.0,
            float(self.get_parameter('bug_max_duration_sec').value),
        )
        self.bug_front_angle_rad = math.radians(
            max(1.0, float(self.get_parameter('bug_front_angle_deg').value))
        )
        self.bug_side_angle_rad = math.radians(
            max(10.0, min(120.0, float(self.get_parameter('bug_side_angle_deg').value)))
        )
        self.qr_spin_enabled = as_bool(self.get_parameter('qr_spin_enabled').value)
        self.qr_spin_cmd_angular_z_rad_s = max(
            0.01,
            abs(float(self.get_parameter('qr_spin_cmd_angular_z_rad_s').value)),
        )
        self.qr_spin_step_angle_rad = math.radians(
            max(1.0, float(self.get_parameter('qr_spin_step_angle_deg').value))
        )
        self.qr_spin_step_duration_sec = max(
            0.0,
            float(self.get_parameter('qr_spin_step_duration_sec').value),
        )
        self.qr_spin_pause_duration_sec = max(
            0.0,
            float(self.get_parameter('qr_spin_pause_duration_sec').value),
        )
        self.qr_spin_total_angle_rad = math.radians(
            max(1.0, float(self.get_parameter('qr_spin_total_angle_deg').value))
        )
        self.qr_spin_cooldown_sec = max(
            0.0,
            float(self.get_parameter('qr_spin_cooldown_sec').value),
        )
        self.qr_spin_max_count = max(0, int(self.get_parameter('qr_spin_max_count').value))
        self.qr_spin_front_dead_end_distance_m = max(
            0.0,
            float(self.get_parameter('qr_spin_front_dead_end_distance_m').value),
        )
        self.qr_spin_side_dead_end_distance_m = max(
            0.0,
            float(self.get_parameter('qr_spin_side_dead_end_distance_m').value),
        )
        self.qr_spin_qr_text_topic = str(
            self.get_parameter('qr_spin_qr_text_topic').value
        )
        self.qr_spin_pause_slam = as_bool(self.get_parameter('qr_spin_pause_slam').value)
        self.qr_spin_slam_node_name = str(
            self.get_parameter('qr_spin_slam_node_name').value
        )
        self.qr_spin_stop_on_qr_detected = as_bool(
            self.get_parameter('qr_spin_stop_on_qr_detected').value
        )
        self.mission_return_home_after_qr_spin = as_bool(
            self.get_parameter('mission_return_home_after_qr_spin').value
        )
        self.mission_home_marker_state_topic = str(
            self.get_parameter('mission_home_marker_state_topic').value
        )
        self.mission_home_marker_name = str(
            self.get_parameter('mission_home_marker_name').value
        ).strip().lower()
        self.mission_home_standoff_min_m = max(
            0.0,
            float(self.get_parameter('mission_home_standoff_min_m').value),
        )
        self.mission_home_standoff_max_m = max(
            self.mission_home_standoff_min_m,
            float(self.get_parameter('mission_home_standoff_max_m').value),
        )
        self.mission_home_goal_tolerance_m = max(
            0.03,
            float(self.get_parameter('mission_home_goal_tolerance_m').value),
        )
        self.mission_return_home_replan_period_sec = max(
            0.5,
            float(self.get_parameter('mission_return_home_replan_period_sec').value),
        )
        self.mission_stop_when_home_reached = as_bool(
            self.get_parameter('mission_stop_when_home_reached').value
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

    def normalized_bug_wall_side(self, value: str) -> str:
        side = value.strip().lower()
        if side not in ('auto', 'left', 'right'):
            self.get_logger().warn(
                f'Unsupported bug_wall_side "{value}". Falling back to auto.',
                throttle_duration_sec=2.0,
            )
            return 'auto'
        return side

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
            elif parameter.name == 'robot_radius_m':
                self.robot_radius_m = max(0.0, float(parameter.value))
            elif parameter.name == 'planner_safety_margin_m':
                self.planner_safety_margin_m = max(0.0, float(parameter.value))
            elif parameter.name == 'path_clearance_m':
                self.path_clearance_m = max(self.robot_radius_m, float(parameter.value))
            elif parameter.name == 'goal_clearance_m':
                self.goal_clearance_m = max(self.robot_radius_m, float(parameter.value))
            elif parameter.name == 'frontier_unknown_margin_cells':
                self.frontier_unknown_margin_cells = max(0, int(parameter.value))
            elif parameter.name == 'frontier_min_area_cells':
                self.frontier_min_area_cells = max(1, int(parameter.value))
            elif parameter.name == 'min_frontier_cluster_size':
                self.frontier_min_area_cells = max(1, int(parameter.value))
            elif parameter.name == 'frontier_goal_projection_radius_m':
                self.frontier_goal_projection_radius_m = max(
                    self.frontier_goal_tolerance_m,
                    float(parameter.value),
                )
            elif parameter.name == 'frontier_standoff_distance_m':
                self.frontier_standoff_distance_m = max(0.0, float(parameter.value))
            elif parameter.name == 'max_projection_attempts_per_frontier':
                self.max_projection_attempts_per_frontier = max(1, int(parameter.value))
            elif parameter.name == 'unknown_visibility_radius_m':
                self.unknown_visibility_radius_m = max(
                    self.frontier_goal_tolerance_m,
                    float(parameter.value),
                )
            elif parameter.name == 'unknown_visibility_min_cells':
                self.unknown_visibility_min_cells = max(0, int(parameter.value))
            elif parameter.name == 'visited_radius_m':
                self.visited_radius_m = max(0.0, float(parameter.value))
            elif parameter.name == 'unvisited_goal_min_distance_m':
                self.unvisited_goal_min_distance_m = max(0.0, float(parameter.value))
                self.unvisited_goal_max_distance_m = max(
                    self.unvisited_goal_min_distance_m,
                    self.unvisited_goal_max_distance_m,
                )
            elif parameter.name == 'unvisited_goal_max_distance_m':
                self.unvisited_goal_max_distance_m = max(
                    self.unvisited_goal_min_distance_m,
                    float(parameter.value),
                )
            elif parameter.name == 'unvisited_goal_tolerance_m':
                self.unvisited_goal_tolerance_m = max(0.03, float(parameter.value))
            elif parameter.name == 'suppress_only_after_motion_failure':
                self.suppress_only_after_motion_failure = as_bool(parameter.value)
            elif parameter.name == 'frontier_failure_memory_enabled':
                self.frontier_failure_memory_enabled = as_bool(parameter.value)
                if not self.frontier_failure_memory_enabled:
                    self.suppressed_frontiers = []
                    self.rejected_frontiers = []
            elif parameter.name == 'frontier_suppression_duration_sec':
                self.frontier_suppression_duration_sec = max(0.0, float(parameter.value))
            elif parameter.name == 'frontier_suppression_radius_m':
                self.frontier_suppression_radius_m = max(0.0, float(parameter.value))
            elif parameter.name == 'frontier_blocked_clearance_margin_m':
                self.frontier_blocked_clearance_margin_m = max(
                    0.0,
                    float(parameter.value),
                )
            elif parameter.name == 'frontier_progress_timeout_sec':
                self.frontier_progress_timeout_sec = max(1.0, float(parameter.value))
            elif parameter.name == 'frontier_progress_epsilon_m':
                self.frontier_progress_epsilon_m = max(0.0, float(parameter.value))
            elif parameter.name == 'recent_path_memory_size':
                self.recent_path_memory_size = max(0, int(parameter.value))
                self.trim_recent_frontier_paths()
            elif parameter.name == 'recent_path_overlap_fraction':
                self.recent_path_overlap_fraction = clamp(float(parameter.value), 0.0, 1.0)
            elif parameter.name == 'recent_path_point_tolerance_m':
                self.recent_path_point_tolerance_m = max(0.01, float(parameter.value))
            elif parameter.name == 'publish_target_markers':
                self.publish_target_markers = as_bool(parameter.value)
                if not self.publish_target_markers:
                    self.clear_target_markers()
            elif parameter.name == 'target_marker_scale_m':
                self.target_marker_scale_m = max(0.02, float(parameter.value))
            elif parameter.name == 'bug_recovery_enabled':
                self.bug_recovery_enabled = as_bool(parameter.value)
                if not self.bug_recovery_enabled:
                    self.stop_bug_recovery()
            elif parameter.name == 'bug_wall_side':
                self.bug_wall_side_mode = self.normalized_bug_wall_side(str(parameter.value))
            elif parameter.name == 'bug_forward_speed_mps':
                self.bug_forward_speed_mps = max(0.0, float(parameter.value))
            elif parameter.name == 'bug_lateral_gain':
                self.bug_lateral_gain = max(0.0, float(parameter.value))
            elif parameter.name == 'bug_desired_wall_distance_m':
                self.bug_desired_wall_distance_m = max(
                    self.obstacle_stop_distance_m,
                    float(parameter.value),
                )
            elif parameter.name == 'bug_release_clearance_m':
                self.bug_release_clearance_m = max(
                    self.obstacle_stop_distance_m,
                    float(parameter.value),
                )
            elif parameter.name == 'bug_release_hold_sec':
                self.bug_release_hold_sec = max(0.0, float(parameter.value))
            elif parameter.name == 'bug_min_duration_sec':
                self.bug_min_duration_sec = max(0.0, float(parameter.value))
            elif parameter.name == 'bug_max_duration_sec':
                self.bug_max_duration_sec = max(0.0, float(parameter.value))
            elif parameter.name == 'bug_front_angle_deg':
                self.bug_front_angle_rad = math.radians(max(1.0, float(parameter.value)))
            elif parameter.name == 'bug_side_angle_deg':
                self.bug_side_angle_rad = math.radians(
                    max(10.0, min(120.0, float(parameter.value)))
                )
            elif parameter.name == 'qr_spin_enabled':
                self.qr_spin_enabled = as_bool(parameter.value)
            elif parameter.name == 'qr_spin_cmd_angular_z_rad_s':
                self.qr_spin_cmd_angular_z_rad_s = max(0.01, abs(float(parameter.value)))
            elif parameter.name == 'qr_spin_step_angle_deg':
                self.qr_spin_step_angle_rad = math.radians(max(1.0, float(parameter.value)))
            elif parameter.name == 'qr_spin_step_duration_sec':
                self.qr_spin_step_duration_sec = max(0.0, float(parameter.value))
            elif parameter.name == 'qr_spin_pause_duration_sec':
                self.qr_spin_pause_duration_sec = max(0.0, float(parameter.value))
            elif parameter.name == 'qr_spin_total_angle_deg':
                self.qr_spin_total_angle_rad = math.radians(max(1.0, float(parameter.value)))
            elif parameter.name == 'qr_spin_cooldown_sec':
                self.qr_spin_cooldown_sec = max(0.0, float(parameter.value))
            elif parameter.name == 'qr_spin_max_count':
                self.qr_spin_max_count = max(0, int(parameter.value))
            elif parameter.name == 'qr_spin_front_dead_end_distance_m':
                self.qr_spin_front_dead_end_distance_m = max(0.0, float(parameter.value))
            elif parameter.name == 'qr_spin_side_dead_end_distance_m':
                self.qr_spin_side_dead_end_distance_m = max(0.0, float(parameter.value))
            elif parameter.name == 'qr_spin_qr_text_topic':
                self.qr_spin_qr_text_topic = str(parameter.value)
            elif parameter.name == 'qr_spin_pause_slam':
                self.qr_spin_pause_slam = as_bool(parameter.value)
            elif parameter.name == 'qr_spin_slam_node_name':
                self.qr_spin_slam_node_name = str(parameter.value)
                self.slam_change_state_client = self.create_client(
                    ChangeState,
                    self.slam_change_state_service_name(),
                )
            elif parameter.name == 'qr_spin_stop_on_qr_detected':
                self.qr_spin_stop_on_qr_detected = as_bool(parameter.value)
            elif parameter.name == 'mission_return_home_after_qr_spin':
                self.mission_return_home_after_qr_spin = as_bool(parameter.value)
            elif parameter.name == 'mission_home_marker_state_topic':
                self.mission_home_marker_state_topic = str(parameter.value)
            elif parameter.name == 'mission_home_marker_name':
                self.mission_home_marker_name = str(parameter.value).strip().lower()
            elif parameter.name == 'mission_home_standoff_min_m':
                self.mission_home_standoff_min_m = max(0.0, float(parameter.value))
                self.mission_home_standoff_max_m = max(
                    self.mission_home_standoff_min_m,
                    self.mission_home_standoff_max_m,
                )
                self.clear_home_target()
            elif parameter.name == 'mission_home_standoff_max_m':
                self.mission_home_standoff_max_m = max(
                    self.mission_home_standoff_min_m,
                    float(parameter.value),
                )
                self.clear_home_target()
            elif parameter.name == 'mission_home_goal_tolerance_m':
                self.mission_home_goal_tolerance_m = max(0.03, float(parameter.value))
            elif parameter.name == 'mission_return_home_replan_period_sec':
                self.mission_return_home_replan_period_sec = max(0.5, float(parameter.value))
            elif parameter.name == 'mission_stop_when_home_reached':
                self.mission_stop_when_home_reached = as_bool(parameter.value)
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
            elif parameter.name == 'side_stop_distance_m':
                self.side_stop_distance_m = max(
                    self.obstacle_stop_distance_m,
                    float(parameter.value),
                )
            elif parameter.name == 'side_check_angle_deg':
                self.side_check_angle_rad = math.radians(
                    max(5.0, min(120.0, float(parameter.value)))
                )
            elif parameter.name == 'side_check_window_deg':
                self.side_check_window_rad = math.radians(
                    max(1.0, min(60.0, float(parameter.value)))
                )
            elif parameter.name == 'wall_escape_enter_distance_m':
                self.wall_escape_enter_distance_m = max(0.0, float(parameter.value))
                self.wall_escape_exit_distance_m = max(
                    self.wall_escape_enter_distance_m,
                    self.wall_escape_exit_distance_m,
                )
            elif parameter.name == 'wall_escape_exit_distance_m':
                self.wall_escape_exit_distance_m = max(
                    self.wall_escape_enter_distance_m,
                    float(parameter.value),
                )
            elif parameter.name == 'wall_escape_lateral_speed_mps':
                self.wall_escape_lateral_speed_mps = max(0.0, float(parameter.value))
            elif parameter.name == 'wall_escape_turn_rate_rad_s':
                self.wall_escape_turn_rate_rad_s = max(0.0, float(parameter.value))
            elif parameter.name == 'wall_escape_max_duration_sec':
                self.wall_escape_max_duration_sec = max(0.0, float(parameter.value))
            elif parameter.name == 'wall_escape_exit_hold_sec':
                self.wall_escape_exit_hold_sec = max(0.0, float(parameter.value))
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
        if self.ensure_visited_grid(msg):
            self.clear_current_frontier()
            self.clear_home_target()

    def marker_state_callback(self, msg: String):
        try:
            state = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(
                f'Ignoring malformed QR marker state JSON: {exc}.',
                throttle_duration_sec=2.0,
            )
            return

        if not isinstance(state, dict):
            return

        frame_id = str(state.get('frame_id', '')).strip()
        if frame_id != self.map_frame:
            self.get_logger().warn(
                f'Ignoring QR marker state in frame "{frame_id}"; expected "{self.map_frame}".',
                throttle_duration_sec=2.0,
            )
            return

        markers = state.get('markers', {})
        if not isinstance(markers, dict) or not markers:
            return

        marker_name = None
        marker_data = None
        if self.mission_home_marker_name:
            marker_name = self.mission_home_marker_name
            marker_data = markers.get(marker_name)
            if marker_data is None:
                return
        elif self.home_marker_xy is None:
            marker_name, marker_data = next(iter(markers.items()))
        else:
            return

        if not isinstance(marker_data, dict):
            return

        try:
            marker_xy = (float(marker_data['x']), float(marker_data['y']))
        except (KeyError, TypeError, ValueError):
            self.get_logger().warn(
                f'Ignoring QR marker "{marker_name}" without numeric x/y coordinates.',
                throttle_duration_sec=2.0,
            )
            return

        changed = (
            self.home_marker_name != marker_name
            or self.home_marker_xy is None
            or math.hypot(
                marker_xy[0] - self.home_marker_xy[0],
                marker_xy[1] - self.home_marker_xy[1],
            )
            > 0.01
        )
        self.home_marker_name = str(marker_name)
        self.home_marker_xy = marker_xy
        if changed:
            self.clear_home_target()
            self.get_logger().info(
                f'Home QR marker set to {self.home_marker_name} at '
                f'({marker_xy[0]:.2f}, {marker_xy[1]:.2f})'
            )

    def qr_text_callback(self, msg: String):
        self.last_qr_text = msg.data.strip()
        self.last_qr_seen_sec = self.now_sec()
        if self.qr_spin_active:
            self.get_logger().info(
                f'QR text "{self.last_qr_text}" detected during spin.',
                throttle_duration_sec=1.0,
            )

    def control_loop(self):
        cmd = Twist()

        if not self.enabled or self.latest_scan is None or self.latest_scan_time is None:
            if self.qr_spin_active:
                self.finish_qr_spin_maneuver()
            self.clear_target_markers()
            self.cmd_pub.publish(cmd)
            return

        scan_age_sec = (
            self.get_clock().now() - self.latest_scan_time
        ).nanoseconds * 1e-9
        if scan_age_sec > self.scan_timeout_sec:
            if self.qr_spin_active:
                self.finish_qr_spin_maneuver()
            self.get_logger().warn(
                f'Scan timeout ({scan_age_sec:.2f}s old). Stopping.',
                throttle_duration_sec=2.0,
            )
            self.cmd_pub.publish(cmd)
            return

        if self.mission_mode == 'done':
            self.clear_target_markers()
            self.cmd_pub.publish(cmd)
            return

        if self.qr_spin_active or self.mission_mode == 'qr_spin':
            spin_cmd = self.update_qr_spin_maneuver()
            self.cmd_pub.publish(spin_cmd if spin_cmd is not None else cmd)
            return

        if self.mission_mode == 'return_home':
            wall_escape_cmd = self.wall_escape_command()
            if wall_escape_cmd is not None:
                self.publish_reactive_markers(None)
                self.cmd_pub.publish(wall_escape_cmd)
                return

            home_cmd = self.return_home_control()
            if home_cmd is not None:
                self.cmd_pub.publish(home_cmd)
                return

            if self.reactive_fallback:
                self.cmd_pub.publish(self.reactive_fallback_command())
            else:
                self.clear_target_markers()
                self.cmd_pub.publish(cmd)
            return

        wall_escape_cmd = self.wall_escape_command()
        if wall_escape_cmd is not None:
            self.reset_frontier_progress()
            self.publish_reactive_markers(None)
            self.cmd_pub.publish(wall_escape_cmd)
            return

        if self.exploration_mode == 'frontier':
            frontier_cmd = self.frontier_control()
            if frontier_cmd is not None and self.cmd_is_nonzero(frontier_cmd):
                self.cmd_pub.publish(frontier_cmd)
                return
            self.get_logger().info(
                'No map target; using simple LiDAR heading.',
                throttle_duration_sec=4.0,
            )
            if self.should_start_qr_spin_maneuver():
                self.start_qr_spin_maneuver()
                spin_cmd = self.update_qr_spin_maneuver()
                self.cmd_pub.publish(spin_cmd if spin_cmd is not None else cmd)
                return

        self.cmd_pub.publish(self.reactive_fallback_command())

    def reactive_fallback_command(self) -> Twist:
        cmd = Twist()
        best = self.choose_best_direction(self.latest_scan)
        if best is None:
            self.publish_reactive_markers(None)
            self.get_logger().warn(
                'No safe LiDAR heading found; stopping.',
                throttle_duration_sec=2.0,
            )
            return cmd

        body_angle_rad = self.scan_angle_to_body_angle(self.latest_scan, best.angle_rad)
        self.publish_reactive_markers(best)
        if best.min_range_m < self.obstacle_stop_distance_m:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = self.front_escape_turn_direction(
                *self.side_clearances(),
            ) * min(self.wall_escape_turn_rate_rad_s, self.max_turn_rate_rad_s)
            self.get_logger().warn(
                'Front blocked: stopping forward motion and turning gently toward clearer side',
                throttle_duration_sec=2.0,
            )
            return cmd

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
            guard_cmd = self.wall_clearance_guard_command()
            if guard_cmd is not None:
                return guard_cmd
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
        return cmd

    def should_start_qr_spin_maneuver(self) -> bool:
        if (
            not self.qr_spin_enabled
            or self.mission_mode != 'explore'
            or self.qr_spin_active
            or self.wall_escape_active
        ):
            return False
        if self.qr_spin_count >= self.qr_spin_max_count:
            return False
        if self.current_frontier is not None:
            return False
        if self.latest_scan is None or self.latest_scan_time is None:
            return False

        now_sec = self.now_sec()
        scan_age_sec = (self.get_clock().now() - self.latest_scan_time).nanoseconds * 1e-9
        if scan_age_sec > self.scan_timeout_sec:
            return False
        if now_sec - self.qr_spin_last_start_sec < self.qr_spin_cooldown_sec:
            return False
        return self.qr_spin_dead_end_detected()

    def qr_spin_dead_end_detected(self) -> bool:
        front_clearance = self.min_scan_range_near_body_angle(
            0.0,
            self.direction_window_rad,
        )
        left_clearance, right_clearance = self.side_clearances()
        detected = (
            front_clearance is not None
            and front_clearance < self.qr_spin_front_dead_end_distance_m
            and left_clearance is not None
            and left_clearance < self.qr_spin_side_dead_end_distance_m
            and right_clearance is not None
            and right_clearance < self.qr_spin_side_dead_end_distance_m
        )
        if detected:
            self.get_logger().info(
                'QR spin candidate: dead end detected '
                f'front={self.format_clearance(front_clearance)} '
                f'left={self.format_clearance(left_clearance)} '
                f'right={self.format_clearance(right_clearance)}',
                throttle_duration_sec=2.0,
            )
        return detected

    def start_qr_spin_maneuver(self) -> bool:
        if not self.should_start_qr_spin_maneuver():
            return False

        now_sec = self.now_sec()
        self.mission_mode = 'qr_spin'
        self.qr_spin_active = True
        self.qr_spin_phase = 'spin'
        self.qr_spin_started_sec = now_sec
        self.qr_spin_phase_started_sec = now_sec
        self.qr_spin_accumulated_angle_rad = 0.0
        self.qr_spin_accumulated_yaw_rad = 0.0
        self.qr_spin_last_start_sec = now_sec
        self.qr_spin_count += 1
        self.reset_frontier_progress()
        self.pause_slam_for_qr_spin()
        self.get_logger().info(
            f'Starting QR spin maneuver {self.qr_spin_count}/'
            f'{self.qr_spin_max_count}: step={math.degrees(self.qr_spin_step_angle_rad):.1f} deg, '
            f'pause={self.qr_spin_pause_duration_sec:.1f}s.'
        )
        return True

    def update_qr_spin_maneuver(self) -> Twist | None:
        if not self.qr_spin_active:
            return None

        cmd = Twist()

        if self.qr_spin_waiting_for_slam_pause:
            wait_sec = self.now_sec() - self.qr_spin_slam_pause_requested_sec
            if (
                self.slam_transition_future is not None
                and not self.slam_transition_future.done()
                and wait_sec < 2.0
            ):
                return cmd
            if self.slam_transition_future is not None and not self.slam_transition_future.done():
                self.get_logger().warn(
                    'Timed out waiting for SLAM deactivate response; continuing QR spin anyway.',
                    throttle_duration_sec=2.0,
                )
            self.qr_spin_waiting_for_slam_pause = False
            self.qr_spin_started_sec = self.now_sec()
            self.qr_spin_phase_started_sec = self.qr_spin_started_sec
            try:
                transition_done = (
                    self.slam_transition_future is not None
                    and self.slam_transition_future.done()
                )
                response = (
                    self.slam_transition_future.result()
                    if transition_done
                    else None
                )
                if response is not None and not response.success:
                    self.qr_spin_slam_was_paused = False
                    self.get_logger().warn(
                        'SLAM deactivate request was rejected; continuing QR spin anyway.',
                        throttle_duration_sec=2.0,
                    )
            except Exception as exc:
                self.qr_spin_slam_was_paused = False
                self.get_logger().warn(
                    f'SLAM deactivate request failed: {exc}. Continuing QR spin anyway.',
                    throttle_duration_sec=2.0,
                )
            return cmd

        now_sec = self.now_sec()
        if self.qr_spin_qr_seen_during_spin():
            self.get_logger().info(
                f'QR text "{self.last_qr_text}" seen during spin; stopping QR scan early.'
            )
            self.finish_qr_spin_maneuver()
            return cmd

        if self.qr_spin_phase == 'spin':
            spin_duration_sec = self.qr_spin_current_step_duration_sec()
            elapsed_sec = now_sec - self.qr_spin_phase_started_sec
            if elapsed_sec < spin_duration_sec:
                cmd.angular.z = self.qr_spin_cmd_angular_z_rad_s
                return cmd

            self.qr_spin_accumulated_angle_rad += (
                abs(self.qr_spin_cmd_angular_z_rad_s) * elapsed_sec
            )
            self.qr_spin_phase = 'pause'
            self.qr_spin_phase_started_sec = now_sec
            self.get_logger().info(
                f'QR spin pause: accumulated='
                f'{math.degrees(self.qr_spin_accumulated_angle_rad):.1f} deg.',
                throttle_duration_sec=2.0,
            )
            return cmd

        if self.qr_spin_phase == 'pause':
            elapsed_sec = now_sec - self.qr_spin_phase_started_sec
            if elapsed_sec < self.qr_spin_pause_duration_sec:
                return cmd
            if self.qr_spin_accumulated_angle_rad >= self.qr_spin_total_angle_rad:
                self.finish_qr_spin_maneuver()
                return cmd
            self.qr_spin_phase = 'spin'
            self.qr_spin_phase_started_sec = now_sec
            return cmd

        self.qr_spin_phase = 'spin'
        self.qr_spin_phase_started_sec = now_sec
        return cmd

    def finish_qr_spin_maneuver(self):
        if not self.qr_spin_active and self.mission_mode != 'qr_spin':
            return

        self.qr_spin_active = False
        self.qr_spin_phase = 'idle'
        self.qr_spin_phase_started_sec = 0.0
        self.qr_spin_accumulated_yaw_rad = 0.0
        self.qr_spin_accumulated_angle_rad = 0.0
        self.resume_slam_after_qr_spin()
        self.get_logger().info('Finished QR spin maneuver.')

        if self.mission_return_home_after_qr_spin and self.home_marker_xy is not None:
            self.start_return_home_mode()
        else:
            self.mission_mode = 'explore'

    def qr_spin_current_step_duration_sec(self) -> float:
        if self.qr_spin_step_duration_sec > 0.0:
            return self.qr_spin_step_duration_sec
        if self.qr_spin_cmd_angular_z_rad_s <= 0.0:
            return 0.0
        return self.qr_spin_step_angle_rad / self.qr_spin_cmd_angular_z_rad_s

    def qr_spin_qr_seen_during_spin(self) -> bool:
        return (
            self.qr_spin_stop_on_qr_detected
            and self.last_qr_seen_sec is not None
            and self.last_qr_seen_sec >= self.qr_spin_started_sec
        )

    def slam_change_state_service_name(self) -> str:
        node_name = self.qr_spin_slam_node_name.strip() or '/slam_toolbox'
        node_name = '/' + node_name.strip('/')
        return f'{node_name}/change_state'

    def pause_slam_for_qr_spin(self):
        if not self.qr_spin_pause_slam:
            return
        if not self.slam_change_state_client.service_is_ready():
            self.get_logger().warn(
                f'SLAM lifecycle service {self.slam_change_state_service_name()} '
                'is not ready; QR spin will continue without pausing SLAM.',
                throttle_duration_sec=5.0,
            )
            return
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_DEACTIVATE
        self.qr_spin_slam_was_paused = True
        self.qr_spin_waiting_for_slam_pause = True
        self.qr_spin_slam_pause_requested_sec = self.now_sec()
        self.slam_transition_future = self.slam_change_state_client.call_async(request)
        self.get_logger().info('Requested SLAM deactivate before QR spin.')

    def resume_slam_after_qr_spin(self):
        if not self.qr_spin_slam_was_paused:
            return
        self.qr_spin_slam_was_paused = False
        self.qr_spin_waiting_for_slam_pause = False
        if not self.slam_change_state_client.service_is_ready():
            self.get_logger().warn(
                f'SLAM lifecycle service {self.slam_change_state_service_name()} '
                'is not ready; could not reactivate SLAM after QR spin.',
                throttle_duration_sec=5.0,
            )
            return
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_ACTIVATE
        self.slam_transition_future = self.slam_change_state_client.call_async(request)
        self.get_logger().info('Requested SLAM activate after QR spin.')

    def start_return_home_mode(self) -> bool:
        if self.home_marker_xy is None:
            self.get_logger().warn(
                'Cannot switch to RETURN_HOME: no home QR marker is available.',
                throttle_duration_sec=2.0,
            )
            return False

        self.mission_mode = 'return_home'
        self.clear_current_frontier()
        self.clear_home_target()
        self.clear_target_markers()
        self.get_logger().info('Switching mission mode to RETURN_HOME')
        return True

    def return_home_control(self) -> Twist | None:
        cmd = Twist()

        if self.home_marker_xy is None:
            self.get_logger().warn(
                'RETURN_HOME requested without a home QR marker; stopping.',
                throttle_duration_sec=2.0,
            )
            return cmd
        if self.latest_map is None or self.latest_map_time is None:
            self.get_logger().warn(
                'RETURN_HOME waiting for /map before planning home path.',
                throttle_duration_sec=2.0,
            )
            return cmd

        map_age_sec = (self.get_clock().now() - self.latest_map_time).nanoseconds * 1e-9
        if map_age_sec > self.map_timeout_sec:
            self.get_logger().warn(
                f'RETURN_HOME map timeout ({map_age_sec:.2f}s old); stopping.',
                throttle_duration_sec=2.0,
            )
            return cmd

        robot_pose = self.robot_pose_in_map()
        if robot_pose is None:
            self.get_logger().warn(
                'RETURN_HOME cannot find robot pose in map; stopping.',
                throttle_duration_sec=2.0,
            )
            return cmd

        if self.home_target is not None:
            distance_to_home_target = math.hypot(
                robot_pose[0] - self.home_target.x_m,
                robot_pose[1] - self.home_target.y_m,
            )
            if distance_to_home_target <= self.mission_home_goal_tolerance_m:
                return self.finish_return_home()

        if self.home_target is None or self.home_replan_due():
            self.plan_home_target()

        if self.home_target is None:
            self.get_logger().warn(
                'RETURN_HOME could not find a reachable standoff near the home QR marker.',
                throttle_duration_sec=2.0,
            )
            return None

        while self.home_target is not None:
            waypoint = self.active_home_waypoint()
            if waypoint is None:
                return self.finish_return_home()

            body_vector = self.map_target_vector_in_body(waypoint[0], waypoint[1])
            if body_vector is None:
                return cmd

            body_x, body_y = body_vector
            distance_m = math.hypot(body_x, body_y)
            if distance_m < self.mission_home_goal_tolerance_m:
                if self.advance_home_waypoint():
                    continue
                return self.finish_return_home()

            body_angle = math.atan2(body_y, body_x)
            scan_clearance = self.min_scan_range_near_body_angle(
                body_angle,
                self.direction_window_rad,
            )
            if scan_clearance is not None and scan_clearance < self.obstacle_stop_distance_m:
                self.get_logger().warn(
                    'RETURN_HOME path direction is blocked by scan; stopping.',
                    throttle_duration_sec=2.0,
                )
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
                    cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            else:
                cmd.linear.x = speed * max(0.0, math.cos(body_angle))
                cmd.angular.z = clamp(
                    self.turn_gain * body_angle,
                    -self.max_turn_rate_rad_s,
                    self.max_turn_rate_rad_s,
                )
            return cmd

        return cmd

    def finish_return_home(self) -> Twist:
        cmd = Twist()
        self.clear_home_target()
        self.clear_current_frontier()
        self.clear_target_markers()
        if self.mission_stop_when_home_reached:
            self.mission_mode = 'done'
            self.get_logger().info('Reached home QR standoff target; mission done.')
        else:
            self.mission_mode = 'explore'
            self.get_logger().info('Reached home QR standoff target; resuming exploration.')
        return cmd

    def home_replan_due(self) -> bool:
        if self.last_home_plan_time is None:
            return True
        elapsed = (self.get_clock().now() - self.last_home_plan_time).nanoseconds * 1e-9
        return elapsed >= self.mission_return_home_replan_period_sec

    def active_home_waypoint(self):
        if self.home_target is None or not self.home_target.path:
            return None
        index = min(self.home_path_index, len(self.home_target.path) - 1)
        return self.home_target.path[index]

    def advance_home_waypoint(self) -> bool:
        if self.home_target is None:
            return False
        next_index = self.home_path_index + 1
        if next_index >= len(self.home_target.path):
            return False
        self.home_path_index = next_index
        return True

    def clear_home_target(self):
        self.home_target = None
        self.home_path_index = 0
        self.last_home_plan_time = None

    def plan_home_target(self):
        grid = self.latest_map
        marker_xy = self.home_marker_xy
        if marker_xy is None or grid is None or grid.info.width <= 0 or grid.info.height <= 0:
            return
        if grid.info.resolution <= 0.0:
            self.get_logger().warn(
                'RETURN_HOME planner cannot use a map with invalid resolution.',
                throttle_duration_sec=2.0,
            )
            return

        self.last_home_plan_time = self.get_clock().now()
        robot_pose = self.robot_pose_in_map()
        if robot_pose is None:
            return

        planner = self.build_planner_grid(grid)
        start = self.world_to_grid(grid, robot_pose[0], robot_pose[1])
        if start is None:
            return

        width = grid.info.width
        height = grid.info.height
        cell_count = width * height
        start_index = self.grid_index(start[0], start[1], width)
        if not planner.safe[start_index]:
            nearest = self.nearest_safe_cell(planner, start[0], start[1])
            if nearest is None:
                self.get_logger().warn(
                    'RETURN_HOME planner cannot find a safe start cell near the robot.',
                    throttle_duration_sec=2.0,
                )
                return
            start_index = self.grid_index(nearest[0], nearest[1], width)

        reachable = bytearray(cell_count)
        parent = [-1] * cell_count
        path_cost_cells = [math.inf] * cell_count
        pending = deque([start_index])
        reachable[start_index] = 1
        parent[start_index] = start_index
        path_cost_cells[start_index] = 0.0
        searched = 0

        while pending:
            current = pending.popleft()
            searched += 1
            for neighbor in self.safe_neighbor_indices(current, planner):
                if reachable[neighbor]:
                    continue
                reachable[neighbor] = 1
                parent[neighbor] = current
                path_cost_cells[neighbor] = (
                    path_cost_cells[current] + self.grid_step_cost(current, neighbor, width)
                )
                pending.append(neighbor)

        origin = grid.info.origin.position
        resolution = grid.info.resolution
        home_grid_x = int((marker_xy[0] - origin.x) / resolution)
        home_grid_y = int((marker_xy[1] - origin.y) / resolution)
        max_radius_cells = max(
            1,
            int(math.ceil(self.mission_home_standoff_max_m / resolution)),
        )
        desired_standoff_m = (
            self.mission_home_standoff_min_m + self.mission_home_standoff_max_m
        ) * 0.5
        candidates = []
        for grid_y in range(
            max(0, home_grid_y - max_radius_cells),
            min(height, home_grid_y + max_radius_cells + 1),
        ):
            for grid_x in range(
                max(0, home_grid_x - max_radius_cells),
                min(width, home_grid_x + max_radius_cells + 1),
            ):
                candidate_index = self.grid_index(grid_x, grid_y, width)
                if not planner.goal_safe[candidate_index] and not planner.path_safe[candidate_index]:
                    continue
                candidate_world = self.grid_to_world(grid, grid_x, grid_y)
                standoff_m = math.hypot(
                    candidate_world[0] - marker_xy[0],
                    candidate_world[1] - marker_xy[1],
                )
                if (
                    standoff_m < self.mission_home_standoff_min_m
                    or standoff_m > self.mission_home_standoff_max_m
                ):
                    continue
                parent_index = self.reachable_goal_parent(candidate_index, planner, reachable)
                if parent_index < 0:
                    continue
                path_cost = path_cost_cells[candidate_index]
                if not math.isfinite(path_cost):
                    path_cost = path_cost_cells[parent_index] + self.grid_step_cost(
                        parent_index,
                        candidate_index,
                        width,
                    )
                if not math.isfinite(path_cost):
                    continue
                candidates.append(
                    (
                        abs(standoff_m - desired_standoff_m),
                        path_cost * resolution,
                        candidate_index,
                        parent_index,
                        standoff_m,
                    )
                )

        if not candidates:
            return

        candidates.sort(key=lambda item: (item[0], item[1]))
        _, path_length_m, target_index, parent_index, standoff_m = candidates[0]
        if parent_index == target_index:
            path = self.reconstruct_world_path(grid, parent, start_index, target_index)
        else:
            path = self.reconstruct_world_path(grid, parent, start_index, parent_index)
            target_x = target_index % width
            target_y = target_index // width
            path = path + (self.grid_to_world(grid, target_x, target_y),)
        if not path:
            return

        target_x = target_index % width
        target_y = target_index // width
        target_world = self.grid_to_world(grid, target_x, target_y)
        self.home_target = FrontierTarget(
            x_m=target_world[0],
            y_m=target_world[1],
            grid_x=target_x,
            grid_y=target_y,
            searched_cells=searched,
            path=path,
            target_reason='return_home',
            clearance_m=planner.clearance_m[target_index],
            path_length_m=path_length_m,
            score=-path_length_m,
        )
        self.home_path_index = 0
        self.get_logger().info(
            f'Planned RETURN_HOME target near {self.home_marker_name}: '
            f'({target_world[0]:.2f}, {target_world[1]:.2f}), '
            f'standoff={standoff_m:.2f} m, path={path_length_m:.2f} m, '
            f'waypoints={len(path)}.'
        )

    def frontier_control(self):
        cmd = Twist()

        if self.latest_map is None or self.latest_map_time is None:
            self.get_logger().warn(
                'No /map received yet. Waiting before frontier exploration.',
                throttle_duration_sec=2.0,
            )
            return None

        robot_pose = self.robot_pose_in_map()
        if robot_pose is not None:
            self.mark_visited_around(self.latest_map, robot_pose[0], robot_pose[1])

        map_age_sec = (self.get_clock().now() - self.latest_map_time).nanoseconds * 1e-9
        if map_age_sec > self.map_timeout_sec:
            self.get_logger().warn(
                f'Map timeout ({map_age_sec:.2f}s old). Using scan-reactive fallback.',
                throttle_duration_sec=2.0,
            )
            return None

        if self.current_frontier is None and self.frontier_replan_due():
            self.plan_frontier_target()

        if self.current_frontier is None:
            self.get_logger().info(
                'No map target found. Using scan-reactive fallback.',
                throttle_duration_sec=4.0,
            )
            self.publish_suppressed_markers()
            return None

        while self.current_frontier is not None:
            waypoint = self.active_frontier_waypoint()
            if waypoint is None:
                self.clear_current_frontier()
                self.clear_target_markers()
                return None

            self.publish_frontier_markers(waypoint)

            body_vector = self.map_target_vector_in_body(waypoint[0], waypoint[1])
            if body_vector is None:
                return None

            body_x, body_y = body_vector
            distance_m = math.hypot(body_x, body_y)
            if distance_m < self.active_target_goal_tolerance():
                if self.advance_frontier_waypoint():
                    continue
                self.get_logger().info(
                    'Reached map target; clearing target and replanning.',
                    throttle_duration_sec=2.0,
                )
                self.clear_current_frontier()
                self.plan_frontier_target()
                self.publish_suppressed_markers()
                continue

            body_angle = math.atan2(body_y, body_x)
            if (
                not self.crab_motion
                and not self.reverse_allowed
                and abs(body_angle) > math.pi / 2.0
            ):
                return self.turn_toward_target_command(body_angle)

            scan_clearance = self.min_scan_range_near_body_angle(
                body_angle,
                self.direction_window_rad,
            )
            if scan_clearance is not None and scan_clearance < self.obstacle_stop_distance_m:
                self.get_logger().warn(
                    'Map target direction is blocked by scan; using simple LiDAR heading.',
                    throttle_duration_sec=2.0,
                )
                return None

            if self.bug_active:
                if self.frontier_progress_timed_out(distance_m):
                    self.suppress_current_frontier('bug_no_progress')
                    self.publish_suppressed_markers()
                    return None
                bug_cmd = self.bug_recovery_command(body_angle)
                if bug_cmd is not None:
                    return bug_cmd
                if self.bug_timed_out:
                    self.suppress_current_frontier('bug_timeout')
                    self.publish_suppressed_markers()
                    return None

            if self.side_clearance_is_too_close():
                if self.bug_recovery_enabled:
                    bug_cmd = self.bug_recovery_command(body_angle)
                    if bug_cmd is not None:
                        return bug_cmd
                    if self.bug_timed_out:
                        self.suppress_current_frontier('bug_timeout')
                        self.publish_suppressed_markers()
                        return None
                guard_cmd = self.wall_clearance_guard_command()
                if guard_cmd is not None:
                    return guard_cmd

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
                    cmd.linear.x = 0.0
                    if abs(cmd.linear.y) < max(0.004, self.min_speed_mps * 0.5):
                        if self.advance_frontier_waypoint():
                            continue
                        return self.turn_toward_target_command(body_angle)
            else:
                cmd.linear.x = speed * max(0.0, math.cos(body_angle))
                cmd.angular.z = clamp(
                    self.turn_gain * body_angle,
                    -self.max_turn_rate_rad_s,
                    self.max_turn_rate_rad_s,
                )

            if self.cmd_is_nonzero(cmd) and self.frontier_progress_timed_out(distance_m):
                self.suppress_current_frontier('no_progress')
                self.publish_suppressed_markers()
                return None

            return cmd

        return None

    def turn_toward_target_command(self, body_angle_rad: float) -> Twist:
        cmd = Twist()
        turn = clamp(
            self.turn_gain * normalize_angle(body_angle_rad),
            -self.max_turn_rate_rad_s,
            self.max_turn_rate_rad_s,
        )
        min_turn = min(self.max_turn_rate_rad_s, max(0.03, self.max_turn_rate_rad_s * 0.35))
        if abs(turn) < min_turn:
            turn = math.copysign(min_turn, body_angle_rad if abs(body_angle_rad) > 1e-6 else 1.0)
        cmd.angular.z = turn
        return cmd

    def wall_escape_command(self):
        """Return a high-priority scan-only safety command, or None.

        This function intentionally uses only the latest LaserScan-derived front
        and side clearances.  It runs before frontier/path following in
        control_loop(), so physical collision avoidance always overrides the map
        planner.
        """
        left_clearance, right_clearance = self.side_clearances()
        front_clearance = self.min_scan_range_near_body_angle(
            0.0,
            self.direction_window_rad,
        )
        now_sec = self.now_sec()

        front_too_close = (
            front_clearance is not None
            and front_clearance < self.obstacle_stop_distance_m
        )
        side_too_close = self.side_clearance_below(
            left_clearance,
            right_clearance,
            self.wall_escape_enter_distance_m,
        )
        side_close_with_front_blocked = (
            front_too_close
            and self.side_clearance_below(
                left_clearance,
                right_clearance,
                self.wall_escape_exit_distance_m,
            )
        )

        if self.wall_escape_active:
            selected_side_clearance = self.clearance_for_wall_side(
                self.wall_escape_side,
                left_clearance,
                right_clearance,
            )
            opposite_side = -self.wall_escape_side
            opposite_clearance = self.clearance_for_wall_side(
                opposite_side,
                left_clearance,
                right_clearance,
            )

            # Keep the same wall side to avoid left/right oscillation.  Only
            # switch if the opposite side becomes substantially closer.
            if (
                self.wall_escape_side != 0
                and selected_side_clearance is not None
                and opposite_clearance is not None
                and opposite_clearance + 0.15 < selected_side_clearance
            ):
                self.wall_escape_side = opposite_side
                self.wall_escape_clear_since_sec = None
                self.get_logger().warn(
                    'Wall escape switching sides: '
                    f'opposite wall is closer by at least 0.15 m; now avoiding '
                    f'{self.wall_side_name(self.wall_escape_side)} wall.',
                    throttle_duration_sec=1.0,
                )

            restored = self.side_clearance_above(
                left_clearance,
                right_clearance,
                self.wall_escape_exit_distance_m,
            )
            timed_out = (
                self.wall_escape_max_duration_sec > 0.0
                and now_sec - self.wall_escape_started_sec >= self.wall_escape_max_duration_sec
            )

            if restored:
                if self.wall_escape_clear_since_sec is None:
                    self.wall_escape_clear_since_sec = now_sec
                clear_long_enough = (
                    now_sec - self.wall_escape_clear_since_sec
                ) >= self.wall_escape_exit_hold_sec
                if not clear_long_enough:
                    self.get_logger().warn(
                        'Wall escape holding until clearance stable: '
                        f'left={self.format_clearance(left_clearance)} '
                        f'right={self.format_clearance(right_clearance)}',
                        throttle_duration_sec=1.0,
                    )
            else:
                self.wall_escape_clear_since_sec = None
                clear_long_enough = False

            if timed_out or clear_long_enough:
                old_side = self.wall_escape_side
                self.wall_escape_active = False
                self.wall_escape_side = 0
                self.wall_escape_clear_since_sec = None
                self.reset_frontier_progress()
                if timed_out:
                    self.get_logger().info(
                        'Exiting wall escape: max duration reached.',
                        throttle_duration_sec=1.0,
                    )
                else:
                    self.get_logger().info(
                        'Exiting wall escape: side clearance restored and held.',
                        throttle_duration_sec=1.0,
                    )
                # If the scan is still unsafe after clearing state, immediately
                # continue with the safety behavior below instead of allowing a
                # planner command through this cycle.
                if not side_too_close and not side_close_with_front_blocked and not front_too_close:
                    return None
                if side_too_close or side_close_with_front_blocked:
                    self.wall_escape_side = old_side or self.closer_wall_side(
                        left_clearance,
                        right_clearance,
                    )
                    self.wall_escape_active = self.wall_escape_side != 0
                    self.wall_escape_started_sec = now_sec

        if not self.wall_escape_active and (side_too_close or side_close_with_front_blocked):
            self.wall_escape_side = self.closer_wall_side(left_clearance, right_clearance)
            if self.wall_escape_side != 0:
                self.wall_escape_active = True
                self.wall_escape_started_sec = now_sec
                self.wall_escape_clear_since_sec = None
                self.reset_frontier_progress()
                self.get_logger().warn(
                    'Entering wall escape: '
                    f'wall={self.wall_side_name(self.wall_escape_side)}, '
                    f'left={self.format_clearance(left_clearance)}, '
                    f'right={self.format_clearance(right_clearance)}, '
                    f'front={self.format_clearance(front_clearance)}',
                    throttle_duration_sec=1.0,
                )

        if self.wall_escape_active and self.wall_escape_side != 0:
            return self.wall_escape_cmd_for_side(self.wall_escape_side)

        if front_too_close:
            self.reset_frontier_progress()
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = self.front_escape_turn_direction(
                left_clearance,
                right_clearance,
            ) * min(self.wall_escape_turn_rate_rad_s, self.max_turn_rate_rad_s)
            self.get_logger().warn(
                'Front blocked: stopping forward motion and turning gently toward clearer side',
                throttle_duration_sec=2.0,
            )
            return cmd

        return None

    def wall_escape_cmd_for_side(self, wall_side: int) -> Twist:
        # wall_side = +1 means wall on left; -1 means wall on right.
        # Move laterally away from that side without yawing into nearby walls.
        away_direction = -float(wall_side)
        cmd = Twist()
        cmd.linear.x = 0.0
        if self.crab_motion:
            cmd.linear.y = away_direction * min(
                self.wall_escape_lateral_speed_mps,
                self.max_lateral_speed_mps,
            )
        else:
            cmd.linear.y = 0.0
        cmd.angular.z = 0.0
        self.reset_frontier_progress()
        self.get_logger().warn(
            f'Wall escape active: moving away from {self.wall_side_name(wall_side)} wall '
            f'(linear.x={cmd.linear.x:.3f}, linear.y={cmd.linear.y:.3f}, '
            f'angular.z={cmd.angular.z:.3f})',
            throttle_duration_sec=1.0,
        )
        return cmd

    def clearance_for_wall_side(
        self,
        wall_side: int,
        left_clearance: float | None,
        right_clearance: float | None,
    ):
        if wall_side > 0:
            return left_clearance
        if wall_side < 0:
            return right_clearance
        return None

    def side_clearance_below(
        self,
        left_clearance: float | None,
        right_clearance: float | None,
        threshold_m: float,
    ) -> bool:
        return (
            left_clearance is not None and left_clearance < threshold_m
        ) or (
            right_clearance is not None and right_clearance < threshold_m
        )

    def side_clearance_above(
        self,
        left_clearance: float | None,
        right_clearance: float | None,
        threshold_m: float,
    ) -> bool:
        left_ok = left_clearance is None or left_clearance > threshold_m
        right_ok = right_clearance is None or right_clearance > threshold_m
        return left_ok and right_ok

    def closer_wall_side(
        self,
        left_clearance: float | None,
        right_clearance: float | None,
    ) -> int:
        left_value = math.inf if left_clearance is None else left_clearance
        right_value = math.inf if right_clearance is None else right_clearance
        if not math.isfinite(left_value) and not math.isfinite(right_value):
            return 0
        return 1 if left_value <= right_value else -1

    def front_escape_turn_direction(
        self,
        left_clearance: float | None,
        right_clearance: float | None,
    ) -> float:
        left_value = -math.inf if left_clearance is None else left_clearance
        right_value = -math.inf if right_clearance is None else right_clearance
        if left_value == right_value:
            return 1.0
        return 1.0 if left_value > right_value else -1.0

    def wall_side_name(self, wall_side: int) -> str:
        if wall_side > 0:
            return 'left'
        if wall_side < 0:
            return 'right'
        return 'unknown'

    def format_clearance(self, clearance_m: float | None) -> str:
        return 'unknown' if clearance_m is None else f'{clearance_m:.2f}'

    def side_clearance_is_too_close(self) -> bool:
        left_clearance, right_clearance = self.side_clearances()
        return (
            left_clearance is not None and left_clearance < self.side_stop_distance_m
        ) or (
            right_clearance is not None and right_clearance < self.side_stop_distance_m
        )

    def wall_clearance_guard_command(self):
        left_clearance, right_clearance = self.side_clearances()
        close_sides = [
            (left_clearance, -1.0),
            (right_clearance, 1.0),
        ]
        close_sides = [
            (clearance, away_direction)
            for clearance, away_direction in close_sides
            if clearance is not None and clearance < self.side_stop_distance_m
        ]
        if not close_sides:
            return None

        clearance, away_direction = min(close_sides, key=lambda item: item[0])
        deficit = self.side_stop_distance_m - clearance
        lateral_speed = clamp(
            self.min_speed_mps + deficit,
            self.min_speed_mps,
            self.max_lateral_speed_mps,
        )

        cmd = Twist()
        if self.crab_motion:
            cmd.linear.x = 0.0
            cmd.linear.y = away_direction * lateral_speed
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        self.get_logger().warn(
            f'Side clearance {clearance:.2f} m is below {self.side_stop_distance_m:.2f} m; '
            'moving away from wall.',
            throttle_duration_sec=2.0,
        )
        return cmd

    def side_clearances(self):
        left_clearance = self.min_scan_range_near_body_angle(
            self.side_check_angle_rad,
            self.side_check_window_rad,
        )
        right_clearance = self.min_scan_range_near_body_angle(
            -self.side_check_angle_rad,
            self.side_check_window_rad,
        )
        return left_clearance, right_clearance

    def bug_recovery_command(self, target_body_angle_rad: float):
        if self.latest_scan is None:
            return None

        self.bug_timed_out = False
        if not self.bug_active:
            self.start_bug_recovery(target_body_angle_rad)

        if self.bug_max_duration_sec > 0.0:
            age_sec = self.now_sec() - self.bug_started_at_sec
            if age_sec > self.bug_max_duration_sec:
                self.get_logger().info(
                    'Bug recovery timed out; rejecting current frontier path.',
                    throttle_duration_sec=2.0,
                )
                self.bug_timed_out = True
                self.stop_bug_recovery()
                return None

        if self.bug_minimum_duration_elapsed() and self.direct_path_released(
            target_body_angle_rad
        ):
            self.stop_bug_recovery()
            return None

        cmd = Twist()
        front_clearance = self.min_scan_range_near_body_angle(0.0, self.bug_front_angle_rad)
        side_angle = self.bug_wall_side * self.bug_side_angle_rad
        side_clearance = self.min_scan_range_near_body_angle(side_angle, self.direction_window_rad)

        forward_speed = min(self.bug_forward_speed_mps, self.max_speed_mps)
        if front_clearance is not None and front_clearance < self.obstacle_stop_distance_m:
            forward_speed = 0.0

        if side_clearance is None:
            lateral_speed = self.bug_wall_side * self.max_lateral_speed_mps
        else:
            wall_error_m = side_clearance - self.bug_desired_wall_distance_m
            lateral_speed = self.bug_wall_side * self.bug_lateral_gain * wall_error_m
            lateral_speed = clamp(
                lateral_speed,
                -self.max_lateral_speed_mps,
                self.max_lateral_speed_mps,
            )

        target_bias = clamp(
            math.sin(target_body_angle_rad) * self.max_lateral_speed_mps * 0.35,
            -self.max_lateral_speed_mps * 0.35,
            self.max_lateral_speed_mps * 0.35,
        )
        cmd.linear.x = forward_speed
        cmd.linear.y = clamp(
            lateral_speed + target_bias,
            -self.max_lateral_speed_mps,
            self.max_lateral_speed_mps,
        )

        if not self.crab_motion:
            cmd.angular.z = clamp(
                self.bug_wall_side * self.max_turn_rate_rad_s,
                -self.max_turn_rate_rad_s,
                self.max_turn_rate_rad_s,
            )
            cmd.linear.y = 0.0

        return cmd

    def start_bug_recovery(self, target_body_angle_rad: float):
        self.bug_active = True
        self.bug_timed_out = False
        self.bug_started_at_sec = self.now_sec()
        self.bug_clear_since_sec = None
        self.bug_wall_side = self.select_bug_wall_side(target_body_angle_rad)
        side_name = 'left' if self.bug_wall_side > 0 else 'right'
        self.get_logger().info(
            f'Bug recovery started, following {side_name} wall.',
            throttle_duration_sec=2.0,
        )

    def stop_bug_recovery(self):
        self.bug_active = False
        self.bug_clear_since_sec = None

    def bug_minimum_duration_elapsed(self) -> bool:
        return (self.now_sec() - self.bug_started_at_sec) >= self.bug_min_duration_sec

    def select_bug_wall_side(self, target_body_angle_rad: float) -> int:
        if self.bug_wall_side_mode == 'left':
            return 1
        if self.bug_wall_side_mode == 'right':
            return -1

        left_clearance = self.min_scan_range_near_body_angle(
            self.bug_side_angle_rad,
            self.direction_window_rad,
        )
        right_clearance = self.min_scan_range_near_body_angle(
            -self.bug_side_angle_rad,
            self.direction_window_rad,
        )
        left = left_clearance if left_clearance is not None else self.max_usable_range_m
        right = right_clearance if right_clearance is not None else self.max_usable_range_m
        if abs(left - right) > 0.05:
            return 1 if left > right else -1
        return 1 if target_body_angle_rad >= 0.0 else -1

    def direct_path_released(self, target_body_angle_rad: float) -> bool:
        clearance = self.min_scan_range_near_body_angle(
            target_body_angle_rad,
            self.direction_window_rad,
        )
        if clearance is None or clearance < self.bug_release_clearance_m:
            self.bug_clear_since_sec = None
            return False

        if self.bug_release_hold_sec <= 0.0:
            return True

        now_sec = self.now_sec()
        if self.bug_clear_since_sec is None:
            self.bug_clear_since_sec = now_sec
            return False
        return (now_sec - self.bug_clear_since_sec) >= self.bug_release_hold_sec

    def publish_frontier_markers(self, active_waypoint: tuple[float, float]):
        if not self.publish_target_markers or self.current_frontier is None:
            return

        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        markers.markers.append(self.delete_all_marker())
        if self.bug_active:
            node_color = (1.0, 0.10, 0.10, 0.95)
            line_color = (1.0, 0.20, 0.20, 0.85)
            active_color = (1.0, 0.0, 0.0, 1.0)
            goal_color = (0.85, 0.0, 0.0, 0.95)
        elif self.current_frontier.target_reason == 'unvisited_seen_free':
            node_color = (0.10, 0.35, 1.0, 0.90)
            line_color = (0.10, 0.45, 1.0, 0.80)
            active_color = (0.0, 0.30, 1.0, 1.0)
            goal_color = (0.0, 0.20, 0.95, 0.95)
        elif self.current_frontier.target_reason == 'frontier':
            node_color = (0.10, 0.80, 0.20, 0.90)
            line_color = (0.0, 0.75, 0.20, 0.80)
            active_color = (0.0, 1.0, 0.25, 1.0)
            goal_color = (0.0, 0.75, 0.10, 0.95)
        else:
            node_color = (1.0, 0.10, 0.10, 0.90)
            line_color = (1.0, 0.20, 0.20, 0.80)
            active_color = (1.0, 0.0, 0.0, 1.0)
            goal_color = (0.85, 0.0, 0.0, 0.95)

        remaining_path = self.current_frontier.path[self.current_path_index:]
        if remaining_path:
            path_nodes = self.make_marker('frontier_nodes', 1, Marker.SPHERE_LIST, now)
            path_nodes.scale.x = self.target_marker_scale_m * 0.65
            path_nodes.scale.y = self.target_marker_scale_m * 0.65
            path_nodes.scale.z = self.target_marker_scale_m * 0.65
            self.set_marker_color(path_nodes, node_color)
            path_nodes.points = [self.point_from_xy(x_m, y_m) for x_m, y_m in remaining_path]
            markers.markers.append(path_nodes)

            path_line = self.make_marker('frontier_path', 2, Marker.LINE_STRIP, now)
            path_line.scale.x = max(0.02, self.target_marker_scale_m * 0.25)
            self.set_marker_color(path_line, line_color)
            path_line.points = [self.point_from_xy(x_m, y_m) for x_m, y_m in remaining_path]
            markers.markers.append(path_line)

        active = self.make_marker('active_target', 3, Marker.SPHERE, now)
        active.pose.position = self.point_from_xy(active_waypoint[0], active_waypoint[1])
        active.scale.x = self.target_marker_scale_m
        active.scale.y = self.target_marker_scale_m
        active.scale.z = self.target_marker_scale_m
        self.set_marker_color(active, active_color)
        markers.markers.append(active)

        final = self.make_marker('frontier_goal', 4, Marker.CUBE, now)
        final.pose.position = self.point_from_xy(
            self.current_frontier.x_m,
            self.current_frontier.y_m,
        )
        final.scale.x = self.target_marker_scale_m * 1.25
        final.scale.y = self.target_marker_scale_m * 1.25
        final.scale.z = self.target_marker_scale_m * 0.45
        self.set_marker_color(final, goal_color)
        markers.markers.append(final)

        viewpoint = self.make_marker('reachable_viewpoint', 7, Marker.CYLINDER, now)
        viewpoint.pose.position = self.point_from_xy(
            self.current_frontier.x_m,
            self.current_frontier.y_m,
        )
        viewpoint.scale.x = self.target_marker_scale_m * 1.7
        viewpoint.scale.y = self.target_marker_scale_m * 1.7
        viewpoint.scale.z = self.target_marker_scale_m * 0.25
        self.set_marker_color(viewpoint, goal_color)
        markers.markers.append(viewpoint)

        self.append_frontier_memory_markers(markers, now)

        self.marker_pub.publish(markers)

    def set_marker_color(self, marker: Marker, color: tuple[float, float, float, float]):
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

    def make_marker(self, namespace: str, marker_id: int, marker_type: int, stamp):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        # Keep targets visible in RViz until the explorer replaces or clears them.
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        return marker

    def clear_target_markers(self):
        if not hasattr(self, 'marker_pub'):
            return
        markers = MarkerArray()
        markers.markers.append(self.delete_all_marker())
        self.marker_pub.publish(markers)

    def publish_suppressed_markers(self):
        if (
            not self.publish_target_markers
            or not hasattr(self, 'marker_pub')
        ):
            return

        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        markers.markers.append(self.delete_all_marker())
        self.append_frontier_memory_markers(markers, now)
        self.marker_pub.publish(markers)

    def publish_reactive_markers(self, choice: DirectionScore | None):
        if not self.publish_target_markers or not hasattr(self, 'marker_pub'):
            return

        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        markers.markers.append(self.delete_all_marker())
        self.append_frontier_memory_markers(markers, now)

        robot_pose = self.robot_pose_in_map()
        if choice is not None and robot_pose is not None and self.latest_scan is not None:
            robot_x, robot_y, robot_yaw = robot_pose
            body_angle = self.scan_angle_to_body_angle(
                self.latest_scan,
                choice.angle_rad,
            )
            map_angle = robot_yaw + body_angle
            length_m = max(0.15, min(choice.mean_range_m, self.max_usable_range_m, 1.0))
            target_x = robot_x + length_m * math.cos(map_angle)
            target_y = robot_y + length_m * math.sin(map_angle)

            heading = self.make_marker('reactive_heading', 8, Marker.LINE_STRIP, now)
            heading.scale.x = max(0.02, self.target_marker_scale_m * 0.25)
            self.set_marker_color(heading, (1.0, 0.10, 0.10, 0.90))
            heading.points = [
                self.point_from_xy(robot_x, robot_y),
                self.point_from_xy(target_x, target_y),
            ]
            markers.markers.append(heading)

            target = self.make_marker('reactive_target', 9, Marker.SPHERE, now)
            target.pose.position = self.point_from_xy(target_x, target_y)
            target.scale.x = self.target_marker_scale_m
            target.scale.y = self.target_marker_scale_m
            target.scale.z = self.target_marker_scale_m
            self.set_marker_color(target, (1.0, 0.0, 0.0, 0.95))
            markers.markers.append(target)

        self.marker_pub.publish(markers)

    def append_frontier_memory_markers(self, markers: MarkerArray, now):
        if self.suppressed_frontiers:
            suppressed = self.make_marker('suppressed_frontiers', 5, Marker.SPHERE_LIST, now)
            suppressed.scale.x = self.target_marker_scale_m * 0.5
            suppressed.scale.y = self.target_marker_scale_m * 0.5
            suppressed.scale.z = self.target_marker_scale_m * 0.5
            suppressed.color.r = 1.0
            suppressed.color.g = 0.1
            suppressed.color.b = 0.1
            suppressed.color.a = 0.65
            suppressed.points = [
                self.point_from_xy(frontier.x_m, frontier.y_m)
                for frontier in self.suppressed_frontiers
            ]
            markers.markers.append(suppressed)

        if self.rejected_frontiers:
            rejected = self.make_marker('rejected_candidates', 6, Marker.CUBE_LIST, now)
            rejected.scale.x = self.target_marker_scale_m * 0.45
            rejected.scale.y = self.target_marker_scale_m * 0.45
            rejected.scale.z = self.target_marker_scale_m * 0.45
            rejected.color.r = 1.0
            rejected.color.g = 0.45
            rejected.color.b = 0.0
            rejected.color.a = 0.75
            rejected.points = [
                self.point_from_xy(frontier.x_m, frontier.y_m)
                for frontier in self.rejected_frontiers
            ]
            markers.markers.append(rejected)

    def point_from_xy(self, x_m: float, y_m: float):
        point = Point()
        point.x = float(x_m)
        point.y = float(y_m)
        point.z = 0.05
        return point

    def delete_all_marker(self):
        marker = Marker()
        marker.action = Marker.DELETEALL
        return marker

    def plan_frontier_target(self):
        if self.current_frontier is not None:
            return

        self.prune_suppressed_frontiers()
        candidate = self.find_frontier_target()
        self.last_frontier_plan_time = self.get_clock().now()
        if candidate is None:
            if self.current_frontier is not None:
                self.get_logger().info(
                    'No new reachable frontier found; keeping current target for recovery.',
                    throttle_duration_sec=4.0,
                )
            return

        self.stop_bug_recovery()
        self.current_frontier = candidate
        self.current_path_index = 0
        self.reset_frontier_progress()
        self.get_logger().info(
            f'New BFS target: reason={self.current_frontier.target_reason} '
            f'({self.current_frontier.x_m:.2f}, {self.current_frontier.y_m:.2f}) '
            f'clearance={self.current_frontier.clearance_m:.2f} m, '
            f'adjacent_unknown={self.current_frontier.unknown_visible_cells}, '
            f'path={self.current_frontier.path_length_m:.2f} m, '
            f'searched={self.current_frontier.searched_cells} cells, '
            f'waypoints={len(self.current_frontier.path)}.'
        )

    def advance_frontier_waypoint(self) -> bool:
        if self.current_frontier is None:
            return False
        next_index = self.current_path_index + 1
        if next_index >= len(self.current_frontier.path):
            return False
        self.current_path_index = next_index
        self.reset_frontier_progress()
        return True

    def clear_current_frontier(self):
        self.stop_bug_recovery()
        self.current_frontier = None
        self.current_path_index = 0
        self.reset_frontier_progress()
        self.last_frontier_plan_time = None

    def reset_frontier_progress(self):
        self.frontier_progress_best_distance_m = None
        self.frontier_progress_last_time_sec = self.now_sec()
        self.frontier_progress_waypoint_index = self.current_path_index

    def frontier_progress_timed_out(self, distance_m: float) -> bool:
        now_sec = self.now_sec()
        if (
            self.frontier_progress_best_distance_m is None
            or self.frontier_progress_waypoint_index != self.current_path_index
            or distance_m
            < self.frontier_progress_best_distance_m - self.frontier_progress_epsilon_m
        ):
            self.frontier_progress_best_distance_m = distance_m
            self.frontier_progress_last_time_sec = now_sec
            self.frontier_progress_waypoint_index = self.current_path_index
            return False

        elapsed_sec = now_sec - self.frontier_progress_last_time_sec
        return elapsed_sec >= self.frontier_progress_timeout_sec

    def suppress_current_frontier(self, reason: str):
        frontier = self.current_frontier
        if frontier is None:
            self.clear_current_frontier()
            return

        self.remember_current_frontier_path(reason)

        motion_failure_reasons = {
            'no_progress',
            'bug_no_progress',
            'bug_timeout',
        }
        if (
            self.suppress_only_after_motion_failure
            and reason not in motion_failure_reasons
        ):
            self.get_logger().info(
                f'Clearing frontier after {reason} without suppression memory.',
                throttle_duration_sec=2.0,
            )
            self.clear_current_frontier()
            return

        if not self.frontier_failure_memory_enabled:
            self.clear_current_frontier()
            return

        self.prune_suppressed_frontiers()
        duration_sec = self.frontier_suppression_duration_sec
        suppressed_count = 0
        if duration_sec > 0.0 and self.frontier_suppression_radius_m > 0.0:
            expires_at_sec = self.now_sec() + duration_sec
            if frontier.frontier_cells and self.latest_map is not None:
                self.suppress_frontier_region(
                    self.latest_map,
                    list(frontier.frontier_cells),
                    reason,
                    duration_sec,
                )
            suppressed_count += self.add_suppressed_frontier_area(
                frontier.x_m,
                frontier.y_m,
                expires_at_sec,
                reason,
            )
            self.get_logger().info(
                f'Suppressing frontier viewpoint near ({frontier.x_m:.2f}, '
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
                **self.suppressed_frontier_stats(x_m, y_m),
            )
        )
        return 1

    def suppressed_frontier_stats(self, x_m: float, y_m: float) -> dict:
        if self.latest_map is None:
            return {}
        stats = self.local_map_stats(
            self.latest_map,
            x_m,
            y_m,
            max(self.frontier_suppression_radius_m, self.robot_radius_m),
        )
        if stats is None:
            return {}
        free_count, unknown_count, occupied_count = stats
        return {
            'free_count': free_count,
            'unknown_count': unknown_count,
            'occupied_count': occupied_count,
        }

    def local_map_stats(self, grid: OccupancyGrid, x_m: float, y_m: float, radius_m: float):
        center = self.world_to_grid(grid, x_m, y_m)
        if center is None or grid.info.resolution <= 0.0:
            return None
        radius_cells = max(1, int(math.ceil(radius_m / grid.info.resolution)))
        radius_sq = radius_cells * radius_cells
        free_count = 0
        unknown_count = 0
        occupied_count = 0
        for y_value in range(
            max(0, center[1] - radius_cells),
            min(grid.info.height, center[1] + radius_cells + 1),
        ):
            for x_value in range(
                max(0, center[0] - radius_cells),
                min(grid.info.width, center[0] + radius_cells + 1),
            ):
                dx = x_value - center[0]
                dy = y_value - center[1]
                if dx * dx + dy * dy > radius_sq:
                    continue
                value = grid.data[self.grid_index(x_value, y_value, grid.info.width)]
                if value < 0:
                    unknown_count += 1
                elif value <= self.frontier_free_threshold:
                    free_count += 1
                else:
                    occupied_count += 1
        return free_count, unknown_count, occupied_count

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

    def cmd_is_nonzero(self, cmd: Twist | None) -> bool:
        if cmd is None:
            return False
        return any(
            abs(value) > 1e-6
            for value in (
                cmd.linear.x,
                cmd.linear.y,
                cmd.angular.z,
            )
        )

    def active_target_goal_tolerance(self) -> float:
        if (
            self.current_frontier is not None
            and self.current_frontier.target_reason == 'unvisited_seen_free'
        ):
            return self.unvisited_goal_tolerance_m
        return self.frontier_goal_tolerance_m

    def map_geometry_signature(self, grid: OccupancyGrid):
        origin = grid.info.origin
        return (
            grid.info.width,
            grid.info.height,
            grid.info.resolution,
            origin.position.x,
            origin.position.y,
            origin.position.z,
            origin.orientation.x,
            origin.orientation.y,
            origin.orientation.z,
            origin.orientation.w,
        )

    def ensure_visited_grid(self, grid: OccupancyGrid) -> bool:
        signature = self.map_geometry_signature(grid)
        cell_count = grid.info.width * grid.info.height
        if self.visited_map_signature == signature and len(self.visited_cells) == cell_count:
            return False
        self.visited_map_signature = signature
        self.visited_cells = bytearray(cell_count)
        return True

    def mark_visited_around(self, grid: OccupancyGrid, x_m: float, y_m: float):
        self.ensure_visited_grid(grid)
        center = self.world_to_grid(grid, x_m, y_m)
        if center is None or grid.info.resolution <= 0.0:
            return
        radius_cells = max(0, int(math.ceil(self.visited_radius_m / grid.info.resolution)))
        radius_sq = radius_cells * radius_cells
        for y_value in range(
            max(0, center[1] - radius_cells),
            min(grid.info.height, center[1] + radius_cells + 1),
        ):
            for x_value in range(
                max(0, center[0] - radius_cells),
                min(grid.info.width, center[0] + radius_cells + 1),
            ):
                dx = x_value - center[0]
                dy = y_value - center[1]
                if dx * dx + dy * dy <= radius_sq:
                    self.visited_cells[self.grid_index(x_value, y_value, grid.info.width)] = 1

    def prune_suppressed_frontiers(self):
        now_sec = self.now_sec()
        if self.rejected_frontiers:
            self.rejected_frontiers = [
                frontier
                for frontier in self.rejected_frontiers
                if frontier.expires_at_sec > now_sec
            ]
        if not self.suppressed_frontiers:
            return
        retained = []
        for frontier in self.suppressed_frontiers:
            if frontier.expires_at_sec > now_sec:
                retained.append(frontier)
                continue
            if self.suppressed_frontier_region_changed(frontier):
                continue
            retained.append(
                SuppressedFrontier(
                    x_m=frontier.x_m,
                    y_m=frontier.y_m,
                    expires_at_sec=now_sec + max(5.0, self.frontier_suppression_duration_sec),
                    reason=frontier.reason,
                    free_count=frontier.free_count,
                    unknown_count=frontier.unknown_count,
                    occupied_count=frontier.occupied_count,
                )
            )
        self.suppressed_frontiers = retained

    def suppressed_frontier_region_changed(self, frontier: SuppressedFrontier) -> bool:
        if (
            self.latest_map is None
            or frontier.free_count < 0
            or frontier.unknown_count < 0
            or frontier.occupied_count < 0
        ):
            return True
        current = self.local_map_stats(
            self.latest_map,
            frontier.x_m,
            frontier.y_m,
            max(self.frontier_suppression_radius_m, self.robot_radius_m),
        )
        if current is None:
            return True
        free_count, unknown_count, occupied_count = current
        total_before = max(
            1,
            frontier.free_count + frontier.unknown_count + frontier.occupied_count,
        )
        changed = (
            abs(free_count - frontier.free_count)
            + abs(unknown_count - frontier.unknown_count)
            + abs(occupied_count - frontier.occupied_count)
        )
        return (changed / total_before) >= 0.25

    def frontier_is_suppressed(self, x_m: float, y_m: float) -> bool:
        if (
            not self.frontier_failure_memory_enabled
            or not self.suppressed_frontiers
            or self.frontier_suppression_radius_m <= 0.0
        ):
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

    def remember_current_frontier_path(self, reason: str):
        frontier = self.current_frontier
        if frontier is None or not frontier.path or self.recent_path_memory_size <= 0:
            return
        self.recent_frontier_paths.append(
            RecentFrontierPath(
                path=frontier.path,
                recorded_at_sec=self.now_sec(),
                reason=reason,
            )
        )
        self.trim_recent_frontier_paths()

    def trim_recent_frontier_paths(self):
        keep = max(0, getattr(self, 'recent_path_memory_size', 0))
        if keep <= 0:
            self.recent_frontier_paths = []
            return
        self.recent_frontier_paths = self.recent_frontier_paths[-keep:]

    def frontier_path_recently_tried(
        self,
        path: tuple[tuple[float, float], ...],
    ) -> bool:
        if (
            self.recent_path_memory_size <= 0
            or self.recent_path_overlap_fraction <= 0.0
            or not path
            or not self.recent_frontier_paths
        ):
            return False

        for recent in self.recent_frontier_paths:
            overlap = self.path_overlap_fraction(path, recent.path)
            if overlap >= self.recent_path_overlap_fraction:
                self.get_logger().info(
                    f'Skipping frontier path that overlaps {overlap:.0%} with recent '
                    f'"{recent.reason}" path.',
                    throttle_duration_sec=2.0,
                )
                return True
        return False

    def path_overlap_fraction(
        self,
        path_a: tuple[tuple[float, float], ...],
        path_b: tuple[tuple[float, float], ...],
    ) -> float:
        if not path_a or not path_b:
            return 0.0
        matched = 0
        tolerance_sq = self.recent_path_point_tolerance_m * self.recent_path_point_tolerance_m
        for ax_m, ay_m in path_a:
            for bx_m, by_m in path_b:
                dx_m = ax_m - bx_m
                dy_m = ay_m - by_m
                if dx_m * dx_m + dy_m * dy_m <= tolerance_sq:
                    matched += 1
                    break
        return matched / max(1, len(path_a))

    def robot_pose_in_map(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
            )
        except TransformException:
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w)
        return translation.x, translation.y, yaw

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
        if grid.info.resolution <= 0.0:
            self.get_logger().warn(
                'Rejecting frontier search because /map resolution is invalid.',
                throttle_duration_sec=2.0,
            )
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

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y
        self.mark_visited_around(grid, robot_x, robot_y)
        planner = self.build_planner_grid(grid)
        start = self.world_to_grid(grid, robot_x, robot_y)
        if start is None:
            return None

        width = grid.info.width
        height = grid.info.height
        start_index = self.grid_index(start[0], start[1], width)
        if not planner.safe[start_index]:
            nearest = self.nearest_safe_cell(planner, start[0], start[1])
            if nearest is None:
                self.get_logger().warn(
                    'Frontier planner cannot find a safe start cell near the robot.',
                    throttle_duration_sec=2.0,
                )
                return None
            start_index = self.grid_index(nearest[0], nearest[1], width)

        bfs_visited = bytearray(width * height)
        parent = [-1] * (width * height)
        path_cost_cells = [math.inf] * (width * height)
        pending = deque([start_index])
        bfs_visited[start_index] = 1
        parent[start_index] = start_index
        path_cost_cells[start_index] = 0.0
        searched = 0
        frontier_index = -1
        unvisited_index = -1

        while pending:
            current = pending.popleft()
            searched += 1

            path_length_m = path_cost_cells[current] * grid.info.resolution
            if current != start_index:
                if self.is_frontier_cell(grid, planner, current):
                    frontier_index = current
                    break
                if (
                    unvisited_index < 0
                    and not self.visited_cells[current]
                    and path_length_m >= self.unvisited_goal_min_distance_m
                    and path_length_m <= self.unvisited_goal_max_distance_m
                ):
                    unvisited_index = current

            for neighbor in self.safe_neighbor_indices(current, planner):
                if bfs_visited[neighbor]:
                    continue
                bfs_visited[neighbor] = 1
                parent[neighbor] = current
                step_cost = self.grid_step_cost(current, neighbor, width)
                path_cost_cells[neighbor] = path_cost_cells[current] + step_cost
                pending.append(neighbor)

        if frontier_index >= 0:
            target_index = frontier_index
            target_reason = 'frontier'
        elif unvisited_index >= 0:
            target_index = unvisited_index
            target_reason = 'unvisited_seen_free'
        else:
            return None

        path = self.reconstruct_world_path(grid, parent, start_index, target_index)
        if not path:
            return None

        target_x = target_index % width
        target_y = target_index // width
        target_world = self.grid_to_world(grid, target_x, target_y)
        unknown_neighbors = self.count_unknown_neighbors(grid, target_index)
        path_length_m = path_cost_cells[target_index] * grid.info.resolution
        frontier_cells = (target_index,) if target_reason == 'frontier' else ()
        return FrontierTarget(
            x_m=target_world[0],
            y_m=target_world[1],
            grid_x=target_x,
            grid_y=target_y,
            searched_cells=searched,
            path=path,
            target_reason=target_reason,
            frontier_cells=frontier_cells,
            frontier_area_cells=1 if target_reason == 'frontier' else 0,
            clearance_m=planner.clearance_m[target_index],
            path_length_m=path_length_m,
            unknown_visible_cells=unknown_neighbors,
            score=-path_length_m,
        )

    def build_planner_grid(self, grid: OccupancyGrid) -> PlannerGrid:
        width = grid.info.width
        height = grid.info.height
        cell_count = width * height
        free = bytearray(cell_count)
        occupied = bytearray(cell_count)
        unknown = bytearray(cell_count)
        danger = bytearray(cell_count)

        for index, value in enumerate(grid.data):
            if value < 0:
                unknown[index] = 1
            elif value <= self.frontier_free_threshold:
                free[index] = 1
            elif value >= self.frontier_occupied_threshold:
                occupied[index] = 1
                danger[index] = 1
            else:
                danger[index] = 1

        margin = self.frontier_unknown_margin_cells
        if margin > 0:
            occupied_indices = [index for index, value in enumerate(occupied) if value]
            for occupied_index in occupied_indices:
                ox = occupied_index % width
                oy = occupied_index // width
                for dy in range(-margin, margin + 1):
                    for dx in range(-margin, margin + 1):
                        nx = ox + dx
                        ny = oy + dy
                        if 0 <= nx < width and 0 <= ny < height:
                            neighbor = self.grid_index(nx, ny, width)
                            if unknown[neighbor]:
                                danger[neighbor] = 1

        clearance_m = self.clearance_distance_map(grid, danger)
        path_required_clearance_m = max(
            self.frontier_min_clearance_m,
            self.path_clearance_m,
            self.robot_radius_m + self.planner_safety_margin_m,
        )
        goal_required_clearance_m = max(
            self.frontier_min_clearance_m,
            self.goal_clearance_m,
            self.robot_radius_m,
        )
        path_safe = bytearray(cell_count)
        goal_safe = bytearray(cell_count)
        for index in range(cell_count):
            if free[index] and clearance_m[index] > path_required_clearance_m:
                path_safe[index] = 1
            if free[index] and clearance_m[index] > goal_required_clearance_m:
                goal_safe[index] = 1

        return PlannerGrid(
            width=width,
            height=height,
            resolution=grid.info.resolution,
            free=free,
            danger=danger,
            path_safe=path_safe,
            goal_safe=goal_safe,
            safe=path_safe,
            clearance_m=clearance_m,
        )

    def clearance_distance_map(self, grid: OccupancyGrid, danger: bytearray) -> list[float]:
        width = grid.info.width
        height = grid.info.height
        distances = [math.inf] * (width * height)
        pending = []
        for index, is_danger in enumerate(danger):
            if is_danger:
                distances[index] = 0.0
                heapq.heappush(pending, (0.0, index))

        if not pending:
            return [self.max_usable_range_m] * (width * height)

        while pending:
            current_distance, current = heapq.heappop(pending)
            if current_distance > distances[current]:
                continue

            current_x = current % width
            current_y = current // width
            for dy, dx, step_cost in (
                (0, 1, 1.0),
                (1, 0, 1.0),
                (0, -1, 1.0),
                (-1, 0, 1.0),
                (1, 1, math.sqrt(2.0)),
                (1, -1, math.sqrt(2.0)),
                (-1, -1, math.sqrt(2.0)),
                (-1, 1, math.sqrt(2.0)),
            ):
                nx = current_x + dx
                ny = current_y + dy
                if not (0 <= nx < width and 0 <= ny < height):
                    continue
                neighbor = self.grid_index(nx, ny, width)
                next_distance = current_distance + step_cost
                if next_distance < distances[neighbor]:
                    distances[neighbor] = next_distance
                    heapq.heappush(pending, (next_distance, neighbor))

        return [
            min(distance * grid.info.resolution, self.max_usable_range_m)
            for distance in distances
        ]

    def connected_frontier_components(
        self,
        grid: OccupancyGrid,
        planner: PlannerGrid,
    ) -> list[list[int]]:
        width = planner.width
        height = planner.height
        frontier = bytearray(width * height)
        for index in range(width * height):
            if self.is_frontier_cell(grid, planner, index):
                frontier[index] = 1

        visited = bytearray(width * height)
        components = []
        for index, is_frontier in enumerate(frontier):
            if not is_frontier or visited[index]:
                continue
            pending = deque([index])
            visited[index] = 1
            component = []
            while pending:
                current = pending.popleft()
                component.append(current)
                for neighbor in self.cardinal_neighbor_indices(current, width, height):
                    if frontier[neighbor] and not visited[neighbor]:
                        visited[neighbor] = 1
                        pending.append(neighbor)
            if len(component) >= self.frontier_min_area_cells:
                components.append(component)
            else:
                self.add_rejected_frontier_debug(grid, component, 'tiny_frontier')
        return components

    def project_frontier_component_to_goal(
        self,
        grid: OccupancyGrid,
        planner: PlannerGrid,
        component: list[int],
        reachable: bytearray,
        path_cost_cells: list[float],
    ):
        width = planner.width
        projection_cells = max(
            1,
            int(math.ceil(self.frontier_goal_projection_radius_m / planner.resolution)),
        )
        standoff_cells = max(
            0.0,
            self.frontier_standoff_distance_m / planner.resolution,
        )
        candidate_distances = {}
        for frontier_index in component:
            fx = frontier_index % width
            fy = frontier_index // width
            for dy in range(-projection_cells, projection_cells + 1):
                for dx in range(-projection_cells, projection_cells + 1):
                    distance_cells = math.hypot(dx, dy)
                    if distance_cells > projection_cells:
                        continue
                    nx = fx + dx
                    ny = fy + dy
                    if not (0 <= nx < planner.width and 0 <= ny < planner.height):
                        continue
                    index = self.grid_index(nx, ny, width)
                    if not planner.goal_safe[index]:
                        continue
                    candidate_distances[index] = min(
                        candidate_distances.get(index, math.inf),
                        distance_cells,
                    )

        if not candidate_distances:
            return None, 'rejected: no safe projected viewpoint'

        candidates = []
        min_standoff_cells = max(1.0, standoff_cells * 0.25)
        for index, distance_to_frontier_cells in candidate_distances.items():
            if distance_to_frontier_cells < min_standoff_cells:
                continue
            parent_index = self.reachable_goal_parent(index, planner, reachable)
            if parent_index < 0:
                continue
            effective_cost = path_cost_cells[index]
            if not math.isfinite(effective_cost):
                effective_cost = path_cost_cells[parent_index] + self.grid_step_cost(
                    parent_index,
                    index,
                    width,
                )
            if not math.isfinite(effective_cost):
                continue
            unknown_visible = self.count_visible_unknown_cells(grid, planner, index)
            if unknown_visible < self.unknown_visibility_min_cells:
                continue
            standoff_penalty = abs(distance_to_frontier_cells - standoff_cells)
            path_length_m = effective_cost * planner.resolution
            score = (
                0.18 * min(unknown_visible, 80)
                + 2.0 * min(planner.clearance_m[index], 1.0)
                - 0.50 * path_length_m
                - 0.25 * standoff_penalty * planner.resolution
            )
            candidates.append(
                ViewpointCandidate(
                    index=index,
                    parent_index=parent_index,
                    distance_to_frontier_cells=distance_to_frontier_cells,
                    path_length_m=path_length_m,
                    clearance_m=planner.clearance_m[index],
                    unknown_visible_cells=unknown_visible,
                    score=score,
                )
            )

        if not candidates:
            return None, 'rejected: no path or low information gain'

        candidates.sort(key=lambda item: item.score, reverse=True)
        for candidate in candidates[: self.max_projection_attempts_per_frontier]:
            return candidate, 'ok'

        return None, 'rejected: projection attempts exhausted'

    def reachable_goal_parent(
        self,
        index: int,
        planner: PlannerGrid,
        reachable: bytearray,
    ) -> int:
        if reachable[index] and planner.path_safe[index]:
            return index

        best_neighbor = -1
        best_clearance = -math.inf
        for neighbor in self.cardinal_neighbor_indices(index, planner.width, planner.height):
            if not reachable[neighbor] or not planner.path_safe[neighbor]:
                continue
            clearance = planner.clearance_m[neighbor]
            if clearance > best_clearance:
                best_clearance = clearance
                best_neighbor = neighbor
        return best_neighbor

    def count_visible_unknown_cells(
        self,
        grid: OccupancyGrid,
        planner: PlannerGrid,
        origin_index: int,
    ) -> int:
        radius_cells = max(
            1,
            int(math.ceil(self.unknown_visibility_radius_m / planner.resolution)),
        )
        width = planner.width
        height = planner.height
        ox = origin_index % width
        oy = origin_index // width
        visible = 0
        radius_sq = radius_cells * radius_cells
        for y_value in range(max(0, oy - radius_cells), min(height, oy + radius_cells + 1)):
            for x_value in range(max(0, ox - radius_cells), min(width, ox + radius_cells + 1)):
                dx = x_value - ox
                dy = y_value - oy
                if dx * dx + dy * dy > radius_sq:
                    continue
                index = self.grid_index(x_value, y_value, width)
                if grid.data[index] >= 0:
                    continue
                if self.line_of_sight_to_unknown(grid, ox, oy, x_value, y_value):
                    visible += 1
        return visible

    def line_of_sight_to_unknown(
        self,
        grid: OccupancyGrid,
        start_x: int,
        start_y: int,
        end_x: int,
        end_y: int,
    ) -> bool:
        width = grid.info.width
        dx = abs(end_x - start_x)
        dy = -abs(end_y - start_y)
        step_x = 1 if start_x < end_x else -1
        step_y = 1 if start_y < end_y else -1
        error = dx + dy
        x_value = start_x
        y_value = start_y

        while True:
            if x_value == end_x and y_value == end_y:
                return True
            index = self.grid_index(x_value, y_value, width)
            if grid.data[index] >= self.frontier_occupied_threshold:
                return False
            double_error = 2 * error
            if double_error >= dy:
                error += dy
                x_value += step_x
            if double_error <= dx:
                error += dx
                y_value += step_y

    def path_indices_are_safe(
        self,
        grid: OccupancyGrid,
        planner: PlannerGrid,
        parent: list[int],
        start_index: int,
        goal_index: int,
    ) -> bool:
        current = goal_index
        while current != start_index and current >= 0:
            if current == goal_index:
                if not planner.goal_safe[current]:
                    return False
            elif not planner.path_safe[current]:
                return False
            current = parent[current]
        return current == start_index and planner.path_safe[start_index]

    def component_center_world(self, grid: OccupancyGrid, component: list[int]):
        if not component:
            return 0.0, 0.0
        width = grid.info.width
        sum_x = 0.0
        sum_y = 0.0
        for index in component:
            grid_x = index % width
            grid_y = index // width
            world_x, world_y = self.grid_to_world(grid, grid_x, grid_y)
            sum_x += world_x
            sum_y += world_y
        count = len(component)
        return sum_x / count, sum_y / count

    def suppress_frontier_region(
        self,
        grid: OccupancyGrid,
        component: list[int],
        reason: str,
        duration_sec: float,
    ):
        if not self.frontier_failure_memory_enabled or not component:
            return
        expires_at_sec = self.now_sec() + duration_sec
        center_x, center_y = self.component_center_world(grid, component)
        count = self.add_suppressed_frontier_area(
            center_x,
            center_y,
            expires_at_sec,
            reason,
        )
        stride = max(1, len(component) // 8)
        for index in component[::stride]:
            grid_x = index % grid.info.width
            grid_y = index // grid.info.width
            x_m, y_m = self.grid_to_world(grid, grid_x, grid_y)
            count += self.add_suppressed_frontier_area(x_m, y_m, expires_at_sec, reason)
        self.get_logger().info(
            f'Rejected frontier region near ({center_x:.2f}, {center_y:.2f}) '
            f'after {reason} ({count} suppressed markers).',
            throttle_duration_sec=2.0,
        )

    def add_rejected_frontier_debug(
        self,
        grid: OccupancyGrid,
        component: list[int],
        reason: str,
    ):
        if not self.publish_target_markers or not component:
            return
        center_x, center_y = self.component_center_world(grid, component)
        expires_at_sec = self.now_sec() + 8.0
        dedupe_radius_sq = max(0.05, self.target_marker_scale_m * 2.0) ** 2
        for frontier in self.rejected_frontiers:
            dx_m = center_x - frontier.x_m
            dy_m = center_y - frontier.y_m
            if dx_m * dx_m + dy_m * dy_m <= dedupe_radius_sq:
                return
        self.rejected_frontiers.append(
            SuppressedFrontier(
                x_m=center_x,
                y_m=center_y,
                expires_at_sec=expires_at_sec,
                reason=reason,
            )
        )
        self.get_logger().info(
            f'Frontier candidate near ({center_x:.2f}, {center_y:.2f}) {reason}.',
            throttle_duration_sec=3.0,
        )

    def nearest_safe_cell(self, planner: PlannerGrid, start_x: int, start_y: int):
        width = planner.width
        height = planner.height
        start_index = self.grid_index(start_x, start_y, width)
        visited = bytearray(width * height)
        pending = deque([start_index])
        visited[start_index] = 1
        while pending:
            current = pending.popleft()
            if planner.safe[current]:
                return current % width, current // width
            for neighbor in self.cardinal_neighbor_indices(current, width, height):
                if not visited[neighbor]:
                    visited[neighbor] = 1
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

    def is_frontier_cell(self, grid: OccupancyGrid, planner: PlannerGrid, index: int) -> bool:
        if not planner.safe[index]:
            return False
        width = planner.width
        height = planner.height
        for neighbor in self.cardinal_neighbor_indices(index, width, height):
            if grid.data[neighbor] < 0:
                return True
        return False

    def count_unknown_neighbors(self, grid: OccupancyGrid, index: int) -> int:
        count = 0
        for neighbor in self.cardinal_neighbor_indices(index, grid.info.width, grid.info.height):
            if grid.data[neighbor] < 0:
                count += 1
        return count

    def safe_neighbor_indices(self, index: int, planner: PlannerGrid):
        width = planner.width
        height = planner.height
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
                neighbor = self.grid_index(nx, ny, width)
                if not planner.safe[neighbor]:
                    continue
                if dx_value != 0 and dy_value != 0:
                    side_a = self.grid_index(x_value + dx_value, y_value, width)
                    side_b = self.grid_index(x_value, y_value + dy_value, width)
                    if not planner.safe[side_a] or not planner.safe[side_b]:
                        continue
                yield neighbor

    def cardinal_neighbor_indices(self, index: int, width: int, height: int):
        x_value = index % width
        y_value = index // width
        for dy_value, dx_value in ((0, 1), (1, 0), (0, -1), (-1, 0)):
            nx = x_value + dx_value
            ny = y_value + dy_value
            if 0 <= nx < width and 0 <= ny < height:
                yield self.grid_index(nx, ny, width)

    def grid_step_cost(self, current: int, neighbor: int, width: int) -> float:
        current_x = current % width
        current_y = current // width
        neighbor_x = neighbor % width
        neighbor_y = neighbor // width
        if current_x != neighbor_x and current_y != neighbor_y:
            return math.sqrt(2.0)
        return 1.0

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
        try:
            if rclpy.ok():
                node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
