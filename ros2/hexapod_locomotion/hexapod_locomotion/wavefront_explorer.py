#!/usr/bin/env python3
"""Wavefront Frontier Detection explorer for occupancy-grid exploration."""

from collections import deque
from dataclasses import dataclass
from enum import IntEnum
import heapq
import math

from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class CellClass(IntEnum):
    UNKNOWN = 0
    FREE = 1
    BLOCKED = 2
    FRONTIER = 3


@dataclass(frozen=True)
class MapModel:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    classes: list[CellClass]
    inflated_blocked: bytearray
    clearance_m: list[float]


@dataclass(frozen=True)
class FrontierCluster:
    cells: tuple[int, ...]
    centroid_x: float
    centroid_y: float
    accepted: bool
    reject_reason: str = ''


@dataclass(frozen=True)
class ProjectedGoal:
    cluster: FrontierCluster
    index: int
    path_indices: tuple[int, ...]
    path_length_m: float
    clearance_m: float
    score: float


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def quaternion_to_yaw(x_value: float, y_value: float, z_value: float, w_value: float) -> float:
    siny_cosp = 2.0 * (w_value * z_value + x_value * y_value)
    cosy_cosp = 1.0 - 2.0 * (y_value * y_value + z_value * z_value)
    return math.atan2(siny_cosp, cosy_cosp)


class WavefrontExplorer(Node):
    """Find reachable frontier clusters and drive toward safe projected goals."""

    def __init__(self):
        super().__init__('wavefront_explorer')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('target_marker_topic', '/explorer/targets')
        self.declare_parameter('active_goal_topic', '/explorer/active_goal')
        self.declare_parameter('planned_path_topic', '/explorer/planned_path')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('enabled', True)
        self.declare_parameter('replan_period_sec', 3.0)
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('map_timeout_sec', 2.5)

        self.declare_parameter('free_threshold', 25)
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('robot_radius_m', 0.30)
        self.declare_parameter('safety_margin_m', 0.10)
        self.declare_parameter('min_frontier_cluster_size', 5)
        self.declare_parameter('goal_projection_radius_m', 0.90)

        self.declare_parameter('cluster_size_weight', 1.0)
        self.declare_parameter('path_length_weight', 8.0)
        self.declare_parameter('clearance_weight', 12.0)
        self.declare_parameter('centroid_distance_weight', 2.0)

        self.declare_parameter('goal_tolerance_m', 0.18)
        self.declare_parameter('waypoint_spacing_m', 0.25)
        self.declare_parameter('max_speed_mps', 0.035)
        self.declare_parameter('max_turn_rate_rad_s', 0.18)
        self.declare_parameter('turn_gain', 0.9)
        self.declare_parameter('heading_deadband_rad', 0.20)
        self.declare_parameter('publish_target_markers', True)
        self.declare_parameter('target_marker_scale_m', 0.12)

        self._load_parameters()

        self.latest_map = None
        self.latest_map_time = None
        self.current_model = None
        self.current_goal = None
        self.accepted_clusters = []
        self.rejected_clusters = []
        self.frontier_cells = ()

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.goal_pub = self.create_publisher(PoseStamped, self.active_goal_topic, 10)
        self.path_pub = self.create_publisher(Path, self.planned_path_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.target_marker_topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, 10)
        self.create_timer(self.replan_period_sec, self.plan_timer)
        self.create_timer(1.0 / self.control_rate_hz, self.control_timer)

        self.get_logger().info(
            'Wavefront explorer ready: '
            f'map={self.map_topic}, cmd_vel={self.cmd_vel_topic}, '
            f'goal={self.active_goal_topic}, markers={self.target_marker_topic}'
        )

    def _load_parameters(self):
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.target_marker_topic = str(self.get_parameter('target_marker_topic').value)
        self.active_goal_topic = str(self.get_parameter('active_goal_topic').value)
        self.planned_path_topic = str(self.get_parameter('planned_path_topic').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.enabled = as_bool(self.get_parameter('enabled').value)
        self.replan_period_sec = max(0.2, float(self.get_parameter('replan_period_sec').value))
        self.control_rate_hz = max(1.0, float(self.get_parameter('control_rate_hz').value))
        self.map_timeout_sec = max(0.1, float(self.get_parameter('map_timeout_sec').value))
        self.free_threshold = int(self.get_parameter('free_threshold').value)
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.robot_radius_m = max(0.0, float(self.get_parameter('robot_radius_m').value))
        self.safety_margin_m = max(0.0, float(self.get_parameter('safety_margin_m').value))
        self.min_frontier_cluster_size = max(
            1,
            int(self.get_parameter('min_frontier_cluster_size').value),
        )
        self.goal_projection_radius_m = max(
            0.0,
            float(self.get_parameter('goal_projection_radius_m').value),
        )
        self.cluster_size_weight = float(self.get_parameter('cluster_size_weight').value)
        self.path_length_weight = float(self.get_parameter('path_length_weight').value)
        self.clearance_weight = float(self.get_parameter('clearance_weight').value)
        self.centroid_distance_weight = float(
            self.get_parameter('centroid_distance_weight').value
        )
        self.goal_tolerance_m = max(0.02, float(self.get_parameter('goal_tolerance_m').value))
        self.waypoint_spacing_m = max(0.02, float(self.get_parameter('waypoint_spacing_m').value))
        self.max_speed_mps = max(0.0, float(self.get_parameter('max_speed_mps').value))
        self.max_turn_rate_rad_s = max(
            0.0,
            float(self.get_parameter('max_turn_rate_rad_s').value),
        )
        self.turn_gain = max(0.0, float(self.get_parameter('turn_gain').value))
        self.heading_deadband_rad = max(
            0.0,
            float(self.get_parameter('heading_deadband_rad').value),
        )
        self.publish_target_markers = as_bool(
            self.get_parameter('publish_target_markers').value
        )
        self.target_marker_scale_m = max(
            0.02,
            float(self.get_parameter('target_marker_scale_m').value),
        )

    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg
        self.latest_map_time = self.get_clock().now()

    def plan_timer(self):
        if not self.enabled:
            self.stop_robot()
            return
        if self.latest_map is None or self.map_is_stale():
            return
        robot_pose = self.lookup_robot_pose()
        if robot_pose is None:
            return

        model = self.build_map_model(self.latest_map)
        self.current_model = model
        start = self.world_to_grid(model, robot_pose[0], robot_pose[1])
        if start is None:
            self.get_logger().warn('Robot pose is outside the occupancy grid.')
            return
        start_index = self.to_index(model, start[0], start[1])
        if not self.is_traversable(model, start_index):
            start_index = self.nearest_traversable(model, start_index)
            if start_index is None:
                self.get_logger().warn('No inflated-safe free cell near robot start.')
                return

        goal = self.find_best_goal(model, start_index)
        self.current_goal = goal
        if goal is None:
            self.stop_robot()
            self.publish_path(model, ())
        else:
            self.publish_goal(model, goal)
            self.publish_path(model, tuple(goal.path_indices))
        if self.publish_target_markers:
            self.publish_markers(model, goal)

    def control_timer(self):
        if not self.enabled or self.current_goal is None:
            self.stop_robot()
            return
        if self.latest_map is None or self.map_is_stale():
            self.stop_robot()
            return
        robot_pose = self.lookup_robot_pose()
        if robot_pose is None:
            self.stop_robot()
            return
        if self.current_model is None:
            self.stop_robot()
            return
        goal_x, goal_y = self.grid_center(self.current_model, self.current_goal.index)
        distance = math.hypot(goal_x - robot_pose[0], goal_y - robot_pose[1])
        if distance <= self.goal_tolerance_m:
            self.stop_robot()
            return

        target_x, target_y = self.next_waypoint(robot_pose, self.current_goal)
        heading = math.atan2(target_y - robot_pose[1], target_x - robot_pose[0])
        heading_error = math.atan2(
            math.sin(heading - robot_pose[2]),
            math.cos(heading - robot_pose[2]),
        )

        cmd = Twist()
        cmd.angular.z = clamp(
            self.turn_gain * heading_error,
            -self.max_turn_rate_rad_s,
            self.max_turn_rate_rad_s,
        )
        speed_scale = max(0.0, math.cos(heading_error))
        if abs(heading_error) < self.heading_deadband_rad:
            speed_scale = 1.0
        cmd.linear.x = self.max_speed_mps * speed_scale
        self.cmd_pub.publish(cmd)

    def next_waypoint(self, robot_pose, goal: ProjectedGoal):
        model = self.current_model
        for index in goal.path_indices:
            x_value, y_value = self.grid_center(model, index)
            if math.hypot(x_value - robot_pose[0], y_value - robot_pose[1]) > (
                self.waypoint_spacing_m
            ):
                return x_value, y_value
        return self.grid_center(model, goal.index)

    def map_is_stale(self) -> bool:
        age = (self.get_clock().now() - self.latest_map_time).nanoseconds / 1e9
        return age > self.map_timeout_sec

    def lookup_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
            )
        except TransformException as exc:
            self.get_logger().debug(f'TF lookup failed: {exc}')
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w)
        return translation.x, translation.y, yaw

    def build_map_model(self, msg: OccupancyGrid) -> MapModel:
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        classes = [CellClass.UNKNOWN] * (width * height)

        for index, occupancy in enumerate(msg.data):
            if occupancy == -1:
                classes[index] = CellClass.UNKNOWN
            elif occupancy >= self.occupied_threshold:
                classes[index] = CellClass.BLOCKED
            elif 0 <= occupancy <= self.free_threshold:
                classes[index] = CellClass.FREE
            else:
                classes[index] = CellClass.BLOCKED

        for index, cell_class in enumerate(tuple(classes)):
            if cell_class != CellClass.FREE:
                continue
            if any(classes[neighbor] == CellClass.UNKNOWN for neighbor in self.neighbors4_index(
                width,
                height,
                index,
            )):
                classes[index] = CellClass.FRONTIER

        clearance = self.compute_clearance(width, height, resolution, classes)
        inflation_radius = self.robot_radius_m + self.safety_margin_m
        inflated = bytearray(width * height)
        for index, cell_class in enumerate(classes):
            if cell_class == CellClass.BLOCKED or clearance[index] < inflation_radius:
                inflated[index] = 1

        return MapModel(
            width=width,
            height=height,
            resolution=resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
            classes=classes,
            inflated_blocked=inflated,
            clearance_m=clearance,
        )

    def compute_clearance(self, width, height, resolution, classes):
        max_distance = float('inf')
        clearance = [max_distance] * (width * height)
        queue = []
        for index, cell_class in enumerate(classes):
            if cell_class == CellClass.BLOCKED:
                clearance[index] = 0.0
                heapq.heappush(queue, (0.0, index))
        if not queue:
            return clearance
        while queue:
            distance, index = heapq.heappop(queue)
            if distance != clearance[index]:
                continue
            x_value = index % width
            y_value = index // width
            for nx_value, ny_value, step in self.neighbors8(width, height, x_value, y_value):
                neighbor = ny_value * width + nx_value
                next_distance = distance + step * resolution
                if next_distance < clearance[neighbor]:
                    clearance[neighbor] = next_distance
                    heapq.heappush(queue, (next_distance, neighbor))
        return clearance

    def find_best_goal(self, model: MapModel, start_index: int):
        parents = {start_index: None}
        distances = {start_index: 0.0}
        map_open = deque([start_index])
        clustered = set()
        reachable = []
        goals = []
        self.accepted_clusters = []
        self.rejected_clusters = []
        self.frontier_cells = ()

        while map_open:
            index = map_open.popleft()
            reachable.append(index)
            for neighbor in self.neighbors4_index(model.width, model.height, index):
                cell_class = model.classes[neighbor]
                if cell_class == CellClass.FRONTIER:
                    if neighbor in clustered:
                        continue
                    cluster = self.grow_frontier_cluster(model, neighbor, clustered)
                    self.frontier_cells += cluster.cells
                    if not cluster.accepted:
                        self.rejected_clusters.append(cluster)
                        continue
                    self.accepted_clusters.append(cluster)
                    goal = self.project_cluster_goal(model, cluster, parents, distances)
                    if goal is None:
                        self.rejected_clusters.append(
                            FrontierCluster(
                                cluster.cells,
                                cluster.centroid_x,
                                cluster.centroid_y,
                                False,
                                'no reachable safe projection',
                            )
                        )
                    else:
                        goals.append(goal)
                    continue
                if neighbor in parents or not self.is_traversable(model, neighbor):
                    continue
                parents[neighbor] = index
                distances[neighbor] = distances[index] + model.resolution
                map_open.append(neighbor)

        if not goals:
            return None
        return max(goals, key=lambda candidate: candidate.score)

    def grow_frontier_cluster(self, model: MapModel, seed: int, clustered: set[int]):
        queue = deque([seed])
        clustered.add(seed)
        cells = []
        while queue:
            index = queue.popleft()
            cells.append(index)
            for neighbor in self.neighbors4_index(model.width, model.height, index):
                if neighbor in clustered or model.classes[neighbor] != CellClass.FRONTIER:
                    continue
                clustered.add(neighbor)
                queue.append(neighbor)

        centroid_x = sum(index % model.width for index in cells) / len(cells)
        centroid_y = sum(index // model.width for index in cells) / len(cells)
        if len(cells) < self.min_frontier_cluster_size:
            return FrontierCluster(
                tuple(cells),
                centroid_x,
                centroid_y,
                False,
                'cluster too small',
            )
        return FrontierCluster(tuple(cells), centroid_x, centroid_y, True)

    def project_cluster_goal(self, model, cluster, parents, distances):
        radius_cells = int(math.ceil(self.goal_projection_radius_m / model.resolution))
        centroid_x = int(round(cluster.centroid_x))
        centroid_y = int(round(cluster.centroid_y))
        best = None

        for dy_value in range(-radius_cells, radius_cells + 1):
            for dx_value in range(-radius_cells, radius_cells + 1):
                x_value = centroid_x + dx_value
                y_value = centroid_y + dy_value
                if not self.in_bounds(model, x_value, y_value):
                    continue
                index = self.to_index(model, x_value, y_value)
                if index not in parents or not self.is_traversable(model, index):
                    continue
                centroid_distance = math.hypot(
                    (x_value - cluster.centroid_x) * model.resolution,
                    (y_value - cluster.centroid_y) * model.resolution,
                )
                if centroid_distance > self.goal_projection_radius_m:
                    continue
                path_indices = self.reconstruct_path(parents, index)
                score = (
                    self.cluster_size_weight * len(cluster.cells)
                    - self.path_length_weight * distances[index]
                    + self.clearance_weight * model.clearance_m[index]
                    - self.centroid_distance_weight * centroid_distance
                )
                candidate = ProjectedGoal(
                    cluster=cluster,
                    index=index,
                    path_indices=path_indices,
                    path_length_m=distances[index],
                    clearance_m=model.clearance_m[index],
                    score=score,
                )
                if best is None or candidate.score > best.score:
                    best = candidate
        return best

    def reconstruct_path(self, parents, index):
        path = []
        while index is not None:
            path.append(index)
            index = parents[index]
        path.reverse()
        return tuple(path)

    def nearest_traversable(self, model: MapModel, start_index: int):
        visited = {start_index}
        queue = deque([start_index])
        while queue:
            index = queue.popleft()
            if self.is_traversable(model, index):
                return index
            for neighbor in self.neighbors4_index(model.width, model.height, index):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
        return None

    def is_traversable(self, model: MapModel, index: int) -> bool:
        return (
            model.classes[index] == CellClass.FREE
            and model.inflated_blocked[index] == 0
        )

    def world_to_grid(self, model: MapModel, x_value: float, y_value: float):
        grid_x = int(math.floor((x_value - model.origin_x) / model.resolution))
        grid_y = int(math.floor((y_value - model.origin_y) / model.resolution))
        if not self.in_bounds(model, grid_x, grid_y):
            return None
        return grid_x, grid_y

    def grid_center(self, model: MapModel, index: int):
        x_value = index % model.width
        y_value = index // model.width
        return (
            model.origin_x + (x_value + 0.5) * model.resolution,
            model.origin_y + (y_value + 0.5) * model.resolution,
        )

    def to_index(self, model: MapModel, x_value: int, y_value: int) -> int:
        return y_value * model.width + x_value

    def in_bounds(self, model: MapModel, x_value: int, y_value: int) -> bool:
        return 0 <= x_value < model.width and 0 <= y_value < model.height

    def neighbors4_index(self, width: int, height: int, index: int):
        x_value = index % width
        y_value = index // width
        if x_value > 0:
            yield index - 1
        if x_value + 1 < width:
            yield index + 1
        if y_value > 0:
            yield index - width
        if y_value + 1 < height:
            yield index + width

    def neighbors8(self, width: int, height: int, x_value: int, y_value: int):
        for dy_value in (-1, 0, 1):
            for dx_value in (-1, 0, 1):
                if dx_value == 0 and dy_value == 0:
                    continue
                nx_value = x_value + dx_value
                ny_value = y_value + dy_value
                if 0 <= nx_value < width and 0 <= ny_value < height:
                    yield nx_value, ny_value, math.hypot(dx_value, dy_value)

    def publish_goal(self, model: MapModel, goal: ProjectedGoal):
        x_value, y_value = self.grid_center(model, goal.index)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = x_value
        msg.pose.position.y = y_value
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

    def publish_path(self, model: MapModel, path_indices):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        for index in path_indices:
            x_value, y_value = self.grid_center(model, index)
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x_value
            pose.pose.position.y = y_value
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def publish_markers(self, model: MapModel, goal: ProjectedGoal | None):
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        markers.markers.append(self.delete_all_marker())

        frontiers = self.make_marker('wfd', 1, Marker.CUBE_LIST, now)
        self.set_marker_color(frontiers, (0.0, 0.35, 1.0, 0.85))
        frontiers.scale.x = model.resolution
        frontiers.scale.y = model.resolution
        frontiers.scale.z = 0.025
        frontiers.points = [self.point_for_index(model, index) for index in self.frontier_cells]
        markers.markers.append(frontiers)

        centroids = self.make_marker('wfd', 2, Marker.SPHERE_LIST, now)
        self.set_marker_color(centroids, (1.0, 0.9, 0.0, 0.95))
        centroids.scale.x = self.target_marker_scale_m
        centroids.scale.y = self.target_marker_scale_m
        centroids.scale.z = self.target_marker_scale_m
        centroids.points = [
            self.point_for_grid(model, cluster.centroid_x, cluster.centroid_y, 0.08)
            for cluster in self.accepted_clusters
        ]
        markers.markers.append(centroids)

        rejected = self.make_marker('wfd', 3, Marker.CUBE_LIST, now)
        self.set_marker_color(rejected, (1.0, 0.28, 0.0, 0.9))
        rejected.scale.x = model.resolution
        rejected.scale.y = model.resolution
        rejected.scale.z = 0.035
        rejected.points = [
            self.point_for_index(model, index, 0.04)
            for cluster in self.rejected_clusters
            for index in cluster.cells
        ]
        markers.markers.append(rejected)

        if goal is not None:
            selected = self.make_marker('wfd', 4, Marker.SPHERE, now)
            self.set_marker_color(selected, (0.0, 1.0, 0.18, 1.0))
            selected.scale.x = self.target_marker_scale_m * 1.8
            selected.scale.y = self.target_marker_scale_m * 1.8
            selected.scale.z = self.target_marker_scale_m * 1.8
            selected.pose.position = self.point_for_index(model, goal.index, 0.16)
            markers.markers.append(selected)

            path = self.make_marker('wfd', 5, Marker.LINE_STRIP, now)
            self.set_marker_color(path, (0.55, 0.0, 1.0, 0.95))
            path.scale.x = 0.035
            path.points = [self.point_for_index(model, index, 0.10) for index in goal.path_indices]
            markers.markers.append(path)

        self.marker_pub.publish(markers)

    def point_for_index(self, model: MapModel, index: int, z_value: float = 0.02):
        x_value, y_value = self.grid_center(model, index)
        point = Point()
        point.x = x_value
        point.y = y_value
        point.z = z_value
        return point

    def point_for_grid(self, model: MapModel, grid_x: float, grid_y: float, z_value: float):
        point = Point()
        point.x = model.origin_x + (grid_x + 0.5) * model.resolution
        point.y = model.origin_y + (grid_y + 0.5) * model.resolution
        point.z = z_value
        return point

    def make_marker(self, namespace, marker_id, marker_type, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.map_frame
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def delete_all_marker(self):
        marker = Marker()
        marker.action = Marker.DELETEALL
        return marker

    def set_marker_color(self, marker: Marker, color):
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

    def stop_robot(self):
        if not self.context.ok():
            return
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = WavefrontExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
