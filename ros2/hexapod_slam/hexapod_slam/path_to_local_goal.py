#!/usr/bin/env python3
"""
path_to_local_goal.py

Bridges the graph planner and local navigation by publishing a short-horizon
local goal lookahead_distance_m ahead along the planned path.
"""

import math
from typing import List, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from .frame_utils import lookup_point_2d
from .map_frontier_utils import distance_m, heading_between_points


# ---------------------------------------------------------------------------
# Pure helpers (testable without ROS2)
# ---------------------------------------------------------------------------

def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def compute_local_goal(
    robot_x: float,
    robot_y: float,
    path_poses: List,          # list of PoseStamped
    lookahead_m: float,
) -> Optional[PoseStamped]:
    """
    Walk along path_poses and return the pose that is ~lookahead_m away
    from the robot.  Falls back to the final pose if the path is shorter.
    """
    if not path_poses:
        return None

    # Find the segment the robot is nearest to
    # Walk the path and find the point that is lookahead_m ahead
    accumulated = 0.0
    prev_x = robot_x
    prev_y = robot_y

    for i, ps in enumerate(path_poses):
        wx = ps.pose.position.x
        wy = ps.pose.position.y
        seg = distance_m(prev_x, prev_y, wx, wy)
        if accumulated + seg >= lookahead_m:
            # Interpolate
            remaining = lookahead_m - accumulated
            frac = remaining / max(seg, 1e-6)
            goal_x = prev_x + frac * (wx - prev_x)
            goal_y = prev_y + frac * (wy - prev_y)
            result = PoseStamped()
            result.header = ps.header
            result.pose.position.x = goal_x
            result.pose.position.y = goal_y
            yaw = heading_between_points(prev_x, prev_y, wx, wy)
            result.pose.orientation = _yaw_to_quat(yaw)
            return result
        accumulated += seg
        prev_x, prev_y = wx, wy

    # Reached end of path – return final pose
    return path_poses[-1]


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

class PathToLocalGoal(Node):
    def __init__(self):
        super().__init__("path_to_local_goal")

        self.declare_parameter("update_rate_hz", 5.0)
        self.declare_parameter("lookahead_distance_m", 0.30)
        self.declare_parameter("goal_tolerance_m", 0.15)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_topic", "odom")

        self._rate = self.get_parameter("update_rate_hz").value
        self._lookahead = self.get_parameter("lookahead_distance_m").value
        self._tolerance = self.get_parameter("goal_tolerance_m").value
        self._map_frame = self.get_parameter("map_frame").value
        self._odom_topic = self.get_parameter("odom_topic").value

        # State
        self._current_target: Optional[PoseStamped] = None
        self._path: Optional[Path] = None
        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._robot_pose_frame: str = self._odom_topic
        self._last_tf_warn_time: float = 0.0
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Subscriptions
        self.create_subscription(PoseStamped, "/maze_graph/current_target", self._target_cb, 1)
        self.create_subscription(Path, "/maze_graph/path", self._path_cb, 1)
        self.create_subscription(Odometry, self._odom_topic, self._odom_cb, 10)

        # Publishers
        self._pub_local_goal = self.create_publisher(PoseStamped, "/maze_graph/local_goal", 1)
        self._pub_status = self.create_publisher(String, "/maze_graph/local_status", 1)

        period = 1.0 / max(0.1, self._rate)
        self.create_timer(period, self._update)

        self.get_logger().info("PathToLocalGoal started.")

    # ------------------------------------------------------------------
    def _target_cb(self, msg: PoseStamped):
        self._current_target = msg

    def _path_cb(self, msg: Path):
        self._path = msg

    def _odom_cb(self, msg: Odometry):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        if msg.header.frame_id:
            self._robot_pose_frame = msg.header.frame_id

    def _robot_position_in_map(self) -> Optional[tuple[float, float]]:
        robot_map_position = lookup_point_2d(
            self._tf_buffer,
            self._map_frame,
            self._robot_pose_frame,
            self._robot_x,
            self._robot_y,
        )
        if robot_map_position is None and self._map_frame == self._robot_pose_frame:
            return self._robot_x, self._robot_y
        return robot_map_position

    # ------------------------------------------------------------------
    def _update(self):
        status_msg = String()

        if self._current_target is None:
            status_msg.data = "no_target"
            self._pub_status.publish(status_msg)
            return

        tx = self._current_target.pose.position.x
        ty = self._current_target.pose.position.y
        robot_map_position = self._robot_position_in_map()
        if robot_map_position is None:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            if now_sec - self._last_tf_warn_time > 5.0:
                self.get_logger().warn(
                    f'Unable to transform robot pose from {self._robot_pose_frame} '
                    f'to {self._map_frame}; local-goal update skipped.'
                )
                self._last_tf_warn_time = now_sec
            status_msg.data = "no_pose_in_map"
            self._pub_status.publish(status_msg)
            return

        robot_x_m, robot_y_m = robot_map_position
        dist_to_target = distance_m(robot_x_m, robot_y_m, tx, ty)

        if dist_to_target < self._tolerance:
            status_msg.data = "reached"
            self._pub_status.publish(status_msg)
            return

        # Compute local goal from path
        local_goal: Optional[PoseStamped] = None
        if self._path is not None and self._path.poses:
            local_goal = compute_local_goal(
                robot_x_m,
                robot_y_m,
                self._path.poses,
                self._lookahead,
            )

        if local_goal is None:
            # Fall back to direct target
            local_goal = PoseStamped()
            local_goal.header.frame_id = self._map_frame
            local_goal.header.stamp = self.get_clock().now().to_msg()
            local_goal.pose.position.x = tx
            local_goal.pose.position.y = ty
            yaw = heading_between_points(robot_x_m, robot_y_m, tx, ty)
            local_goal.pose.orientation = _yaw_to_quat(yaw)

        local_goal.header.stamp = self.get_clock().now().to_msg()
        self._pub_local_goal.publish(local_goal)

        status_msg.data = "moving"
        self._pub_status.publish(status_msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = PathToLocalGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
