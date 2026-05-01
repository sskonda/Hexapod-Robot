#!/usr/bin/env python3

"""Project detected QR colors onto the mapped front wall using LiDAR."""

import json
import math
from typing import Dict, Optional, Tuple

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


COLOR_TABLE: Dict[str, Tuple[int, Tuple[float, float, float, float]]] = {
    'red': (1, (1.0, 0.1, 0.1, 0.95)),
    'blue': (2, (0.1, 0.35, 1.0, 0.95)),
    'green': (3, (0.1, 0.9, 0.2, 0.95)),
    'yellow': (4, (1.0, 0.9, 0.1, 0.95)),
    'orange': (5, (1.0, 0.5, 0.05, 0.95)),
}


def quaternion_to_yaw(x_value: float, y_value: float, z_value: float, w_value: float) -> float:
    siny_cosp = 2.0 * (w_value * z_value + x_value * y_value)
    cosy_cosp = 1.0 - 2.0 * (y_value * y_value + z_value * z_value)
    return math.atan2(siny_cosp, cosy_cosp)


class QrWallMarkerNode(Node):
    def __init__(self):
        super().__init__('qr_wall_marker')

        self.declare_parameter('qr_text_topic', '/qr_code/text')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('marker_topic', '/qr_code/markers')
        self.declare_parameter('marker_state_topic', '/qr_code/marker_state')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('scan_timeout_sec', 0.75)
        self.declare_parameter('front_search_half_width_deg', 8.0)
        self.declare_parameter('marker_size_m', 0.12)
        self.declare_parameter('marker_height_m', 0.18)
        self.declare_parameter('label_height_m', 0.34)
        self.declare_parameter('marker_lifetime_sec', 0.0)

        self.qr_text_topic = str(self.get_parameter('qr_text_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.marker_topic = str(self.get_parameter('marker_topic').value)
        self.marker_state_topic = str(self.get_parameter('marker_state_topic').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.scan_timeout_sec = max(0.05, float(self.get_parameter('scan_timeout_sec').value))
        self.front_search_half_width_rad = math.radians(
            max(0.0, float(self.get_parameter('front_search_half_width_deg').value))
        )
        self.marker_size_m = max(0.02, float(self.get_parameter('marker_size_m').value))
        self.marker_height_m = max(0.02, float(self.get_parameter('marker_height_m').value))
        self.label_height_m = max(self.marker_height_m, float(self.get_parameter('label_height_m').value))
        self.marker_lifetime_sec = max(
            0.0,
            float(self.get_parameter('marker_lifetime_sec').value),
        )

        self.latest_scan: Optional[LaserScan] = None
        self.latest_scan_stamp = None
        self.detected_markers: Dict[str, Tuple[float, float]] = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, marker_qos)
        self.marker_state_pub = self.create_publisher(String, self.marker_state_topic, state_qos)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(String, self.qr_text_topic, self.qr_text_callback, 10)

        self.get_logger().info(
            f'Projecting QR detections from {self.qr_text_topic} onto {self.marker_topic}'
        )
        self.publish_marker_state()

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg
        self.latest_scan_stamp = self.get_clock().now()

    def qr_text_callback(self, msg: String):
        color_name = msg.data.strip().lower()
        if color_name not in COLOR_TABLE:
            self.get_logger().warn(
                f'Ignoring unsupported QR text {msg.data!r}. Expected one of {sorted(COLOR_TABLE)}.',
                throttle_duration_sec=2.0,
            )
            return

        if self.latest_scan is None or self.latest_scan_stamp is None:
            self.get_logger().warn('QR detected before any LiDAR scan was available.', throttle_duration_sec=2.0)
            return

        if (self.get_clock().now() - self.latest_scan_stamp).nanoseconds > int(self.scan_timeout_sec * 1e9):
            self.get_logger().warn('Latest LiDAR scan is stale, skipping QR wall marker.', throttle_duration_sec=2.0)
            return

        wall_point = self.project_front_wall_to_map(self.latest_scan)
        if wall_point is None:
            self.get_logger().warn('Could not find a valid front-wall LiDAR hit for QR marker.', throttle_duration_sec=2.0)
            return

        replaced = color_name in self.detected_markers
        self.detected_markers[color_name] = wall_point
        self.publish_markers()
        self.publish_marker_state()
        self.get_logger().info(
            f'{"Replaced" if replaced else "Placed"} {color_name} QR marker at '
            f'({wall_point[0]:.2f}, {wall_point[1]:.2f}) in {self.map_frame}.',
            throttle_duration_sec=1.0,
        )

    def project_front_wall_to_map(self, scan: LaserScan) -> Optional[Tuple[float, float]]:
        scan_frame = scan.header.frame_id.strip()
        if not scan_frame:
            return None

        try:
            base_from_scan = self.tf_buffer.lookup_transform(
                self.base_frame,
                scan_frame,
                Time(),
            )
            map_from_scan = self.tf_buffer.lookup_transform(
                self.map_frame,
                scan_frame,
                Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'Could not transform {scan_frame} into {self.base_frame}/{self.map_frame}: {exc}',
                throttle_duration_sec=2.0,
            )
            return None

        rotation = base_from_scan.transform.rotation
        scan_to_base_yaw = quaternion_to_yaw(
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )
        desired_scan_angle = -scan_to_base_yaw
        measurement = self.find_front_measurement(scan, desired_scan_angle)
        if measurement is None:
            return None

        range_m, scan_angle = measurement
        point_in_scan_x = range_m * math.cos(scan_angle)
        point_in_scan_y = range_m * math.sin(scan_angle)

        map_rotation = map_from_scan.transform.rotation
        map_yaw = quaternion_to_yaw(
            map_rotation.x,
            map_rotation.y,
            map_rotation.z,
            map_rotation.w,
        )
        translation = map_from_scan.transform.translation
        point_in_map_x = (
            translation.x
            + point_in_scan_x * math.cos(map_yaw)
            - point_in_scan_y * math.sin(map_yaw)
        )
        point_in_map_y = (
            translation.y
            + point_in_scan_x * math.sin(map_yaw)
            + point_in_scan_y * math.cos(map_yaw)
        )
        return point_in_map_x, point_in_map_y

    def find_front_measurement(
        self,
        scan: LaserScan,
        desired_scan_angle: float,
    ) -> Optional[Tuple[float, float]]:
        best = None
        for index, raw_range in enumerate(scan.ranges):
            if not math.isfinite(raw_range):
                continue
            if raw_range < scan.range_min or raw_range > scan.range_max:
                continue

            angle = scan.angle_min + index * scan.angle_increment
            angle_error = math.atan2(
                math.sin(angle - desired_scan_angle),
                math.cos(angle - desired_scan_angle),
            )
            abs_error = abs(angle_error)
            if abs_error > self.front_search_half_width_rad:
                continue

            candidate = (abs_error, raw_range, angle)
            if best is None or candidate < best:
                best = candidate

        if best is None:
            return None
        return best[1], best[2]

    def publish_markers(self):
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        marker_lifetime = self.duration_message(self.marker_lifetime_sec)

        for color_name, (x_pos, y_pos) in sorted(self.detected_markers.items()):
            marker_id, rgba = COLOR_TABLE[color_name]

            wall_marker = Marker()
            wall_marker.header.frame_id = self.map_frame
            wall_marker.header.stamp = now
            wall_marker.ns = 'qr_wall'
            wall_marker.id = marker_id
            wall_marker.type = Marker.CUBE
            wall_marker.action = Marker.ADD
            wall_marker.pose.position.x = x_pos
            wall_marker.pose.position.y = y_pos
            wall_marker.pose.position.z = self.marker_height_m * 0.5
            wall_marker.pose.orientation.w = 1.0
            wall_marker.scale.x = self.marker_size_m
            wall_marker.scale.y = self.marker_size_m
            wall_marker.scale.z = self.marker_height_m
            wall_marker.lifetime = marker_lifetime
            self.set_marker_color(wall_marker, rgba)
            markers.markers.append(wall_marker)

            label_marker = Marker()
            label_marker.header.frame_id = self.map_frame
            label_marker.header.stamp = now
            label_marker.ns = 'qr_wall_label'
            label_marker.id = marker_id + 100
            label_marker.type = Marker.TEXT_VIEW_FACING
            label_marker.action = Marker.ADD
            label_marker.pose.position.x = x_pos
            label_marker.pose.position.y = y_pos
            label_marker.pose.position.z = self.label_height_m
            label_marker.pose.orientation.w = 1.0
            label_marker.scale.z = self.marker_size_m
            label_marker.text = color_name
            label_marker.lifetime = marker_lifetime
            self.set_marker_color(label_marker, rgba)
            markers.markers.append(label_marker)

        self.marker_pub.publish(markers)

    def publish_marker_state(self):
        state = {
            'frame_id': self.map_frame,
            'markers': {
                color_name: {
                    'x': coordinates[0],
                    'y': coordinates[1],
                }
                for color_name, coordinates in sorted(self.detected_markers.items())
            },
        }
        self.marker_state_pub.publish(String(data=json.dumps(state, sort_keys=True)))

    def set_marker_color(self, marker: Marker, color: Tuple[float, float, float, float]):
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

    def duration_message(self, duration_sec: float) -> Duration:
        seconds = max(0.0, float(duration_sec))
        whole_seconds = int(seconds)
        nanoseconds = int(round((seconds - whole_seconds) * 1_000_000_000))
        if nanoseconds >= 1_000_000_000:
            whole_seconds += 1
            nanoseconds -= 1_000_000_000
        return Duration(sec=whole_seconds, nanosec=nanoseconds)


def main(args=None):
    rclpy.init(args=args)
    node = QrWallMarkerNode()
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
