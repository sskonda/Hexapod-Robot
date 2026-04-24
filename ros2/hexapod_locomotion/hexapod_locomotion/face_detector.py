#!/usr/bin/env python3

"""Camera face detector that publishes an annotated image stream."""

from pathlib import Path

import cv2
import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, RegionOfInterest


CASCADE_FILE_NAME = 'haarcascade_frontalface_default.xml'


class FaceDetector(Node):
    def __init__(self):
        super().__init__('face_detector')

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('output_image_topic', '/face_detection/image')
        self.declare_parameter('bounding_box_topic', '/face_detection/bounding_box')
        self.declare_parameter('cascade_path', '')
        self.declare_parameter('scale_factor', 1.1)
        self.declare_parameter('min_neighbors', 5)
        self.declare_parameter('min_size_pixels', 30)
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('log_throttle_sec', 2.0)

        image_topic = self.get_parameter('image_topic').value
        output_image_topic = self.get_parameter('output_image_topic').value
        bounding_box_topic = self.get_parameter('bounding_box_topic').value
        qos_depth = int(self.get_parameter('qos_depth').value)

        self.scale_factor = float(self.get_parameter('scale_factor').value)
        self.min_neighbors = int(self.get_parameter('min_neighbors').value)
        min_size_pixels = int(self.get_parameter('min_size_pixels').value)
        self.min_size = (min_size_pixels, min_size_pixels)
        self.log_throttle_ns = int(
            float(self.get_parameter('log_throttle_sec').value) * 1_000_000_000
        )
        self.last_detection_log_ns = -self.log_throttle_ns

        self.bridge = CvBridge()
        cascade_path = self._resolve_cascade_path()
        self.face_cascade = cv2.CascadeClassifier(str(cascade_path))
        if self.face_cascade.empty():
            raise RuntimeError(f'Failed to load Haar cascade: {cascade_path}')

        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_depth,
        )
        self.image_pub = self.create_publisher(Image, output_image_topic, qos_depth)
        self.bounding_box_pub = self.create_publisher(
            RegionOfInterest,
            bounding_box_topic,
            qos_depth,
        )

        self.get_logger().info(f'Subscribed to {image_topic}')
        self.get_logger().info(f'Publishing annotated images on {output_image_topic}')
        self.get_logger().info(f'Publishing largest face ROI on {bounding_box_topic}')

    def _resolve_cascade_path(self):
        configured_path = str(self.get_parameter('cascade_path').value).strip()
        candidates = []

        if configured_path:
            candidates.append(Path(configured_path).expanduser())

        candidates.append(Path(__file__).resolve().parents[1] / 'cascades' / CASCADE_FILE_NAME)

        try:
            package_share = Path(get_package_share_directory('hexapod_locomotion'))
            candidates.append(package_share / 'cascades' / CASCADE_FILE_NAME)
        except PackageNotFoundError:
            pass

        cv2_data_dir = getattr(getattr(cv2, 'data', None), 'haarcascades', None)
        if cv2_data_dir:
            candidates.append(Path(cv2_data_dir) / CASCADE_FILE_NAME)

        for candidate in candidates:
            if candidate.is_file():
                return candidate

        searched = ', '.join(str(candidate) for candidate in candidates)
        raise FileNotFoundError(f'Could not find {CASCADE_FILE_NAME}. Searched: {searched}')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Failed to convert image: {exc}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        faces = self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=self.scale_factor,
            minNeighbors=self.min_neighbors,
            minSize=self.min_size,
        )

        if len(faces) > 0:
            self._log_face_detected()
            self._publish_largest_roi(faces)

        for x, y, w, h in faces:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out_msg.header = msg.header
        self.image_pub.publish(out_msg)

    def _log_face_detected(self):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_detection_log_ns < self.log_throttle_ns:
            return

        self.get_logger().info('Face detected')
        self.last_detection_log_ns = now_ns

    def _publish_largest_roi(self, faces):
        x, y, w, h = max(faces, key=lambda face: face[2] * face[3])
        roi = RegionOfInterest()
        roi.x_offset = int(x)
        roi.y_offset = int(y)
        roi.width = int(w)
        roi.height = int(h)
        roi.do_rectify = False
        self.bounding_box_pub.publish(roi)


def main():
    rclpy.init()
    node = FaceDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
