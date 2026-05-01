#!/usr/bin/env python3

"""Decode QR codes from a camera image stream and publish their text."""

from typing import List, Sequence, Tuple

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


Point = Tuple[int, int]


class QrCodeDetectorNode(Node):
    def __init__(self):
        super().__init__('qr_code_detector')

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('text_topic', '/qr_code/text')
        self.declare_parameter('annotated_image_topic', '/qr_code/image')
        self.declare_parameter('publish_annotated_image', True)
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('log_throttle_sec', 2.0)
        self.declare_parameter('republish_same_text', False)
        self.declare_parameter('output_image_width', 320)
        self.declare_parameter('output_image_height', 240)
        self.declare_parameter('output_image_grayscale', True)

        image_topic = str(self.get_parameter('image_topic').value)
        text_topic = str(self.get_parameter('text_topic').value)
        annotated_image_topic = str(self.get_parameter('annotated_image_topic').value)
        qos_depth = int(self.get_parameter('qos_depth').value)

        self.publish_annotated_image = bool(
            self.get_parameter('publish_annotated_image').value
        )
        self.republish_same_text = bool(self.get_parameter('republish_same_text').value)
        self.output_image_width = max(
            1,
            int(self.get_parameter('output_image_width').value),
        )
        self.output_image_height = max(
            1,
            int(self.get_parameter('output_image_height').value),
        )
        self.output_image_grayscale = bool(
            self.get_parameter('output_image_grayscale').value
        )
        self.log_throttle_ns = int(
            float(self.get_parameter('log_throttle_sec').value) * 1_000_000_000
        )
        self.last_log_ns = -self.log_throttle_ns
        self.last_published_texts = set()

        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()

        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.text_pub = self.create_publisher(String, text_topic, qos_depth)
        self.image_pub = None
        if self.publish_annotated_image:
            self.image_pub = self.create_publisher(
                Image,
                annotated_image_topic,
                qos_profile_sensor_data,
            )

        self.get_logger().info(f'Subscribed to {image_topic}')
        self.get_logger().info(f'Publishing QR text on {text_topic}')
        if self.publish_annotated_image:
            self.get_logger().info(
                f'Publishing annotated QR images on {annotated_image_topic}'
            )

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Failed to convert image: {exc}')
            return

        decoded_texts, polygons = self._decode_qr_codes(frame)
        texts_to_publish = [text for text in decoded_texts if text]

        if texts_to_publish:
            if self.republish_same_text:
                publishable_texts = texts_to_publish
            else:
                publishable_texts = [
                    text
                    for text in texts_to_publish
                    if text not in self.last_published_texts
                ]
                self.last_published_texts = set(texts_to_publish)

            for text in publishable_texts:
                self.text_pub.publish(String(data=text))

            if publishable_texts:
                self._log_detected_texts(publishable_texts)
        else:
            self.last_published_texts.clear()

        if self.image_pub is not None:
            annotated = frame.copy()
            self._draw_annotations(annotated, decoded_texts, polygons)
            output_frame, output_encoding = self._prepare_output_frame(annotated)
            out_msg = self.bridge.cv2_to_imgmsg(output_frame, encoding=output_encoding)
            out_msg.header = msg.header
            self.image_pub.publish(out_msg)

    def _decode_qr_codes(self, frame) -> Tuple[List[str], List[List[Point]]]:
        try:
            detected, decoded_info, points, _ = self.detector.detectAndDecodeMulti(frame)
        except Exception:
            detected, decoded_info, points = False, [], None

        if detected and decoded_info:
            return (
                [str(text).strip() for text in decoded_info],
                self._normalize_polygons(points),
            )

        text, points, _ = self.detector.detectAndDecode(frame)
        if text:
            polygons = [points] if points is not None else []
            return [str(text).strip()], self._normalize_polygons(polygons)

        return [], []

    def _normalize_polygons(self, points: Sequence) -> List[List[Point]]:
        if points is None:
            return []

        polygons = []
        for polygon in points:
            if polygon is None:
                continue

            normalized = []
            for point in polygon:
                if len(point) < 2:
                    continue
                normalized.append(
                    (int(round(float(point[0]))), int(round(float(point[1]))))
                )

            if normalized:
                polygons.append(normalized)
        return polygons

    def _draw_annotations(
        self,
        frame,
        decoded_texts: Sequence[str],
        polygons: Sequence[Sequence[Point]],
    ):
        for index, polygon in enumerate(polygons):
            if len(polygon) < 2:
                continue

            for point_index, start in enumerate(polygon):
                end = polygon[(point_index + 1) % len(polygon)]
                cv2.line(frame, start, end, (0, 255, 0), 2)

            text = decoded_texts[index] if index < len(decoded_texts) else ''
            if text:
                origin_x, origin_y = polygon[0]
                cv2.putText(
                    frame,
                    text,
                    (origin_x, max(20, origin_y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

    def _prepare_output_frame(self, frame):
        resized = cv2.resize(
            frame,
            (self.output_image_width, self.output_image_height),
            interpolation=cv2.INTER_AREA,
        )
        if self.output_image_grayscale:
            return cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY), 'mono8'
        return resized, 'bgr8'

    def _log_detected_texts(self, texts: Sequence[str]):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_log_ns < self.log_throttle_ns:
            return

        joined = ', '.join(repr(text) for text in texts)
        self.get_logger().info(f'Detected QR text: {joined}')
        self.last_log_ns = now_ns


def main(args=None):
    rclpy.init(args=args)
    node = QrCodeDetectorNode()
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
