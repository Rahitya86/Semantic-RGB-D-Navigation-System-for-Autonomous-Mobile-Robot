#!/usr/bin/env python3
"""
Combined Detector Node

Subscribes to the camera image plus YOLO and color detection outputs,
republishes merged detections on `/detections/combined`, and publishes an
annotated image on `/detections/combined/image` for `rqt_image_view`.
"""

from copy import deepcopy
from threading import Lock

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray

class CombinedDetector(Node):
    def __init__(self):
        super().__init__('combined_detector')
        self.bridge = CvBridge()
        self.declare_parameter('yolo_topic', '/yolo/detections')
        self.declare_parameter('color_topic', '/semantic/color_detections')
        self.declare_parameter('combined_topic', '/detections/combined')
        self.declare_parameter('camera_topic', '/camera/rgb/image_raw')
        self.declare_parameter('combined_image_topic', '/detections/combined/image')

        yolo_topic = self.get_parameter('yolo_topic').get_parameter_value().string_value
        color_topic = self.get_parameter('color_topic').get_parameter_value().string_value
        combined_topic = self.get_parameter('combined_topic').get_parameter_value().string_value
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.combined_image_topic = self.get_parameter(
            'combined_image_topic'
        ).get_parameter_value().string_value

        self.lock = Lock()
        self.latest_yolo = None
        self.latest_color = None
        self.latest_image = None

        self.yolo_sub = self.create_subscription(
            Detection2DArray,
            yolo_topic,
            self.yolo_cb,
            10
        )
        self.color_sub = self.create_subscription(
            Detection2DArray,
            color_topic,
            self.color_cb,
            10
        )
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_cb,
            qos_profile_sensor_data
        )

        self.pub = self.create_publisher(Detection2DArray, combined_topic, 10)
        self.image_pub = self.create_publisher(Image, self.combined_image_topic, 10)
        self.get_logger().info(
            'Combined Detector initialized. '
            f'Subscribing: {self.camera_topic}, {yolo_topic}, {color_topic} -> '
            f'publishing: {combined_topic}, {self.combined_image_topic}'
        )

        # Timer to periodically publish merged detections
        self.create_timer(0.1, self.publish_combined)

    def yolo_cb(self, msg: Detection2DArray):
        with self.lock:
            self.latest_yolo = msg

    def color_cb(self, msg: Detection2DArray):
        with self.lock:
            self.latest_color = msg

    def image_cb(self, msg: Image):
        with self.lock:
            self.latest_image = msg

    def _draw_detections(self, frame, detections, default_color, prefix=''):
        for detection in detections:
            bbox = detection.bbox

            # Validate bbox fields (bbox.size_x/size_y may be missing or invalid)
            try:
                width = float(bbox.size_x)
                height = float(bbox.size_y)
            except Exception:
                continue

            if not np.isfinite(width) or not np.isfinite(height) or width <= 1.0 or height <= 1.0:
                continue

            x_center, y_center = self._bbox_center_xy(bbox)

            x1 = max(0, int(x_center - width / 2.0))
            y1 = max(0, int(y_center - height / 2.0))
            x2 = min(frame.shape[1] - 1, int(x_center + width / 2.0))
            y2 = min(frame.shape[0] - 1, int(y_center + height / 2.0))

            if x2 <= x1 or y2 <= y1:
                continue

            # Resolve label + confidence (for color selection and text)
            label = 'object'
            confidence = 0.0
            if detection.results:
                result = detection.results[0]
                label = getattr(
                    result.hypothesis,
                    'class_id',
                    getattr(result.hypothesis, 'class_name', label)
                )
                confidence = float(getattr(result.hypothesis, 'score', 0.0))

            # If this is a color detector label, use its actual semantic color.
            # color_detector publishes class_id as: red_box/blue_box/green_box/yellow_box/purple_box
            color_map = {
                'red_box': (0, 0, 255),
                'blue_box': (255, 0, 0),
                'green_box': (0, 255, 0),
                'yellow_box': (0, 255, 255),
                'purple_box': (255, 0, 255),
            }
            draw_color = color_map.get(str(label), default_color)

            cv2.rectangle(frame, (x1, y1), (x2, y2), draw_color, 2)

            label_text = f'{prefix}{label} {confidence:.2f}' if prefix else f'{label} {confidence:.2f}'
            label_y = max(20, y1 - 8)
            cv2.putText(
                frame,
                label_text,
                (x1, label_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                draw_color,
                2,
                cv2.LINE_AA,
            )
            cv2.circle(frame, (x_center, y_center), 3, draw_color, -1)


    def _bbox_center_xy(self, bbox):
        center = bbox.center
        if hasattr(center, 'position'):
            return int(center.position.x), int(center.position.y)
        return int(center.x), int(center.y)

    def publish_combined_image(self):
        if self.latest_image is None:
            placeholder = np.zeros((720, 1280, 3), dtype=np.uint8)
            placeholder[:] = (35, 35, 35)
            cv2.putText(
                placeholder,
                f'Waiting for {self.camera_topic} ...',
                (60, 180),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.2,
                (0, 255, 255),
                3,
                cv2.LINE_AA,
            )
            cv2.putText(
                placeholder,
                'Start Gazebo + K12 world to see live detections',
                (60, 240),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                placeholder,
                f'rqt_image_view -> {self.combined_image_topic}',
                (60, 300),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (180, 180, 255),
                2,
                cv2.LINE_AA,
            )
            out_msg = self.bridge.cv2_to_imgmsg(placeholder, encoding='bgr8')
            out_msg.header.stamp = self.get_clock().now().to_msg()
            out_msg.header.frame_id = 'combined_detector'
            self.image_pub.publish(out_msg)
            return

        image_msg = self.latest_image
        if image_msg.encoding in ('16UC1', '32FC1', 'mono16'):
            depth = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
            depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
            depth_min = float(np.min(depth)) if depth.size else 0.0
            depth_max = float(np.max(depth)) if depth.size else 0.0
            if depth_max > depth_min:
                normalized = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
            else:
                normalized = np.zeros_like(depth, dtype=np.uint8)
            frame = cv2.applyColorMap(normalized.astype(np.uint8), cv2.COLORMAP_TURBO)
            cv2.putText(
                frame,
                f'Depth view: {image_msg.encoding} min={depth_min:.2f} max={depth_max:.2f}',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
        else:
            frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            frame = deepcopy(frame)

        yolo_detections = []
        color_detections = []
        with self.lock:
            if self.latest_yolo is not None and getattr(self.latest_yolo, 'detections', None):
                yolo_detections = list(self.latest_yolo.detections)
            if self.latest_color is not None and getattr(self.latest_color, 'detections', None):
                color_detections = list(self.latest_color.detections)

        if yolo_detections:
            self._draw_detections(frame, yolo_detections, (0, 255, 0), prefix='YOLO: ')
        if color_detections:
            # default_color is only used if label is not in color_map
            self._draw_detections(frame, color_detections, (255, 0, 255), prefix='Color: ')


        cv2.putText(
            frame,
            f'YOLO: {len(yolo_detections)}  Color: {len(color_detections)}  Combined: {len(yolo_detections) + len(color_detections)}',
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out_msg.header = image_msg.header
        self.image_pub.publish(out_msg)

    def publish_combined(self):
        with self.lock:
            out = Detection2DArray()
            # Use newest header if available
            if self.latest_yolo is not None:
                out.header = self.latest_yolo.header
            elif self.latest_color is not None:
                out.header = self.latest_color.header
            elif self.latest_image is not None:
                out.header = self.latest_image.header
            else:
                out.header = Header()
                out.header.stamp = self.get_clock().now().to_msg()

            # Merge detections (shallow copy of lists)
            if self.latest_yolo is not None and getattr(self.latest_yolo, 'detections', None):
                out.detections.extend(self.latest_yolo.detections)
            if self.latest_color is not None and getattr(self.latest_color, 'detections', None):
                out.detections.extend(self.latest_color.detections)

            # Publish merged detections
            self.pub.publish(out)

        self.publish_combined_image()

def main(args=None):
    rclpy.init(args=args)
    node = CombinedDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
