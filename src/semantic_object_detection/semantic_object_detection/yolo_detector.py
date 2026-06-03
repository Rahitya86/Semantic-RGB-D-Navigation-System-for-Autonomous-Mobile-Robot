#!/usr/bin/env python3
"""
YOLO Detector - FIXED VERSION with better logging and lower threshold
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import time
import logging
import os

logging.basicConfig(level=logging.INFO, format='[%(levelname)s]: %(message)s')
logger = logging.getLogger('YOLODetector')


class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        self.get_logger().info('=== YOLO Detector Starting (FIXED) ===')
        
        self.bridge = CvBridge()

        # Parameters with better defaults
        # For this K12 setup, only depth camera raw image is typically published as:
        #   /camera/depth_camera/image_raw
        # So default YOLO input should match that unless overridden.
        self.declare_parameter('camera_topic', '/camera/depth_camera/image_raw')

        self.declare_parameter('confidence_threshold', 0.15)  # MUCH LOWER!
        self.declare_parameter('model_path', 'yolov8n.pt')
        # Comma-separated list of YOLO class names (COCO) to keep.
        # Example: "chair,table,person"
        self.declare_parameter('target_classes', '')

        # Heuristic post-filtering to reduce wall/semantic-mark false positives.
        # These operate on YOLO bounding boxes (image-space) and are world-agnostic.
        self.declare_parameter('enable_bbox_heuristics', True)
        self.declare_parameter('max_box_area_ratio', 0.35)  # reject boxes covering too much of the frame
        self.declare_parameter('min_aspect_ratio', 0.15)      # reject extremely skinny boxes
        self.declare_parameter('max_aspect_ratio', 8.0)       # reject extremely tall boxes
        self.declare_parameter('border_margin_ratio', 0.02)  # reject boxes that touch image borders too much


        self.camera_topic = self.get_parameter('camera_topic').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.model_path = self.get_parameter('model_path').value
        target_classes = self.get_parameter('target_classes').value
        
        self.target_classes = {c.strip() for c in target_classes.split(',') if c.strip()}

        # Whitelist-only behavior for world-spawned objects:
        # - If target_classes is non-empty: keep only those YOLO classes.
        # - If target_classes is empty: keep all (current behavior).
        # This launch param is how you restrict to objects that correspond to your world assets.


        self.enable_bbox_heuristics = self.get_parameter('enable_bbox_heuristics').value
        self.max_box_area_ratio = float(self.get_parameter('max_box_area_ratio').value)
        self.min_aspect_ratio = float(self.get_parameter('min_aspect_ratio').value)
        self.max_aspect_ratio = float(self.get_parameter('max_aspect_ratio').value)
        self.border_margin_ratio = float(self.get_parameter('border_margin_ratio').value)

        # Load model
        self.model = None
        self._load_model()
        
        self.frame_count = 0
        self.last_time = time.time()
        self.fps = 0.0
        
        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, self.camera_topic, self.image_callback, qos_profile_sensor_data
        )
        
        # Publishers
        self.annotated_pub = self.create_publisher(Image, '/yolo/annotated_image', 10)
        self.detections_pub = self.create_publisher(Detection2DArray, '/yolo/detections', 10)
        
        self.get_logger().info(f'Subscribed to: {self.camera_topic}')
        self.get_logger().info(f'Publishing detections to: /yolo/detections')
        self.get_logger().info(f'Confidence threshold: {self.confidence_threshold}')
        
        # Timer for periodic status
        self.create_timer(5.0, self.print_status)

    def _load_model(self):
        """Load YOLO model with error handling"""
        model_to_try = [
            self.model_path,
            os.path.expanduser('~/yolov8n.pt'),
            '/opt/ultralytics/yolov8n.pt',
            'yolov8n.pt'
        ]
        
        for path in model_to_try:
            try:
                if os.path.exists(path):
                    self.model = YOLO(path)
                    self.get_logger().info(f'✓ Model loaded: {path}')
                    return
                # Try direct load (ultralytics auto-downloads)
                self.model = YOLO(path)
                self.get_logger().info(f'✓ Model loaded: {path}')
                return
            except Exception as e:
                self.get_logger().debug(f'Tried {path}: {e}')
                continue
        
        # Last resort - auto-download
        try:
            self.model = YOLO('yolov8n.pt')
            self.get_logger().info('✓ Model loaded via auto-download')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')

    def image_callback(self, msg: Image):
        if self.model is None:
            self.get_logger().warn('No model loaded, skipping...', throttle_sec=5)
            return
            
        try:
            # Convert image
            # Convert based on encoding.
            # If it's already rgb8 (as observed), request bgr8 and proceed.
            if getattr(msg, 'encoding', '') in ('rgb8', 'bgr8', 'mono8'):
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                # YOLO expects 8-bit BGR. If we get float depth (32FC1), convert to a visible colormap.
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                except Exception:
                    depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                    depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
                    depth_min = float(np.min(depth)) if depth.size else 0.0
                    depth_max = float(np.max(depth)) if depth.size else 0.0
                    if depth_max > depth_min:
                        normalized = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
                    else:
                        normalized = np.zeros_like(depth, dtype=np.uint8)
                    cv_image = cv2.applyColorMap(normalized.astype(np.uint8), cv2.COLORMAP_TURBO)

            h, w = cv_image.shape[:2]

            self.get_logger().debug(f'Image received: {w}x{h}')
            
            # Run inference
            results = self.model(cv_image, conf=self.confidence_threshold, verbose=False)
            
            # Process
            annotated = self.draw_annotations(cv_image.copy(), results[0])
            detections_array = self.extract_detections(results[0], msg.header)
            
            # Publish
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)
            self.detections_pub.publish(detections_array)
            
            # Log
            det_count = len(detections_array.detections)
            if det_count > 0:
                self.get_logger().info(f'✓ YOLO detected: {det_count} objects')
                
            self.update_fps()
            
        except Exception as e:
            self.get_logger().error(f'Error: {e}', throttle_duration_sec=5)

    def draw_annotations(self, image, results):
        annotated = image.copy()
        det_count = 0
        h, w = annotated.shape[:2]
        
        if results.boxes is not None:
            for box in results.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                conf = float(box.conf[0].cpu().numpy())
                cls = int(box.cls[0].cpu().numpy())
                class_name = self.model.names[cls]
                
                if self.target_classes and class_name not in self.target_classes:
                    continue

                if self.enable_bbox_heuristics:
                    if not self._bbox_passes_heuristics(
                        x1, y1, x2, y2, float(w), float(h)
                    ):
                        continue

                det_count += 1

                
                # Color based on confidence
                if conf > 0.5:
                    color = (0, 255, 0)  # Green - good
                elif conf > 0.3:
                    color = (0, 255, 255)  # Yellow - ok
                else:
                    color = (0, 165, 255)  # Orange - low conf
                
                cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                label = f'{class_name} {conf:.2f}'
                cv2.putText(annotated, label, (x1, y1-8), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Info
        cv2.putText(annotated, f'YOLO FPS: {self.fps:.1f}', (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(annotated, f'Detections: {det_count}', (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return annotated

    def _bbox_passes_heuristics(self, x1, y1, x2, y2, w, h):
        """Heuristics to reject wall/semantic-mark false positives.

        x1,y1,x2,y2 are image-space pixel coordinates.
        """
        img_area = float(w * h) if w > 0 and h > 0 else 0.0
        if img_area <= 0:
            return True

        box_w = float(max(0.0, x2 - x1))
        box_h = float(max(0.0, y2 - y1))
        box_area = box_w * box_h

        # Reject very large boxes (often walls spanning most of the view)
        if self.max_box_area_ratio > 0 and (box_area / img_area) > self.max_box_area_ratio:
            return False

        # Reject extreme aspect ratios
        if box_h > 1e-6:
            aspect = box_w / box_h
            if aspect < self.min_aspect_ratio:
                return False
            if aspect > self.max_aspect_ratio:
                return False

        # Reject boxes that mostly touch borders
        margin_x = float(self.border_margin_ratio) * float(w)
        margin_y = float(self.border_margin_ratio) * float(h)
        if (
            x1 <= margin_x or x2 >= (float(w) - margin_x) or
            y1 <= margin_y or y2 >= (float(h) - margin_y)
        ):
            return False

        return True

    def extract_detections(self, results, header):

        detections_array = Detection2DArray()
        detections_array.header = header
        img_h = float(getattr(results, 'orig_shape', (0, 0))[0])
        img_w = float(getattr(results, 'orig_shape', (0, 0))[1])
        
        if results.boxes is not None:
            for box in results.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0].cpu().numpy())
                cls = int(box.cls[0].cpu().numpy())
                class_name = self.model.names[cls]
                
                if self.target_classes and class_name not in self.target_classes:
                    continue

                if self.enable_bbox_heuristics and img_w > 0.0 and img_h > 0.0:
                    if not self._bbox_passes_heuristics(x1, y1, x2, y2, img_w, img_h):
                        continue
                
                detection = Detection2D()
                detection.header = header
                detection.bbox.center.position.x = float((x1 + x2) / 2)
                detection.bbox.center.position.y = float((y1 + y2) / 2)
                detection.bbox.size_x = float(x2 - x1)
                detection.bbox.size_y = float(y2 - y1)
                
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = class_name
                hypothesis.hypothesis.score = conf
                detection.results = [hypothesis]
                
                detections_array.detections.append(detection)
        
        return detections_array

    def update_fps(self):
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            elapsed = time.time() - self.last_time
            self.fps = 30 / elapsed
            self.last_time = time.time()

    def print_status(self):
        self.get_logger().info(f'Status: FPS={self.fps:.1f}, model={"loaded" if self.model else "NOT LOADED"}')


def main(args=None):
    rclpy.init(args=args)
    detector = YOLODetector()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
