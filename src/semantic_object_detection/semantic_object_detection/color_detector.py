#!/usr/bin/env python3
"""
ROS2 Color Detection Node - FIXED VERSION
With more flexible HSV ranges and debugging
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
import logging

logging.basicConfig(
    level=logging.INFO,
    format='[%(levelname)s] [%(name)s]: %(message)s'
)
logger = logging.getLogger('ColorDetector')


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        
        self.get_logger().info('=== Color Detector Starting (FIXED) ===')
        
        self.bridge = CvBridge()

        # For this K12 setup, only depth camera raw image is typically published.
        # Color detector expects BGR8, so it may not work with depth-only streams.
        # Default to the depth camera topic to keep the pipeline alive; user can override.
        self.declare_parameter('camera_topic', '/camera/depth_camera/image_raw')

        self.declare_parameter('min_contour_area', 30.0)  # LOWERED for testing
        self.declare_parameter('debug_mode', True)

        # Whitelist: only accept the colored boxes that exist in k12_indoor.world
        # (these are the 5 colored primitives named obstacle_*_box_*)
        self.declare_parameter(
            'allowed_box_labels',
            'red_box,blue_box,green_box,yellow_box,purple_box'
        )

        allowed_box_labels = self.get_parameter('allowed_box_labels').value
        self.allowed_box_labels = {c.strip() for c in allowed_box_labels.split(',') if c.strip()}


        self.camera_topic = self.get_parameter('camera_topic').value
        self.min_contour_area = self.get_parameter('min_contour_area').value
        self.debug_mode = self.get_parameter('debug_mode').value

        # MORE FLEXIBLE HSV ranges for Gazebo simulation
        self.color_ranges = {
            'red_box': {
                'ranges': [
                    (np.array([0, 80, 80]), np.array([20, 255, 255])),      # Red-low
                    (np.array([165, 80, 80]), np.array([180, 255, 255])), # Red-high
                ],
                'color': (0, 0, 255)
            },
            'blue_box': {
                'ranges': [
                    (np.array([90, 60, 60]), np.array([140, 255, 255])),  # Blue
                ],
                'color': (255, 0, 0)
            },
            'green_box': {
                'ranges': [
                    (np.array([30, 60, 60]), np.array([90, 255, 255])),    # Green
                ],
                'color': (0, 255, 0)
            },
            'yellow_box': {
                'ranges': [
                    (np.array([10, 60, 80]), np.array([50, 255, 255])), # Yellow/Orange
                ],
                'color': (0, 255, 255)
            },
            'purple_box': {
                'ranges': [
                    (np.array([120, 50, 50]), np.array([165, 255, 255])), # Purple/Magenta
                ],
                'color': (255, 0, 255)
            }
        }
        
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        
        # Debug kernels for each color
        self.debug_masks = {}
        
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            qos_profile_sensor_data
        )
        
        self.color_image_pub = self.create_publisher(
            Image,
            '/semantic/color_image',
            10
        )
        
        self.color_detections_pub = self.create_publisher(
            Detection2DArray,
            '/semantic/color_detections',
            10
        )
        
        # Debug publishers - visualize masks
        if self.debug_mode:
            self.debug_pub = self.create_publisher(Image, '/semantic/debug_mask', 10)
        
        self.get_logger().info(f'Subscribed to: {self.camera_topic}')
        self.get_logger().info(f'Min contour area: {self.min_contour_area}')
        self.get_logger().info('Publishing: /semantic/color_detections')
        self.get_logger().info('Detecting: Red, Blue, Green, Yellow, Purple boxes')

        # Whitelist-derived box label set is stored in self.allowed_box_labels.
        # If the camera sees other colored artifacts, they will be ignored.


    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Debug: show sample pixel HSV values
            if self.debug_mode:
                center_h, center_s, center_v = hsv_image[hsv_image.shape[0]//2, hsv_image.shape[1]//2]
                self.get_logger().debug(f'Center pixel HSV: H={center_h}, S={center_s}, V={center_v}')
            
            all_detections = []
            annotated = cv_image.copy()
            
            # Combined mask for debugging
            combined_mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
            
            # Only process colors that are present in the world whitelist
            for color_name, color_info in self.color_ranges.items():
                if self.allowed_box_labels and color_name not in self.allowed_box_labels:
                    continue

                # Create mask
                mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)

                for lower, upper in color_info['ranges']:
                    mask = cv2.bitwise_or(mask, cv2.inRange(hsv_image, lower, upper))
                
                # Morphological cleanup
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=1)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=1)

                # Dilate slightly to recover missing pixels on the box silhouette
                # (helps with viewpoint where only a side/edge might otherwise be visible)
                mask = cv2.dilate(mask, self.kernel, iterations=1)

                
                # Save for debug visualization
                self.debug_masks[color_name] = mask
                combined_mask = cv2.bitwise_or(combined_mask, mask)
                
                # Find contours
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                self.get_logger().debug(f'{color_name}: {len(contours)} contours found')
                
                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area < self.min_contour_area:
                        continue
                    
                    # Use a rotated rectangle (minAreaRect) to better cover the full box extent,
                    # then convert to an axis-aligned bbox.
                    rect = cv2.minAreaRect(contour)  # ((cx,cy),(w,h),angle)
                    box_points = cv2.boxPoints(rect)  # 4 corner points
                    box_points = np.int32(box_points)

                    x1 = int(np.min(box_points[:, 0]))
                    y1 = int(np.min(box_points[:, 1]))
                    x2 = int(np.max(box_points[:, 0]))
                    y2 = int(np.max(box_points[:, 1]))
                    w = max(0, x2 - x1)
                    h = max(0, y2 - y1)
                    x, y = x1, y1

                    M = cv2.moments(contour)

                    if M['m00'] > 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                    else:
                        cx, cy = x + w//2, y + h//2
                    
                    # Draw
                    cv2.rectangle(annotated, (x, y), (x+w, y+h), color_info['color'], 2)
                    cv2.circle(annotated, (cx, cy), 5, color_info['color'], -1)
                    label = f'{color_name} {area:.0f}px'
                    cv2.putText(annotated, label, (x, y-8), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_info['color'], 2)
                    
                    self.get_logger().debug(f'Detected: {color_name} at ({cx},{cy}) area={area:.0f}')
                    
                    all_detections.append({
                        'label': color_name,
                        'x': x, 'y': y, 'w': w, 'h': h,
                        'cx': cx, 'cy': cy, 'area': area
                    })
            
            # Draw count
            cv2.putText(annotated, f'Color Dets: {len(all_detections)}', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Publish
            color_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            color_msg.header = msg.header
            self.color_image_pub.publish(color_msg)
            
            # Publish detections
            detections_array = self.extract_detections(all_detections, msg.header)
            self.color_detections_pub.publish(detections_array)
            
            # Debug: combined mask visualization
            if self.debug_mode and self.debug_masks:
                # Colorize the combined mask for visualization
                mask_colored = cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)
                # Add colored overlay
                for color_name, color_info in self.color_ranges.items():
                    if color_name in self.debug_masks:
                        mask_colored[np.where(self.debug_masks[color_name] > 0)] = color_info['color']
                
                debug_msg = self.bridge.cv2_to_imgmsg(mask_colored, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_pub.publish(debug_msg)
            
            # Log detections
            if len(all_detections) > 0:
                self.get_logger().info(f'✓ Detected {len(all_detections)} colored boxes')
            
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def extract_detections(self, detections, header):
        detections_array = Detection2DArray()
        detections_array.header = header
        
        for det in detections:
            detection = Detection2D()
            detection.header = header
            detection.bbox.center.position.x = float(det['cx'])
            detection.bbox.center.position.y = float(det['cy'])
            detection.bbox.size_x = float(det['w'])
            detection.bbox.size_y = float(det['h'])
            
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = det['label']
            hypothesis.hypothesis.score = 1.0
            detection.results = [hypothesis]
            
            detections_array.detections.append(detection)
        
        return detections_array


def main(args=None):
    rclpy.init(args=args)
    detector = ColorDetector()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()