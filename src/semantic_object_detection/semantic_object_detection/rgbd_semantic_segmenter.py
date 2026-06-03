#!/usr/bin/env python3
"""
RGB-D semantic segmentation for the K12 Gazebo world.

This node replaces YOLO-style bounding boxes with dense semantic products:
semantic label image, color overlay, semantic point cloud, Nav2 obstacle cloud,
and a local occupancy grid derived from semantic classes.
"""

import json
import math
from dataclasses import dataclass

import cv2
from message_filters import ApproximateTimeSynchronizer, Subscriber, TimeSynchronizer
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


@dataclass(frozen=True)
class SemanticClass:
    label_id: int
    name: str
    color: tuple
    cost: int
    navigable: bool


SEMANTIC_CLASSES = [
    SemanticClass(0, 'unknown', (0, 0, 0), -1, False),
    SemanticClass(1, 'floor', (80, 220, 210), 0, True),
    SemanticClass(2, 'wall', (150, 150, 150), 100, False),
    SemanticClass(3, 'red_box', (0, 0, 255), 100, False),
    SemanticClass(4, 'blue_box', (255, 0, 0), 100, False),
    SemanticClass(5, 'green_box', (0, 255, 0), 100, False),
    SemanticClass(6, 'yellow_box', (0, 255, 255), 100, False),
    SemanticClass(7, 'purple_box', (255, 0, 255), 100, False),
    SemanticClass(8, 'object', (0, 140, 255), 100, False),
    SemanticClass(9, 'wall_marker', (255, 255, 0), 80, False),
]

CLASS_BY_ID = {semantic_class.label_id: semantic_class for semantic_class in SEMANTIC_CLASSES}
CLASS_ID_BY_NAME = {semantic_class.name: semantic_class.label_id for semantic_class in SEMANTIC_CLASSES}


class RGBDSemanticSegmenter(Node):
    def __init__(self):
        super().__init__('rgbd_semantic_segmenter')
        self.bridge = CvBridge()

        self.declare_parameter('rgb_topic', '/camera/depth_camera/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth_camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/depth_camera/camera_info')
        self.declare_parameter('process_every_n_frames', 2)
        self.declare_parameter('use_approx_sync', True)
        self.declare_parameter('sync_queue_size', 20)
        self.declare_parameter('sync_slop_sec', 0.04)
        self.declare_parameter('min_depth_m', 0.12)
        self.declare_parameter('max_depth_m', 8.0)
        self.declare_parameter('point_decimation', 4)
        self.declare_parameter('object_min_area_px', 180)
        self.declare_parameter('floor_start_ratio', 0.58)
        self.declare_parameter('wall_stop_ratio', 0.78)
        self.declare_parameter('depth_edge_threshold_m', 0.16)
        self.declare_parameter('camera_offset_xyz', '0.59,0.0,0.32')
        self.declare_parameter('navigation_frame', 'base_link')
        self.declare_parameter('publish_semantic_cloud', True)
        self.declare_parameter('publish_local_costmap', True)
        self.declare_parameter('costmap_resolution', 0.05)
        self.declare_parameter('costmap_width_m', 8.0)
        self.declare_parameter('costmap_height_m', 6.0)
        self.declare_parameter('costmap_inflation_radius_m', 0.18)

        self.rgb_topic = self.get_parameter('rgb_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.process_every_n_frames = max(1, int(self.get_parameter('process_every_n_frames').value))
        self.use_approx_sync = bool(self.get_parameter('use_approx_sync').value)
        self.sync_queue_size = max(5, int(self.get_parameter('sync_queue_size').value))
        self.sync_slop_sec = max(0.0, float(self.get_parameter('sync_slop_sec').value))
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        self.point_decimation = max(1, int(self.get_parameter('point_decimation').value))
        self.object_min_area_px = int(self.get_parameter('object_min_area_px').value)
        self.floor_start_ratio = float(self.get_parameter('floor_start_ratio').value)
        self.wall_stop_ratio = float(self.get_parameter('wall_stop_ratio').value)
        self.depth_edge_threshold_m = float(self.get_parameter('depth_edge_threshold_m').value)
        self.camera_offset_xyz = self._parse_xyz(self.get_parameter('camera_offset_xyz').value)
        self.navigation_frame = self.get_parameter('navigation_frame').value
        self.publish_semantic_cloud = bool(self.get_parameter('publish_semantic_cloud').value)
        self.publish_local_costmap = bool(self.get_parameter('publish_local_costmap').value)
        self.costmap_resolution = float(self.get_parameter('costmap_resolution').value)
        self.costmap_width_m = float(self.get_parameter('costmap_width_m').value)
        self.costmap_height_m = float(self.get_parameter('costmap_height_m').value)
        self.costmap_inflation_radius_m = float(self.get_parameter('costmap_inflation_radius_m').value)

        self.camera_info = None
        self.frame_count = 0

        self.rgb_sub = Subscriber(self, Image, self.rgb_topic, qos_profile=qos_profile_sensor_data)
        self.depth_sub = Subscriber(self, Image, self.depth_topic, qos_profile=qos_profile_sensor_data)
        if self.use_approx_sync:
            self.sync = ApproximateTimeSynchronizer(
                [self.rgb_sub, self.depth_sub],
                queue_size=self.sync_queue_size,
                slop=self.sync_slop_sec,
            )
        else:
            self.sync = TimeSynchronizer(
                [self.rgb_sub, self.depth_sub],
                queue_size=self.sync_queue_size,
            )
        self.sync.registerCallback(self.rgbd_cb)
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_cb,
            qos_profile_sensor_data,
        )

        self.label_pub = self.create_publisher(Image, '/semantic/label_image', 10)
        self.color_pub = self.create_publisher(Image, '/semantic/color_image', 10)
        self.overlay_pub = self.create_publisher(Image, '/semantic/overlay_image', 10)
        self.detections_pub = self.create_publisher(Detection2DArray, '/semantic/detections', 10)
        self.class_info_pub = self.create_publisher(String, '/semantic/classes', 1)
        self.semantic_cloud_pub = self.create_publisher(PointCloud2, '/semantic/points', 10)
        self.nav_obstacle_pub = self.create_publisher(PointCloud2, '/semantic/navigation_obstacles', 10)
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/semantic/local_costmap', 10)

        self.create_timer(2.0, self.publish_class_info)

        self.get_logger().info(
            'RGB-D semantic segmenter ready. '
            f'rgb={self.rgb_topic}, depth={self.depth_topic}, info={self.camera_info_topic}'
        )
        self.get_logger().info(
            'Publishing: /semantic/label_image, /semantic/overlay_image, '
            '/semantic/points, /semantic/navigation_obstacles, /semantic/local_costmap'
        )
        self.get_logger().info(
            'RGB-D sync mode: '
            f'{"approximate" if self.use_approx_sync else "exact"} '
            f'(queue={self.sync_queue_size}, slop={self.sync_slop_sec:.3f}s)'
        )

    def _parse_xyz(self, value):
        if isinstance(value, str):
            parts = [p.strip() for p in value.split(',') if p.strip()]
        else:
            parts = list(value)
        if len(parts) != 3:
            return [0.59, 0.0, 0.32]
        return [float(p) for p in parts]

    def camera_info_cb(self, msg):
        self.camera_info = msg

    def rgbd_cb(self, rgb_msg, depth_msg):
        self.frame_count += 1
        if self.frame_count % self.process_every_n_frames != 0:
            return

        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            depth = self._depth_to_meters(depth, depth_msg.encoding)
        except Exception as exc:
            self.get_logger().error(f'Failed to decode synchronized RGB-D frame: {exc}', throttle_duration_sec=2.0)
            return

        if depth.shape[:2] != rgb.shape[:2]:
            depth = cv2.resize(depth, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST)

        labels = self.segment(rgb, depth)
        color_image = self.colorize_labels(labels)
        overlay = cv2.addWeighted(rgb, 0.58, color_image, 0.42, 0.0)
        detections = self.extract_detections(labels, rgb_msg.header)

        self.publish_images(labels, color_image, overlay, rgb_msg.header)
        self.detections_pub.publish(detections)

        if self.publish_semantic_cloud:
            semantic_cloud, obstacle_cloud = self.create_clouds(labels, depth, rgb_msg.header)
            self.semantic_cloud_pub.publish(semantic_cloud)
            self.nav_obstacle_pub.publish(obstacle_cloud)

        if self.publish_local_costmap:
            self.costmap_pub.publish(self.create_costmap(labels, depth, rgb_msg.header))

    def _depth_to_meters(self, depth, encoding):
        depth = np.asarray(depth)
        if encoding == '16UC1' or depth.dtype == np.uint16:
            depth = depth.astype(np.float32) * 0.001
        else:
            depth = depth.astype(np.float32)
        depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
        return depth

    def segment(self, rgb, depth):
        height, width = depth.shape[:2]
        labels = np.zeros((height, width), dtype=np.uint8)
        valid_depth = (depth >= self.min_depth_m) & (depth <= self.max_depth_m)

        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        colored_box_mask = np.zeros((height, width), dtype=np.uint8)
        self._apply_color_class(labels, colored_box_mask, hsv, 'red_box')
        self._apply_color_class(labels, colored_box_mask, hsv, 'blue_box')
        self._apply_color_class(labels, colored_box_mask, hsv, 'green_box')
        self._apply_color_class(labels, colored_box_mask, hsv, 'yellow_box')
        self._apply_color_class(labels, colored_box_mask, hsv, 'purple_box')

        row_indices = np.indices((height, width), dtype=np.int32)[0]
        floor_rows = row_indices >= int(height * self.floor_start_ratio)
        wall_rows = row_indices <= int(height * self.wall_stop_ratio)

        colored = colored_box_mask > 0
        floor_mask = valid_depth & floor_rows & ~colored
        labels[floor_mask] = CLASS_ID_BY_NAME['floor']

        object_mask = self._object_mask_from_rgbd(rgb, hsv, depth, valid_depth, colored, floor_rows)
        labels[object_mask] = CLASS_ID_BY_NAME['object']

        wall_marker_mask = self._wall_marker_mask(hsv, labels, valid_depth)
        labels[wall_marker_mask] = CLASS_ID_BY_NAME['wall_marker']

        wall_mask = valid_depth & wall_rows & (labels == CLASS_ID_BY_NAME['unknown'])
        labels[wall_mask] = CLASS_ID_BY_NAME['wall']

        return labels

    def _apply_color_class(self, labels, combined_mask, hsv, class_name):
        ranges = {
            'red_box': [([0, 80, 80], [20, 255, 255]), ([165, 80, 80], [180, 255, 255])],
            'blue_box': [([90, 60, 60], [140, 255, 255])],
            'green_box': [([30, 60, 60], [90, 255, 255])],
            'yellow_box': [([10, 60, 80], [50, 255, 255])],
            'purple_box': [([120, 50, 50], [165, 255, 255])],
        }
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges[class_name]:
            mask = cv2.bitwise_or(
                mask,
                cv2.inRange(hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8)),
            )

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        class_id = CLASS_ID_BY_NAME[class_name]
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 900.0:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            aspect = float(w) / float(h) if h > 0 else 0.0
            if w < 18 or h < 18 or aspect < 0.35 or aspect > 3.0:
                continue
            cv2.drawContours(combined_mask, [contour], -1, 255, thickness=cv2.FILLED)
            cv2.drawContours(labels, [contour], -1, class_id, thickness=cv2.FILLED)

    def _object_mask_from_rgbd(self, rgb, hsv, depth, valid_depth, colored, floor_rows):
        saturation = hsv[:, :, 1]
        value = hsv[:, :, 2]

        depth_blur = cv2.GaussianBlur(depth, (5, 5), 0)
        grad_x = cv2.Sobel(depth_blur, cv2.CV_32F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(depth_blur, cv2.CV_32F, 0, 1, ksize=3)
        depth_edges = np.sqrt(grad_x * grad_x + grad_y * grad_y) > self.depth_edge_threshold_m

        strong_texture = (saturation > 45) & (value > 35)
        not_lower_floor_band = ~floor_rows
        candidate = valid_depth & ~colored & not_lower_floor_band & (strong_texture | depth_edges)

        gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
        image_edges = cv2.Canny(gray, 50, 130) > 0
        candidate |= valid_depth & ~colored & not_lower_floor_band & image_edges

        mask = candidate.astype(np.uint8) * 255
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.dilate(mask, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        filtered = np.zeros_like(mask)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) >= self.object_min_area_px:
                cv2.drawContours(filtered, [contour], -1, 255, thickness=cv2.FILLED)

        return filtered > 0

    def _wall_marker_mask(self, hsv, labels, valid_depth):
        saturation = hsv[:, :, 1]
        value = hsv[:, :, 2]
        hue = hsv[:, :, 0]
        colored_marker = valid_depth & (labels == 0) & (saturation > 55) & (value > 60)
        marker_hues = (
            ((hue >= 0) & (hue <= 20)) |
            ((hue >= 90) & (hue <= 145)) |
            ((hue >= 20) & (hue <= 45)) |
            ((hue >= 120) & (hue <= 170))
        )
        return colored_marker & marker_hues

    def colorize_labels(self, labels):
        color_image = np.zeros((labels.shape[0], labels.shape[1], 3), dtype=np.uint8)
        for class_id, semantic_class in CLASS_BY_ID.items():
            color_image[labels == class_id] = semantic_class.color
        return color_image

    def publish_images(self, labels, color_image, overlay, header):
        label_msg = self.bridge.cv2_to_imgmsg(labels, encoding='mono8')
        label_msg.header = header
        self.label_pub.publish(label_msg)

        color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        color_msg.header = header
        self.color_pub.publish(color_msg)

        overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        overlay_msg.header = header
        self.overlay_pub.publish(overlay_msg)

    def extract_detections(self, labels, header):
        detections_array = Detection2DArray()
        detections_array.header = header

        for class_id, semantic_class in CLASS_BY_ID.items():
            if class_id in (0, 1):
                continue
            mask = (labels == class_id).astype(np.uint8) * 255
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.object_min_area_px:
                    continue
                x, y, w, h = cv2.boundingRect(contour)
                detection = Detection2D()
                detection.header = header
                detection.bbox.center.position.x = float(x + w / 2.0)
                detection.bbox.center.position.y = float(y + h / 2.0)
                detection.bbox.size_x = float(w)
                detection.bbox.size_y = float(h)
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = semantic_class.name
                hypothesis.hypothesis.score = 1.0
                detection.results = [hypothesis]
                detections_array.detections.append(detection)

        return detections_array

    def _camera_intrinsics(self, width, height):
        if self.camera_info is not None and len(self.camera_info.k) >= 6 and self.camera_info.k[0] > 0:
            return (
                float(self.camera_info.k[0]),
                float(self.camera_info.k[4]),
                float(self.camera_info.k[2]),
                float(self.camera_info.k[5]),
            )

        hfov = 1.047
        fx = float(width) / (2.0 * math.tan(hfov / 2.0))
        fy = fx
        return fx, fy, float(width) / 2.0, float(height) / 2.0

    def create_clouds(self, labels, depth, source_header):
        height, width = depth.shape[:2]
        fx, fy, cx, cy = self._camera_intrinsics(width, height)
        step = self.point_decimation

        semantic_points = []
        obstacle_xyz = []
        valid = (depth >= self.min_depth_m) & (depth <= self.max_depth_m)

        for v in range(0, height, step):
            for u in range(0, width, step):
                if not valid[v, u]:
                    continue
                class_id = int(labels[v, u])
                if class_id == CLASS_ID_BY_NAME['unknown']:
                    continue
                x, y, z = self._pixel_to_base_point(u, v, float(depth[v, u]), fx, fy, cx, cy)
                semantic_class = CLASS_BY_ID[class_id]
                rgb_uint = self._pack_rgb(semantic_class.color)
                semantic_points.append((x, y, z, rgb_uint, class_id))
                if not semantic_class.navigable and class_id != CLASS_ID_BY_NAME['unknown']:
                    obstacle_xyz.append((x, y, z))

        header = Header()
        header.stamp = source_header.stamp
        header.frame_id = self.navigation_frame

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
            PointField(name='label', offset=16, datatype=PointField.UINT32, count=1),
        ]

        semantic_cloud = point_cloud2.create_cloud(header, fields, semantic_points)
        obstacle_cloud = point_cloud2.create_cloud_xyz32(header, obstacle_xyz)
        return semantic_cloud, obstacle_cloud

    def _pixel_to_base_point(self, u, v, depth_m, fx, fy, cx, cy):
        x_right = (float(u) - cx) * depth_m / fx
        y_down = (float(v) - cy) * depth_m / fy
        x_forward = depth_m
        y_left = -x_right
        z_up = -y_down
        return (
            x_forward + self.camera_offset_xyz[0],
            y_left + self.camera_offset_xyz[1],
            z_up + self.camera_offset_xyz[2],
        )

    def _pack_rgb(self, bgr):
        b, g, r = bgr
        return (int(r) << 16) | (int(g) << 8) | int(b)

    def create_costmap(self, labels, depth, source_header):
        height, width = depth.shape[:2]
        fx, fy, cx, cy = self._camera_intrinsics(width, height)
        res = self.costmap_resolution
        grid_w = int(self.costmap_width_m / res)
        grid_h = int(self.costmap_height_m / res)
        origin_x = 0.0
        origin_y = -self.costmap_height_m / 2.0

        grid = np.full((grid_h, grid_w), -1, dtype=np.int8)
        valid = (depth >= self.min_depth_m) & (depth <= self.max_depth_m)

        for v in range(0, height, self.point_decimation):
            for u in range(0, width, self.point_decimation):
                if not valid[v, u]:
                    continue
                class_id = int(labels[v, u])
                if class_id == CLASS_ID_BY_NAME['unknown']:
                    continue

                x, y, _ = self._pixel_to_base_point(u, v, float(depth[v, u]), fx, fy, cx, cy)
                gx = int((x - origin_x) / res)
                gy = int((y - origin_y) / res)
                if gx < 0 or gx >= grid_w or gy < 0 or gy >= grid_h:
                    continue

                semantic_class = CLASS_BY_ID[class_id]
                if semantic_class.navigable:
                    if grid[gy, gx] < 0:
                        grid[gy, gx] = 0
                else:
                    grid[gy, gx] = max(grid[gy, gx], semantic_class.cost)

        self._inflate_costmap(grid)

        msg = OccupancyGrid()
        msg.header.stamp = source_header.stamp
        msg.header.frame_id = self.navigation_frame
        msg.info = MapMetaData()
        msg.info.resolution = float(res)
        msg.info.width = grid_w
        msg.info.height = grid_h
        msg.info.origin.position.x = float(origin_x)
        msg.info.origin.position.y = float(origin_y)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.flatten(order='C').astype(np.int8).tolist()
        return msg

    def _inflate_costmap(self, grid):
        inflation_cells = int(self.costmap_inflation_radius_m / self.costmap_resolution)
        if inflation_cells <= 0:
            return
        occupied = (grid >= 80).astype(np.uint8) * 255
        kernel_size = inflation_cells * 2 + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        inflated = cv2.dilate(occupied, kernel, iterations=1) > 0
        grid[(inflated) & (grid < 80)] = 80

    def publish_class_info(self):
        msg = String()
        msg.data = json.dumps(
            {
                semantic_class.label_id: {
                    'name': semantic_class.name,
                    'bgr': semantic_class.color,
                    'cost': semantic_class.cost,
                    'navigable': semantic_class.navigable,
                }
                for semantic_class in SEMANTIC_CLASSES
            },
            sort_keys=True,
        )
        self.class_info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RGBDSemanticSegmenter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
