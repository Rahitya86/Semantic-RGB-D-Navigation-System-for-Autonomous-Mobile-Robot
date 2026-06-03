#!/usr/bin/env python3
"""Fuse RGB-D and semantic label images into semantic PointCloud2 products."""

import math

import cv2
from message_filters import ApproximateTimeSynchronizer, Subscriber, TimeSynchronizer
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener


SEMANTIC_COLORS_BGR = {
    0: (0, 0, 0),
    1: (80, 220, 210),
    2: (150, 150, 150),
    3: (0, 0, 255),
    4: (255, 0, 0),
    5: (0, 255, 0),
    6: (0, 255, 255),
    7: (255, 0, 255),
    8: (0, 140, 255),
    9: (255, 255, 0),
}


class SemanticCloudGenerator(Node):
    def __init__(self):
        super().__init__('semantic_cloud_generator')
        self.bridge = CvBridge()

        self.declare_parameter('rgb_topic', '/camera/depth_camera/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth_camera/depth/image_raw')
        self.declare_parameter('semantic_topic', '/semantic/label_image')
        self.declare_parameter('camera_info_topic', '/camera/depth_camera/camera_info')

        self.declare_parameter('semantic_points_topic', '/semantic/points')
        self.declare_parameter('obstacle_points_topic', '/semantic/navigation_obstacles')
        self.declare_parameter('semantic_map_points_topic', '/semantic/map_points')
        self.declare_parameter('obstacle_map_points_topic', '/semantic/map_obstacles')

        self.declare_parameter('output_frame', 'base_link')
        self.declare_parameter('accumulation_frame', 'odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('camera_offset_xyz', '0.59,0.0,0.32')
        self.declare_parameter('use_camera_tf', True)
        self.declare_parameter('point_decimation', 6)
        self.declare_parameter('min_depth_m', 0.12)
        self.declare_parameter('max_depth_m', 6.0)
        self.declare_parameter('use_approx_sync', True)
        self.declare_parameter('sync_queue_size', 20)
        self.declare_parameter('sync_slop_sec', 0.05)
        self.declare_parameter('publish_rate_hz', 6.0)
        self.declare_parameter('publish_map_clouds', True)
        self.declare_parameter('map_publish_rate_hz', 2.0)
        self.declare_parameter('map_voxel_size_m', 0.10)
        self.declare_parameter('max_map_points', 100000)
        self.declare_parameter('unknown_label_id', 0)
        self.declare_parameter('obstacle_label_ids', '2,3,4,5,6,7,8,9')
        self.declare_parameter('colorize_by_semantic_label', True)
        self.declare_parameter('semantic_color_order', 'bgr')

        self.rgb_topic = self.get_parameter('rgb_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.semantic_topic = self.get_parameter('semantic_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.semantic_points_topic = self.get_parameter('semantic_points_topic').value
        self.obstacle_points_topic = self.get_parameter('obstacle_points_topic').value
        self.semantic_map_points_topic = self.get_parameter('semantic_map_points_topic').value
        self.obstacle_map_points_topic = self.get_parameter('obstacle_map_points_topic').value
        self.output_frame = self.get_parameter('output_frame').value
        self.accumulation_frame = self.get_parameter('accumulation_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.camera_offset_xyz = self._parse_xyz(self.get_parameter('camera_offset_xyz').value)
        self.use_camera_tf = bool(self.get_parameter('use_camera_tf').value)
        self.point_decimation = max(1, int(self.get_parameter('point_decimation').value))
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        self.use_approx_sync = bool(self.get_parameter('use_approx_sync').value)
        self.sync_queue_size = max(5, int(self.get_parameter('sync_queue_size').value))
        self.sync_slop_sec = max(0.0, float(self.get_parameter('sync_slop_sec').value))
        self.publish_rate_hz = max(0.5, float(self.get_parameter('publish_rate_hz').value))
        self.publish_map_clouds = bool(self.get_parameter('publish_map_clouds').value)
        self.map_publish_rate_hz = max(0.2, float(self.get_parameter('map_publish_rate_hz').value))
        self.map_voxel_size_m = max(0.01, float(self.get_parameter('map_voxel_size_m').value))
        self.max_map_points = max(1000, int(self.get_parameter('max_map_points').value))
        self.unknown_label_id = int(self.get_parameter('unknown_label_id').value)
        self.obstacle_label_ids = self._parse_label_ids(self.get_parameter('obstacle_label_ids').value)
        self.colorize_by_semantic_label = bool(self.get_parameter('colorize_by_semantic_label').value)
        self.semantic_color_order = str(self.get_parameter('semantic_color_order').value).lower()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.semantic_accum_points = {}
        self.obstacle_accum_points = {}
        self.latest_sample = None
        self.last_processed_stamp_ns = -1
        self.camera_info = None
        self.last_publish_stamp = None
        self.map_publish_period_ns = int(1e9 / self.map_publish_rate_hz)
        self.last_map_publish_ns = 0

        self.rgb_sub = Subscriber(self, Image, self.rgb_topic, qos_profile=qos_profile_sensor_data)
        self.depth_sub = Subscriber(self, Image, self.depth_topic, qos_profile=qos_profile_sensor_data)
        self.semantic_sub = Subscriber(self, Image, self.semantic_topic, qos_profile=qos_profile_sensor_data)
        if self.use_approx_sync:
            self.sync = ApproximateTimeSynchronizer(
                [self.rgb_sub, self.depth_sub, self.semantic_sub],
                queue_size=self.sync_queue_size,
                slop=self.sync_slop_sec,
            )
        else:
            self.sync = TimeSynchronizer(
                [self.rgb_sub, self.depth_sub, self.semantic_sub],
                queue_size=self.sync_queue_size,
            )
        self.sync.registerCallback(self.synced_inputs_cb)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_cb, qos_profile_sensor_data)

        self.semantic_cloud_pub = self.create_publisher(PointCloud2, self.semantic_points_topic, 10)
        self.obstacle_cloud_pub = self.create_publisher(PointCloud2, self.obstacle_points_topic, 10)
        self.semantic_map_cloud_pub = self.create_publisher(PointCloud2, self.semantic_map_points_topic, 10)
        self.obstacle_map_cloud_pub = self.create_publisher(PointCloud2, self.obstacle_map_points_topic, 10)
        self.create_timer(1.0 / self.publish_rate_hz, self.publish_clouds)

        self.get_logger().info(
            'Semantic cloud generator ready: '
            f'rgb={self.rgb_topic}, depth={self.depth_topic}, semantic={self.semantic_topic}'
        )
        self.get_logger().info(
            f'Publishing {self.semantic_points_topic} and {self.obstacle_points_topic} in {self.output_frame}'
        )
        self.get_logger().info(
            f'Publishing persistent {self.semantic_map_points_topic} and '
            f'{self.obstacle_map_points_topic} in {self.map_frame} '
            f'from accumulated {self.accumulation_frame} points'
        )
        self.get_logger().info(
            'Sync mode: '
            f'{"approximate" if self.use_approx_sync else "exact"} '
            f'(queue={self.sync_queue_size}, slop={self.sync_slop_sec:.3f}s)'
        )

    def synced_inputs_cb(self, rgb_msg, depth_msg, semantic_msg):
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            semantic = self.bridge.imgmsg_to_cv2(semantic_msg, desired_encoding='passthrough')
            self.latest_sample = (
                rgb,
                self._depth_to_meters(depth, depth_msg.encoding),
                self._semantic_to_labels(semantic, semantic_msg.encoding),
                rgb_msg.header,
            )
        except Exception as exc:
            self.get_logger().error(f'Failed to decode synchronized RGB-D-semantic sample: {exc}', throttle_duration_sec=2.0)

    def camera_info_cb(self, msg):
        self.camera_info = msg

    def publish_clouds(self):
        if self.latest_sample is None:
            self.get_logger().warn(
                'Waiting for RGB, depth, and semantic label topics before publishing /semantic/points',
                throttle_duration_sec=3.0,
            )
            return

        rgb, depth_raw, labels_raw, source_header = self.latest_sample
        stamp_ns = int(source_header.stamp.sec) * 1000000000 + int(source_header.stamp.nanosec)
        if stamp_ns == self.last_processed_stamp_ns:
            return

        depth = self._resize_to_rgb(depth_raw, rgb, cv2.INTER_NEAREST)
        labels = self._resize_to_rgb(labels_raw, rgb, cv2.INTER_NEAREST)

        if labels.ndim != 2:
            labels = labels[:, :, 0]

        semantic_cloud, obstacle_cloud, semantic_points, obstacle_points = self._create_live_clouds(
            rgb, depth, labels, source_header
        )
        self.semantic_cloud_pub.publish(semantic_cloud)
        self.obstacle_cloud_pub.publish(obstacle_cloud)
        self.last_publish_stamp = semantic_cloud.header.stamp
        self.last_processed_stamp_ns = stamp_ns

        if self.publish_map_clouds:
            base_to_accum = self.lookup_transform(
                self.accumulation_frame,
                self.output_frame,
                semantic_cloud.header.stamp,
            )
            if base_to_accum is not None:
                self.accumulate_points(semantic_points, obstacle_points, base_to_accum)
            self.publish_persistent_map_clouds_if_due(semantic_cloud.header.stamp)

    def _create_live_clouds(self, rgb, depth, labels, source_header):
        height, width = depth.shape[:2]
        fx, fy, cx, cy = self._camera_intrinsics(width, height)
        valid = (depth >= self.min_depth_m) & (depth <= self.max_depth_m)
        camera_to_output = None
        camera_frame = source_header.frame_id
        if self.use_camera_tf and camera_frame:
            camera_to_output = self.lookup_transform(
                self.output_frame,
                camera_frame,
                source_header.stamp,
            )

        semantic_points = []
        obstacle_points = []

        for v in range(0, height, self.point_decimation):
            for u in range(0, width, self.point_decimation):
                if not valid[v, u]:
                    continue

                label_id = int(labels[v, u])
                if label_id == self.unknown_label_id:
                    continue

                if camera_to_output is not None:
                    camera_point = self._pixel_to_camera_point(u, v, float(depth[v, u]), fx, fy, cx, cy)
                    x, y, z = self._transform_xyz_point(camera_point, camera_to_output)
                else:
                    x, y, z = self._pixel_to_base_point(u, v, float(depth[v, u]), fx, fy, cx, cy)
                color = SEMANTIC_COLORS_BGR.get(label_id, tuple(int(c) for c in rgb[v, u]))
                if not self.colorize_by_semantic_label:
                    color = tuple(int(c) for c in rgb[v, u])
                rgb_uint = self._pack_rgb(color)
                semantic_points.append((x, y, z, rgb_uint, label_id))

                if label_id in self.obstacle_label_ids:
                    obstacle_points.append((x, y, z))

        header = Header()
        header.stamp = source_header.stamp
        header.frame_id = self.output_frame

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
            PointField(name='label', offset=16, datatype=PointField.UINT32, count=1),
        ]
        return (
            point_cloud2.create_cloud(header, fields, semantic_points),
            point_cloud2.create_cloud_xyz32(header, obstacle_points),
            semantic_points,
            obstacle_points,
        )

    def accumulate_points(self, semantic_points, obstacle_points, base_to_accum):
        for point in semantic_points:
            accum_point = self._transform_semantic_point(point, base_to_accum)
            key = self._semantic_voxel_key(accum_point)
            self.semantic_accum_points[key] = accum_point

        for point in obstacle_points:
            accum_point = self._transform_xyz_point(point, base_to_accum)
            key = self._obstacle_voxel_key(accum_point)
            self.obstacle_accum_points[key] = accum_point

        self._trim_map(self.semantic_accum_points)
        self._trim_map(self.obstacle_accum_points)

    def publish_persistent_map_clouds_if_due(self, stamp):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_map_publish_ns < self.map_publish_period_ns:
            return
        self.last_map_publish_ns = now_ns

        if self.accumulation_frame == self.map_frame:
            semantic_map_points = list(self.semantic_accum_points.values())
            obstacle_map_points = list(self.obstacle_accum_points.values())
        else:
            accum_to_map = self.lookup_transform(
                self.map_frame,
                self.accumulation_frame,
                stamp_msg=None,
            )
            if accum_to_map is None:
                return
            semantic_map_points = [
                self._transform_semantic_point(point, accum_to_map)
                for point in self.semantic_accum_points.values()
            ]
            obstacle_map_points = [
                self._transform_xyz_point(point, accum_to_map)
                for point in self.obstacle_accum_points.values()
            ]

        header = Header()
        header.stamp = stamp
        header.frame_id = self.map_frame
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
            PointField(name='label', offset=16, datatype=PointField.UINT32, count=1),
        ]
        self.semantic_map_cloud_pub.publish(
            point_cloud2.create_cloud(header, fields, semantic_map_points)
        )
        self.obstacle_map_cloud_pub.publish(
            point_cloud2.create_cloud_xyz32(header, obstacle_map_points)
        )

    def lookup_transform(self, target_frame, source_frame, stamp_msg=None):
        lookup_time = Time()
        if stamp_msg is not None:
            lookup_time = Time.from_msg(stamp_msg)

        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                lookup_time,
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            if stamp_msg is not None:
                try:
                    return self.tf_buffer.lookup_transform(
                        target_frame,
                        source_frame,
                        Time(),
                        timeout=Duration(seconds=0.05),
                    )
                except TransformException:
                    pass
            self.get_logger().warn(
                f'Waiting for TF {target_frame} <- {source_frame}: {exc}',
                throttle_duration_sec=3.0,
            )
            return None

    def _transform_semantic_point(self, point, transform):
        x, y, z = self._transform_xyz(point[0], point[1], point[2], transform)
        return (x, y, z, point[3], point[4])

    def _transform_xyz_point(self, point, transform):
        return self._transform_xyz(point[0], point[1], point[2], transform)

    def _transform_xyz(self, x, y, z, transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w

        tx = (1.0 - 2.0 * (qy * qy + qz * qz)) * x
        tx += 2.0 * (qx * qy - qz * qw) * y
        tx += 2.0 * (qx * qz + qy * qw) * z

        ty = 2.0 * (qx * qy + qz * qw) * x
        ty += (1.0 - 2.0 * (qx * qx + qz * qz)) * y
        ty += 2.0 * (qy * qz - qx * qw) * z

        tz = 2.0 * (qx * qz - qy * qw) * x
        tz += 2.0 * (qy * qz + qx * qw) * y
        tz += (1.0 - 2.0 * (qx * qx + qy * qy)) * z

        return (
            tx + translation.x,
            ty + translation.y,
            tz + translation.z,
        )

    def _semantic_voxel_key(self, point):
        return (
            int(round(point[0] / self.map_voxel_size_m)),
            int(round(point[1] / self.map_voxel_size_m)),
            int(round(point[2] / self.map_voxel_size_m)),
            int(point[4]),
        )

    def _obstacle_voxel_key(self, point):
        return (
            int(round(point[0] / self.map_voxel_size_m)),
            int(round(point[1] / self.map_voxel_size_m)),
            int(round(point[2] / self.map_voxel_size_m)),
        )

    def _trim_map(self, points):
        while len(points) > self.max_map_points:
            points.pop(next(iter(points)))

    def _camera_intrinsics(self, width, height):
        if self.camera_info is not None and len(self.camera_info.k) >= 6 and self.camera_info.k[0] > 0.0:
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

    def _pixel_to_base_point(self, u, v, depth_m, fx, fy, cx, cy):
        x_right, y_down, z_forward = self._pixel_to_camera_point(u, v, depth_m, fx, fy, cx, cy)
        x_forward = z_forward
        y_left = -x_right
        z_up = -y_down
        return (
            x_forward + self.camera_offset_xyz[0],
            y_left + self.camera_offset_xyz[1],
            z_up + self.camera_offset_xyz[2],
        )

    def _pixel_to_camera_point(self, u, v, depth_m, fx, fy, cx, cy):
        x_right = (float(u) - cx) * depth_m / fx
        y_down = (float(v) - cy) * depth_m / fy
        z_forward = depth_m
        return (x_right, y_down, z_forward)

    def _semantic_to_labels(self, semantic, encoding):
        semantic = np.asarray(semantic)
        if semantic.ndim == 2:
            return semantic.astype(np.uint16)

        color_image = semantic[:, :, :3]
        if 'rgb' in encoding.lower() or self.semantic_color_order == 'rgb':
            color_image = color_image[:, :, ::-1]

        labels = np.zeros(color_image.shape[:2], dtype=np.uint16)
        for label_id, bgr in SEMANTIC_COLORS_BGR.items():
            target = np.array(bgr, dtype=np.int16)
            distance = np.linalg.norm(color_image.astype(np.int16) - target, axis=2)
            labels[distance <= 36.0] = label_id
        return labels

    def _resize_to_rgb(self, image, rgb, interpolation):
        if image.shape[:2] == rgb.shape[:2]:
            return image
        return cv2.resize(image, (rgb.shape[1], rgb.shape[0]), interpolation=interpolation)

    def _depth_to_meters(self, depth, encoding):
        depth = np.asarray(depth)
        if encoding == '16UC1' or depth.dtype == np.uint16:
            depth = depth.astype(np.float32) * 0.001
        else:
            depth = depth.astype(np.float32)
        return np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)

    def _parse_xyz(self, value):
        if isinstance(value, str):
            parts = [part.strip() for part in value.split(',') if part.strip()]
        else:
            parts = list(value)
        if len(parts) != 3:
            self.get_logger().warn('camera_offset_xyz must contain 3 values; using 0.59,0.0,0.32')
            return [0.59, 0.0, 0.32]
        return [float(part) for part in parts]

    def _parse_label_ids(self, value):
        if isinstance(value, str):
            parts = [part.strip() for part in value.split(',') if part.strip()]
        else:
            parts = list(value)
        return {int(part) for part in parts}

    def _pack_rgb(self, bgr):
        b, g, r = bgr
        return (int(r) << 16) | (int(g) << 8) | int(b)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticCloudGenerator()
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
