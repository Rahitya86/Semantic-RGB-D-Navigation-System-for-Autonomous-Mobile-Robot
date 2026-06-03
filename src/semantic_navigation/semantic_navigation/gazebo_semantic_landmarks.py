#!/usr/bin/env python3
"""Publish semantic object landmarks from named Gazebo models."""

import re

import rclpy
from gazebo_msgs.msg import ModelStates
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray


CLASS_RULES = [
    (re.compile(r'cafe_table|desk|table', re.IGNORECASE), 'table', (0.10, 0.55, 1.00)),
    (re.compile(r'chair|tote', re.IGNORECASE), 'chair', (0.45, 0.80, 1.00)),
    (re.compile(r'bookshelf|shelf', re.IGNORECASE), 'shelf', (0.15, 0.85, 0.30)),
    (re.compile(r'cabinet', re.IGNORECASE), 'cabinet', (0.70, 0.55, 0.25)),
    (re.compile(r'trash', re.IGNORECASE), 'trash_can', (0.35, 0.35, 0.35)),
    (re.compile(r'barrel', re.IGNORECASE), 'barrel', (1.00, 0.45, 0.05)),
    (re.compile(r'red_box', re.IGNORECASE), 'red_box', (1.00, 0.05, 0.05)),
    (re.compile(r'blue_box', re.IGNORECASE), 'blue_box', (0.05, 0.20, 1.00)),
    (re.compile(r'green_box', re.IGNORECASE), 'green_box', (0.05, 0.85, 0.20)),
    (re.compile(r'yellow_box', re.IGNORECASE), 'yellow_box', (1.00, 0.85, 0.05)),
    (re.compile(r'purple_box', re.IGNORECASE), 'purple_box', (0.75, 0.10, 0.95)),
    (re.compile(r'box|case|cone', re.IGNORECASE), 'box', (0.90, 0.55, 0.15)),
]


class GazeboSemanticLandmarks(Node):
    def __init__(self):
        super().__init__('gazebo_semantic_landmarks')
        self.declare_parameter('model_states_topic', '/gazebo/model_states')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('object_topic', '/semantic/object_landmarks')
        self.declare_parameter('marker_topic', '/semantic/object_markers')
        self.declare_parameter('publish_rate_hz', 2.0)

        self.model_states_topic = self.get_parameter('model_states_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.object_topic = self.get_parameter('object_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.publish_rate_hz = max(0.5, float(self.get_parameter('publish_rate_hz').value))

        self.latest_states = None
        self.create_subscription(ModelStates, self.model_states_topic, self.model_states_cb, 10)
        self.detection_pub = self.create_publisher(Detection3DArray, self.object_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.create_timer(1.0 / self.publish_rate_hz, self.publish_landmarks)

        self.get_logger().info(
            f'Publishing Gazebo semantic landmarks from {self.model_states_topic} to {self.object_topic}'
        )

    def model_states_cb(self, msg):
        self.latest_states = msg

    def publish_landmarks(self):
        if self.latest_states is None:
            self.get_logger().warn(
                f'Waiting for Gazebo model states on {self.model_states_topic}',
                throttle_duration_sec=3.0,
            )
            return

        detections = Detection3DArray()
        detections.header.stamp = self.get_clock().now().to_msg()
        detections.header.frame_id = self.frame_id

        markers = MarkerArray()
        seen_ids = set()

        for index, (name, pose) in enumerate(zip(self.latest_states.name, self.latest_states.pose)):
            class_name, color = self.classify(name)
            if class_name is None:
                continue

            detection = Detection3D()
            detection.header = detections.header
            detection.bbox.center = pose
            detection.bbox.size.x = 0.8
            detection.bbox.size.y = 0.8
            detection.bbox.size.z = 0.8
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = class_name
            hypothesis.hypothesis.score = 1.0
            detection.results = [hypothesis]
            detection.id = name
            detections.detections.append(detection)

            marker_id = self.stable_marker_id(name)
            seen_ids.add(marker_id)
            markers.markers.append(self.make_text_marker(marker_id, name, class_name, pose, color))

        self.detection_pub.publish(detections)
        self.marker_pub.publish(markers)

    def classify(self, model_name):
        for pattern, class_name, color in CLASS_RULES:
            if pattern.search(model_name):
                return class_name, color
        return None, None

    def make_text_marker(self, marker_id, model_name, class_name, pose, color):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.frame_id
        marker.ns = 'semantic_objects'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose = pose
        marker.pose.position.z += 1.0
        marker.scale.z = 0.35
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = 1.0
        marker.text = f'{class_name}: {model_name}'
        marker.lifetime.sec = 2
        return marker

    def stable_marker_id(self, name):
        return abs(hash(name)) % 2147483647


def main(args=None):
    rclpy.init(args=args)
    node = GazeboSemanticLandmarks()
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
