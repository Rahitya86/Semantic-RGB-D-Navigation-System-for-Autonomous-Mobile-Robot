#!/usr/bin/env python3
"""Resolve simple semantic text commands into Nav2 goal poses."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray


ALIASES = {
    'desk': 'table',
    'cafe': 'table',
    'table': 'table',
    'chair': 'chair',
    'shelf': 'shelf',
    'bookshelf': 'shelf',
    'cabinet': 'cabinet',
    'trash': 'trash_can',
    'barrel': 'barrel',
    'red box': 'red_box',
    'blue box': 'blue_box',
    'green box': 'green_box',
    'yellow box': 'yellow_box',
    'purple box': 'purple_box',
    'box': 'box',
}


class SemanticGoalResolver(Node):
    def __init__(self):
        super().__init__('semantic_goal_resolver')
        self.declare_parameter('command_topic', '/semantic/goal_command')
        self.declare_parameter('landmark_topic', '/semantic/object_landmarks')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('goal_frame_id', 'map')
        self.declare_parameter('robot_frame_id', 'base_link')
        self.declare_parameter('approach_distance_m', 1.35)

        self.goal_frame_id = self.get_parameter('goal_frame_id').value
        self.robot_frame_id = self.get_parameter('robot_frame_id').value
        self.approach_distance_m = float(self.get_parameter('approach_distance_m').value)
        self.latest_landmarks = None
        self.latest_odom = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            String,
            self.get_parameter('command_topic').value,
            self.command_cb,
            10,
        )
        self.create_subscription(
            Detection3DArray,
            self.get_parameter('landmark_topic').value,
            self.landmarks_cb,
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_cb,
            10,
        )
        self.goal_pub = self.create_publisher(PoseStamped, self.get_parameter('goal_topic').value, 10)

        self.get_logger().info(
            'Semantic goal resolver ready. Example: '
            'ros2 topic pub --once /semantic/goal_command std_msgs/msg/String "{data: go to table}"'
        )

    def landmarks_cb(self, msg):
        self.latest_landmarks = msg

    def odom_cb(self, msg):
        self.latest_odom = msg

    def command_cb(self, msg):
        if self.latest_landmarks is None:
            self.get_logger().warn('No semantic landmarks received yet')
            return

        target_class = self.parse_target(msg.data)
        if target_class is None:
            self.get_logger().warn(f'Could not parse semantic target from command: {msg.data}')
            return

        target = self.select_target(target_class)
        if target is None:
            self.get_logger().warn(f'No landmark found for class {target_class}')
            return

        goal = self.make_goal(target)
        self.goal_pub.publish(goal)
        self.get_logger().info(
            f'Published /goal_pose for {target_class} at '
            f'({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})'
        )

    def parse_target(self, command):
        normalized = ' '.join(command.lower().replace('_', ' ').split())
        for phrase, class_name in sorted(ALIASES.items(), key=lambda item: len(item[0]), reverse=True):
            if phrase in normalized:
                return class_name
        return None

    def select_target(self, class_name):
        candidates = []
        for detection in self.latest_landmarks.detections:
            if not detection.results:
                continue
            detected_class = detection.results[0].hypothesis.class_id
            if detected_class == class_name or (class_name == 'box' and detected_class.endswith('_box')):
                candidates.append(detection)

        if not candidates:
            return None
        robot_position = self.current_robot_position()
        if robot_position is None:
            return candidates[0]

        return min(
            candidates,
            key=lambda detection: self.distance_sq(robot_position, detection.bbox.center.position),
        )

    def make_goal(self, detection):
        object_pose = detection.bbox.center
        object_position = object_pose.position

        robot_position = self.current_robot_position()
        if robot_position is not None:
            away_x = robot_position.x - object_position.x
            away_y = robot_position.y - object_position.y
        else:
            away_x = -1.0
            away_y = 0.0

        length = math.hypot(away_x, away_y)
        if length < 1e-4:
            away_x, away_y, length = -1.0, 0.0, 1.0

        unit_x = away_x / length
        unit_y = away_y / length
        goal_x = object_position.x + unit_x * self.approach_distance_m
        goal_y = object_position.y + unit_y * self.approach_distance_m
        yaw = math.atan2(object_position.y - goal_y, object_position.x - goal_x)

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.goal_frame_id
        goal.pose.position.x = float(goal_x)
        goal.pose.position.y = float(goal_y)
        goal.pose.position.z = 0.0
        goal.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.orientation.w = math.cos(yaw * 0.5)
        return goal

    def current_robot_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.goal_frame_id,
                self.robot_frame_id,
                Time(),
                timeout=Duration(seconds=0.15),
            )
            return transform.transform.translation
        except TransformException as exc:
            self.get_logger().warn(
                f'Could not get {self.goal_frame_id}->{self.robot_frame_id} TF, falling back to /odom: {exc}',
                throttle_duration_sec=2.0,
            )

        if self.latest_odom is not None:
            return self.latest_odom.pose.pose.position
        return None

    def distance_sq(self, first, second):
        dx = first.x - second.x
        dy = first.y - second.y
        return dx * dx + dy * dy


def main(args=None):
    rclpy.init(args=args)
    node = SemanticGoalResolver()
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
