#!/usr/bin/env python3
"""Drive a slow coverage route through the K12 indoor world for RTAB-Map."""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class FullWorldMapper(Node):
    def __init__(self):
        super().__init__('full_world_mapper')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('linear_speed', 0.12)
        self.declare_parameter('angular_speed', 0.45)
        self.declare_parameter('xy_tolerance', 0.35)
        self.declare_parameter('yaw_tolerance', 0.20)
        self.declare_parameter('pause_at_waypoint_sec', 1.5)
        self.declare_parameter('shutdown_when_complete', True)
        self.declare_parameter('complete_shutdown_delay_sec', 2.0)

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.xy_tolerance = float(self.get_parameter('xy_tolerance').value)
        self.yaw_tolerance = float(self.get_parameter('yaw_tolerance').value)
        self.pause_at_waypoint_sec = float(self.get_parameter('pause_at_waypoint_sec').value)
        self.shutdown_when_complete = bool(self.get_parameter('shutdown_when_complete').value)
        self.complete_shutdown_delay_sec = float(self.get_parameter('complete_shutdown_delay_sec').value)

        self.route = [
            (0.0, 0.0, 0.0),
            (0.0, 5.4, math.pi / 2.0),
            (0.0, 11.8, math.pi / 2.0),
            (-2.4, 14.2, math.pi),
            (2.5, 14.0, 0.0),
            (0.0, 11.8, -math.pi / 2.0),
            (0.0, 0.0, 0.0),
            # Keep the robot off the center line here; the lobby red box sits near x=0.
            (1.25, -2.8, -math.pi / 2.0),
            (1.25, -7.5, -math.pi / 2.0),
            (1.25, -12.0, -math.pi / 2.0),
            (1.25, -16.5, -math.pi / 2.0),
            (-1.25, -16.5, math.pi),
            (-1.25, -12.0, math.pi / 2.0),
            (-1.25, -6.4, math.pi / 2.0),
            (1.25, -6.4, 0.0),
            (1.25, -6.4, math.pi / 2.0),
            (1.25, -2.8, math.pi / 2.0),
            (0.0, 0.0, 0.0),
        ]
        self.index = 0
        self.latest_pose = None
        self.pause_until = None
        self.finished = False
        self.shutdown_timer = None

        self.cmd_pub = self.create_publisher(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_cb,
            10,
        )
        self.timer = self.create_timer(0.1, self.control_cb)
        self.get_logger().info(
            f'Full-world mapping route ready with {len(self.route)} waypoints. '
            'Drive slowly and keep RTAB-Map running until route complete.'
        )

    def odom_cb(self, msg):
        pose = msg.pose.pose
        q = pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.latest_pose = (pose.position.x, pose.position.y, yaw)

    def control_cb(self):
        if self.latest_pose is None or self.finished:
            return

        now = self.get_clock().now()
        if self.pause_until is not None:
            if now < self.pause_until:
                self.stop()
                return
            self.pause_until = None

        if self.index >= len(self.route):
            self.finished = True
            self.stop()
            self.get_logger().info('Full-world mapping route complete. Saving the 2D map now.')
            if self.shutdown_when_complete:
                self.shutdown_timer = self.create_timer(
                    self.complete_shutdown_delay_sec,
                    self.shutdown_cb,
                )
            return

        x, y, yaw = self.latest_pose
        target_x, target_y, target_yaw = self.route[self.index]
        dx = target_x - x
        dy = target_y - y
        distance = math.hypot(dx, dy)
        target_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(target_heading - yaw)
        final_yaw_error = normalize_angle(target_yaw - yaw)

        cmd = Twist()
        if distance > self.xy_tolerance:
            if abs(heading_error) > 0.35:
                cmd.angular.z = self.clamp(heading_error, -self.angular_speed, self.angular_speed)
            else:
                cmd.linear.x = min(self.linear_speed, 0.45 * distance)
                cmd.angular.z = self.clamp(1.3 * heading_error, -self.angular_speed, self.angular_speed)
        elif abs(final_yaw_error) > self.yaw_tolerance:
            cmd.angular.z = self.clamp(final_yaw_error, -self.angular_speed, self.angular_speed)
        else:
            self.get_logger().info(
                f'Reached waypoint {self.index + 1}/{len(self.route)}: '
                f'({target_x:.1f}, {target_y:.1f})'
            )
            self.index += 1
            self.pause_until = now + rclpy.duration.Duration(seconds=self.pause_at_waypoint_sec)
        self.cmd_pub.publish(cmd)

    def stop(self):
        self.cmd_pub.publish(Twist())

    def shutdown_cb(self):
        if self.shutdown_timer is not None:
            self.shutdown_timer.cancel()
        self.stop()
        self.get_logger().info('Full-world mapper finished; exiting so map_saver can run.')
        rclpy.shutdown()

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))


def main(args=None):
    rclpy.init(args=args)
    node = FullWorldMapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
