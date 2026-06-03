#!/usr/bin/env python3
"""Limit and smooth cmd_vel for stable mapping-friendly motion."""

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


class CmdVelLimiter(Node):
    def __init__(self):
        super().__init__('cmd_vel_limiter')

        self.declare_parameter('input_cmd_topic', '/cmd_vel')
        self.declare_parameter('output_cmd_topic', '/cmd_vel_smoothed')
        self.declare_parameter('accept_stamped_cmd', True)
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('cmd_timeout_sec', 0.4)
        self.declare_parameter('max_linear_x', 0.22)
        self.declare_parameter('max_angular_z', 0.55)
        self.declare_parameter('max_linear_accel', 0.45)
        self.declare_parameter('max_angular_accel', 1.00)
        self.declare_parameter('turn_slowdown_threshold', 0.25)
        self.declare_parameter('turn_linear_scale', 0.40)
        self.declare_parameter('linear_deadband', 0.003)
        self.declare_parameter('angular_deadband', 0.006)

        self.input_cmd_topic = self.get_parameter('input_cmd_topic').value
        self.output_cmd_topic = self.get_parameter('output_cmd_topic').value
        self.accept_stamped_cmd = bool(self.get_parameter('accept_stamped_cmd').value)
        self.publish_rate_hz = max(5.0, float(self.get_parameter('publish_rate_hz').value))
        self.cmd_timeout_sec = max(0.1, float(self.get_parameter('cmd_timeout_sec').value))
        self.max_linear_x = max(0.05, float(self.get_parameter('max_linear_x').value))
        self.max_angular_z = max(0.1, float(self.get_parameter('max_angular_z').value))
        self.max_linear_accel = max(0.05, float(self.get_parameter('max_linear_accel').value))
        self.max_angular_accel = max(0.1, float(self.get_parameter('max_angular_accel').value))
        self.turn_slowdown_threshold = max(
            0.0, float(self.get_parameter('turn_slowdown_threshold').value)
        )
        self.turn_linear_scale = _clamp(float(self.get_parameter('turn_linear_scale').value), 0.1, 1.0)
        self.linear_deadband = max(0.0, float(self.get_parameter('linear_deadband').value))
        self.angular_deadband = max(0.0, float(self.get_parameter('angular_deadband').value))

        self.target = Twist()
        self.current = Twist()
        self.last_input_time = self.get_clock().now()
        self.last_update_time = self.get_clock().now()

        self.sub = self.create_subscription(Twist, self.input_cmd_topic, self.cmd_cb, 20)
        self.stamped_sub = None
        if self.accept_stamped_cmd:
            # Allows teleop_twist_keyboard with -p stamped:=true on the same topic.
            self.stamped_sub = self.create_subscription(
                TwistStamped,
                self.input_cmd_topic,
                self.cmd_stamped_cb,
                20,
            )
        self.pub = self.create_publisher(Twist, self.output_cmd_topic, 20)
        self.create_timer(1.0 / self.publish_rate_hz, self.update)

        self.get_logger().info(
            'cmd_vel limiter active: '
            f'{self.input_cmd_topic} -> {self.output_cmd_topic}, '
            f'vmax={self.max_linear_x:.2f} m/s, wmax={self.max_angular_z:.2f} rad/s'
        )

    def cmd_cb(self, msg):
        linear = float(msg.linear.x)
        angular = float(msg.angular.z)

        angular = _clamp(angular, -self.max_angular_z, self.max_angular_z)

        linear_limit = self.max_linear_x
        if abs(angular) > self.turn_slowdown_threshold:
            linear_limit *= self.turn_linear_scale
        linear = _clamp(linear, -linear_limit, linear_limit)

        if abs(linear) < self.linear_deadband:
            linear = 0.0
        if abs(angular) < self.angular_deadband:
            angular = 0.0

        self.target.linear.x = linear
        self.target.angular.z = angular
        self.last_input_time = self.get_clock().now()

    def cmd_stamped_cb(self, msg):
        self.cmd_cb(msg.twist)

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.last_update_time = now

        if (now - self.last_input_time).nanoseconds / 1e9 > self.cmd_timeout_sec:
            target_linear = 0.0
            target_angular = 0.0
        else:
            target_linear = self.target.linear.x
            target_angular = self.target.angular.z

        linear_step = self.max_linear_accel * dt
        angular_step = self.max_angular_accel * dt

        linear_error = target_linear - self.current.linear.x
        angular_error = target_angular - self.current.angular.z

        linear_inc = _clamp(linear_error, -linear_step, linear_step)
        angular_inc = _clamp(angular_error, -angular_step, angular_step)

        self.current.linear.x += linear_inc
        self.current.angular.z += angular_inc

        out = Twist()
        out.linear.x = self.current.linear.x
        out.angular.z = self.current.angular.z
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelLimiter()
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
