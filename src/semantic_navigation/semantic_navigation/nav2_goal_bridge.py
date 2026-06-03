#!/usr/bin/env python3
"""Forward PoseStamped goals to Nav2 NavigateToPose action."""

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class Nav2GoalBridge(Node):
    def __init__(self):
        super().__init__('nav2_goal_bridge')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('action_name', 'navigate_to_pose')
        self.declare_parameter('wait_for_server_timeout_sec', 2.0)

        self.wait_timeout = float(self.get_parameter('wait_for_server_timeout_sec').value)
        self.action_client = ActionClient(
            self,
            NavigateToPose,
            self.get_parameter('action_name').value,
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter('goal_topic').value,
            self.goal_cb,
            10,
        )
        self.get_logger().info('Forwarding /goal_pose messages to Nav2 NavigateToPose')

    def goal_cb(self, pose_msg):
        if not self.action_client.wait_for_server(timeout_sec=self.wait_timeout):
            self.get_logger().warn('Nav2 navigate_to_pose action server is not ready yet')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_msg
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_cb)
        self.get_logger().info(
            f'Sent Nav2 goal at ({pose_msg.pose.position.x:.2f}, {pose_msg.pose.position.y:.2f}) '
            f'in frame {pose_msg.header.frame_id}'
        )

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected the goal')
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        status = future.result().status
        self.get_logger().info(f'Nav2 goal finished with status {status}')


def main(args=None):
    rclpy.init(args=args)
    node = Nav2GoalBridge()
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
