#!/usr/bin/env python3
"""Launch RGB-D semantic segmentation for K12 Gazebo navigation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    rgb_topic = DeclareLaunchArgument(
        'rgb_topic',
        default_value='/camera/depth_camera/image_raw',
        description='RGB image topic',
    )
    depth_topic = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/depth_camera/depth/image_raw',
        description='Registered depth image topic',
    )
    camera_info_topic = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/depth_camera/camera_info',
        description='CameraInfo topic',
    )
    process_every_n_frames = DeclareLaunchArgument(
        'process_every_n_frames',
        default_value='2',
        description='Run semantic segmentation every N RGB frames',
    )
    navigation_frame = DeclareLaunchArgument(
        'navigation_frame',
        default_value='base_link',
        description='Frame for semantic clouds and local costmap',
    )
    launch_rqt = DeclareLaunchArgument(
        'launch_rqt',
        default_value='true',
        description='Open rqt_image_view on the live RGB camera feed',
    )

    segmenter = Node(
        package='semantic_object_detection',
        executable='rgbd_semantic_segmenter',
        name='rgbd_semantic_segmenter',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {'rgb_topic': LaunchConfiguration('rgb_topic')},
            {'depth_topic': LaunchConfiguration('depth_topic')},
            {'camera_info_topic': LaunchConfiguration('camera_info_topic')},
            {
                'process_every_n_frames': ParameterValue(
                    LaunchConfiguration('process_every_n_frames'),
                    value_type=int,
                )
            },
            {'navigation_frame': LaunchConfiguration('navigation_frame')},
        ],
    )

    viewer = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view', '/camera/depth_camera/image_raw'],
                output='screen',
                condition=IfCondition(LaunchConfiguration('launch_rqt')),
            )
        ],
    )

    return LaunchDescription(
        [
            rgb_topic,
            depth_topic,
            camera_info_topic,
            process_every_n_frames,
            navigation_frame,
            launch_rqt,
            segmenter,
            viewer,
        ]
    )
