#!/usr/bin/env python3
"""
Semantic perception launch file.

RGB-D semantic segmentation is the default path for K12 Gazebo navigation.
Legacy YOLO/color/combined detectors can still be enabled for comparison.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    enable_rgbd_semantic = DeclareLaunchArgument(
        'enable_rgbd_semantic',
        default_value='true',
        description='Enable RGB-D semantic segmentation',
    )
    enable_yolo = DeclareLaunchArgument(
        'enable_yolo',
        default_value='false',
        description='Enable legacy YOLOv8 detector',
    )
    enable_color = DeclareLaunchArgument(
        'enable_color',
        default_value='false',
        description='Enable legacy HSV color detector',
    )
    enable_combined = DeclareLaunchArgument(
        'enable_combined',
        default_value='false',
        description='Enable legacy combined YOLO/color overlay',
    )

    camera_topic = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/depth_camera/image_raw',
        description='RGB camera topic',
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
        default_value='1',
        description='Run RGB-D segmentation every N RGB frames',
    )
    navigation_frame = DeclareLaunchArgument(
        'navigation_frame',
        default_value='base_link',
        description='Frame for semantic point clouds and local costmap',
    )

    yolo_conf = DeclareLaunchArgument(
        'yolo_conf',
        default_value='0.15',
        description='Legacy YOLO confidence threshold',
    )
    yolo_model = DeclareLaunchArgument(
        'yolo_model',
        default_value='yolov8n.pt',
        description='Legacy YOLO model path or model name',
    )

    view_topic = DeclareLaunchArgument(
        'view_topic',
        default_value='/camera/depth_camera/image_raw',
        description='Image topic shown in rqt_image_view',
    )
    combined_camera_topic = DeclareLaunchArgument(
        'combined_camera_topic',
        default_value='/camera/depth_camera/image_raw',
        description='Legacy combined overlay camera topic',
    )
    combined_image_topic = DeclareLaunchArgument(
        'combined_image_topic',
        default_value='/detections/combined/image',
        description='Legacy combined overlay output topic',
    )
    launch_rqt = DeclareLaunchArgument(
        'launch_rqt',
        default_value='true',
        description='Launch rqt_image_view',
    )

    rgbd_semantic_node = Node(
        package='semantic_object_detection',
        executable='rgbd_semantic_segmenter',
        name='rgbd_semantic_segmenter',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(LaunchConfiguration('enable_rgbd_semantic')),
        parameters=[
            {'rgb_topic': LaunchConfiguration('camera_topic')},
            {'depth_topic': LaunchConfiguration('depth_topic')},
            {'camera_info_topic': LaunchConfiguration('camera_info_topic')},
            {'navigation_frame': LaunchConfiguration('navigation_frame')},
            {
                'process_every_n_frames': ParameterValue(
                    LaunchConfiguration('process_every_n_frames'),
                    value_type=int,
                )
            },
        ],
    )

    yolo_detector_node = Node(
        package='semantic_object_detection',
        executable='yolo_detector',
        name='yolo_detector',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(LaunchConfiguration('enable_yolo')),
        parameters=[
            {'camera_topic': LaunchConfiguration('camera_topic')},
            {'model_path': LaunchConfiguration('yolo_model')},
            {'confidence_threshold': ParameterValue(LaunchConfiguration('yolo_conf'), value_type=float)},
        ],
    )

    color_detector_node = Node(
        package='semantic_object_detection',
        executable='color_detector',
        name='color_detector',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(LaunchConfiguration('enable_color')),
        parameters=[
            {'camera_topic': LaunchConfiguration('camera_topic')},
        ],
    )

    combined_detector_node = Node(
        package='semantic_object_detection',
        executable='combined_detector',
        name='combined_detector',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(LaunchConfiguration('enable_combined')),
        parameters=[
            {'camera_topic': LaunchConfiguration('combined_camera_topic')},
            {'combined_image_topic': LaunchConfiguration('combined_image_topic')},
        ],
    )

    viewer = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view', LaunchConfiguration('view_topic')],
                output='screen',
                condition=IfCondition(LaunchConfiguration('launch_rqt')),
            )
        ],
    )

    return LaunchDescription(
        [
            enable_rgbd_semantic,
            enable_yolo,
            enable_color,
            enable_combined,
            camera_topic,
            depth_topic,
            camera_info_topic,
            process_every_n_frames,
            navigation_frame,
            yolo_conf,
            yolo_model,
            view_topic,
            combined_camera_topic,
            combined_image_topic,
            launch_rqt,
            rgbd_semantic_node,
            yolo_detector_node,
            color_detector_node,
            combined_detector_node,
            viewer,
        ]
    )
