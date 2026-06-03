#!/usr/bin/env python3
"""Start Gazebo, RGB-D semantic RTAB-Map, RViz2, and rtabmapviz for K12."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation time',
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('k12_description'),
            'worlds',
            'k12_indoor.world',
        ]),
        description='Gazebo world file',
    )
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Launch Gazebo GUI',
    )
    launch_gazebo_arg = DeclareLaunchArgument(
        'launch_gazebo',
        default_value='false',
        description='Start Gazebo from this mapping launch. Set true only if Gazebo is not already running.',
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Open RViz2 with the mapping display config',
    )
    launch_rtabmap_viz_arg = DeclareLaunchArgument(
        'launch_rtabmap_viz',
        default_value='true',
        description='Open real-time RTAB-Map visualization',
    )
    reset_rtabmap_db_arg = DeclareLaunchArgument(
        'reset_rtabmap_db',
        default_value='false',
        description='Delete old RTAB-Map DB before mapping to avoid stacked/duplicate maps',
    )
    process_every_n_frames_arg = DeclareLaunchArgument(
        'process_every_n_frames',
        default_value='1',
        description='Run semantic segmentation every N RGB-D frames',
    )
    rgbd_odom_wait_imu_to_init_arg = DeclareLaunchArgument(
        'rgbd_odom_wait_imu_to_init',
        default_value='false',
        description='Wait for IMU orientation before RGB-D odometry starts',
    )
    start_semantics_arg = DeclareLaunchArgument(
        'start_semantics',
        default_value='true',
        description='Start semantic segmentation and semantic cloud generation during mapping',
    )
    mapping_start_delay_arg = DeclareLaunchArgument(
        'mapping_start_delay',
        default_value='10.0',
        description='Seconds to wait for Gazebo sensors before starting RTAB-Map',
    )
    rviz_start_delay_arg = DeclareLaunchArgument(
        'rviz_start_delay',
        default_value='12.0',
        description='Seconds to wait before opening RViz2',
    )
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='0.0')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.0')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='0.05')
    spawn_yaw_arg = DeclareLaunchArgument('spawn_yaw', default_value='0.0')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('k12_description'),
                'launch',
                'gazebo.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_z': LaunchConfiguration('spawn_z'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_gazebo')),
    )

    semantic_mapping = TimerAction(
        period=LaunchConfiguration('mapping_start_delay'),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('semantic_navigation'),
                        'launch',
                        'semantic_navigation.launch.py',
                    ])
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'launch_rtabmap_viz': LaunchConfiguration('launch_rtabmap_viz'),
                    'reset_rtabmap_db': LaunchConfiguration('reset_rtabmap_db'),
                    'process_every_n_frames': LaunchConfiguration('process_every_n_frames'),
                    'rgbd_odom_wait_imu_to_init': LaunchConfiguration('rgbd_odom_wait_imu_to_init'),
                    'start_segmenter': LaunchConfiguration('start_semantics'),
                    'start_cloud_generator': LaunchConfiguration('start_semantics'),
                    'start_gazebo_landmarks': 'false',
                    'start_goal_resolver': 'false',
                }.items(),
            )
        ],
    )

    rviz = TimerAction(
        period=LaunchConfiguration('rviz_start_delay'),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('k12_description'),
                        'launch',
                        'view_robot.launch.py',
                    ])
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'use_robot_state_publisher': 'false',
                    'use_joint_state_publisher': 'false',
                    'launch_semantic_navigation': 'false',
                }.items(),
                condition=IfCondition(LaunchConfiguration('launch_rviz')),
            )
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        world_arg,
        gui_arg,
        launch_gazebo_arg,
        launch_rviz_arg,
        launch_rtabmap_viz_arg,
        reset_rtabmap_db_arg,
        process_every_n_frames_arg,
        rgbd_odom_wait_imu_to_init_arg,
        start_semantics_arg,
        mapping_start_delay_arg,
        rviz_start_delay_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        spawn_yaw_arg,
        gazebo,
        semantic_mapping,
        rviz,
    ])
