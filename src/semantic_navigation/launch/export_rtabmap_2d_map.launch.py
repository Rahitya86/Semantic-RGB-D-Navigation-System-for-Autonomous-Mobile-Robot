#!/usr/bin/env python3
"""Load the saved RTAB-Map DB and save its /map topic as a Nav2 2D map."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation time while RTAB-Map republishes the saved map',
    )
    launch_gazebo_arg = DeclareLaunchArgument(
        'launch_gazebo',
        default_value='true',
        description='Start Gazebo before exporting. Set false if Gazebo is already running.',
    )
    gui_arg = DeclareLaunchArgument('gui', default_value='false')
    rtabmap_database_path_arg = DeclareLaunchArgument(
        'rtabmap_database_path',
        default_value=PathJoinSubstitution([
            EnvironmentVariable('HOME'),
            '.ros',
            'k12_semantic_rtabmap.db',
        ]),
        description='Saved RTAB-Map database to convert',
    )
    map_output_prefix_arg = DeclareLaunchArgument(
        'map_output_prefix',
        default_value=PathJoinSubstitution([
            EnvironmentVariable('HOME'),
            '.ros',
            'k12_semantic_map',
        ]),
        description='Output path without extension. Produces .yaml and .pgm files.',
    )
    save_delay_arg = DeclareLaunchArgument(
        'save_delay',
        default_value='30.0',
        description='Seconds to wait for /map before saving it',
    )
    rtabmap_start_delay_arg = DeclareLaunchArgument(
        'rtabmap_start_delay',
        default_value='10.0',
        description='Seconds to wait for Gazebo sensors before loading RTAB-Map',
    )
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='0.0')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.0')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='0.20')
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
            'gui': LaunchConfiguration('gui'),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_z': LaunchConfiguration('spawn_z'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_gazebo')),
    )

    rtabmap_localization = TimerAction(
        period=LaunchConfiguration('rtabmap_start_delay'),
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
                    'rtabmap_database_path': LaunchConfiguration('rtabmap_database_path'),
                    'reset_rtabmap_db': 'false',
                    'rtabmap_incremental_memory': 'false',
                    'rtabmap_init_wm_with_all_nodes': 'true',
                    'start_segmenter': 'false',
                    'start_cloud_generator': 'false',
                    'start_gazebo_landmarks': 'false',
                    'start_goal_resolver': 'false',
                    'launch_rtabmap_viz': 'false',
                }.items(),
            )
        ],
    )

    save_map = TimerAction(
        period=LaunchConfiguration('save_delay'),
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'run',
                    'nav2_map_server',
                    'map_saver_cli',
                    '-f',
                    LaunchConfiguration('map_output_prefix'),
                    '--ros-args',
                    '-p',
                    'map_subscribe_transient_local:=true',
                ],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        launch_gazebo_arg,
        gui_arg,
        rtabmap_database_path_arg,
        map_output_prefix_arg,
        save_delay_arg,
        rtabmap_start_delay_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        spawn_yaw_arg,
        gazebo,
        rtabmap_localization,
        save_map,
    ])
