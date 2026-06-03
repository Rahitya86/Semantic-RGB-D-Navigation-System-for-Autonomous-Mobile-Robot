#!/usr/bin/env python3
"""Start RTAB-Map and automatically drive a full-world coverage route."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    reset_rtabmap_db_arg = DeclareLaunchArgument(
        'reset_rtabmap_db',
        default_value='true',
        description='Use true for a fresh full-world map, false to append to the current DB',
    )
    launch_rviz_arg = DeclareLaunchArgument('launch_rviz', default_value='true')
    launch_rtabmap_viz_arg = DeclareLaunchArgument('launch_rtabmap_viz', default_value='true')
    start_semantics_arg = DeclareLaunchArgument('start_semantics', default_value='false')
    auto_drive_delay_arg = DeclareLaunchArgument('auto_drive_delay', default_value='25.0')
    auto_save_map_arg = DeclareLaunchArgument(
        'auto_save_map',
        default_value='true',
        description='Save and clean the 2D map after the full-world mapper finishes',
    )
    map_output_prefix_arg = DeclareLaunchArgument(
        'map_output_prefix',
        default_value=PathJoinSubstitution([
            EnvironmentVariable('HOME'),
            '.ros',
            'k12_semantic_map',
        ]),
        description='Output path without extension. Produces .yaml/.pgm and clean variants.',
    )

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('semantic_navigation'),
                'launch',
                'k12_rgbd_mapping.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'gui': LaunchConfiguration('gui'),
            'reset_rtabmap_db': LaunchConfiguration('reset_rtabmap_db'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'launch_rtabmap_viz': LaunchConfiguration('launch_rtabmap_viz'),
            'start_semantics': LaunchConfiguration('start_semantics'),
        }.items(),
    )

    auto_driver_node = Node(
        package='semantic_navigation',
        executable='full_world_mapper',
        name='full_world_mapper',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'shutdown_when_complete': True,
        }],
    )

    auto_driver = TimerAction(
        period=LaunchConfiguration('auto_drive_delay'),
        actions=[auto_driver_node],
    )

    save_map = ExecuteProcess(
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

    clean_map = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'semantic_navigation',
            'clean_2d_map',
        ],
        output='screen',
    )

    save_when_route_complete = RegisterEventHandler(
        OnProcessExit(
            target_action=auto_driver_node,
            on_exit=[
                TimerAction(period=3.0, actions=[save_map]),
                TimerAction(period=8.0, actions=[clean_map]),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_save_map')),
    )

    return LaunchDescription([
        use_sim_time_arg,
        gui_arg,
        reset_rtabmap_db_arg,
        launch_rviz_arg,
        launch_rtabmap_viz_arg,
        start_semantics_arg,
        auto_drive_delay_arg,
        auto_save_map_arg,
        map_output_prefix_arg,
        mapping,
        auto_driver,
        save_when_route_complete,
    ])
