#!/usr/bin/env python3
"""Generate the clean K12 Nav2 map and optionally save a raw RTAB map backup."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_output_prefix_arg = DeclareLaunchArgument(
        'map_output_prefix',
        default_value=PathJoinSubstitution([
            EnvironmentVariable('HOME'),
            '.ros',
            'k12_semantic_map',
        ]),
        description='Raw output path without extension. Produces .yaml and .pgm files.',
    )
    save_raw_rtabmap_arg = DeclareLaunchArgument(
        'save_raw_rtabmap',
        default_value='false',
        description='Also save RTAB-Map raw occupancy grid from /rtabmap/map.',
    )
    raw_yaml_arg = DeclareLaunchArgument(
        'raw_yaml',
        default_value=PathJoinSubstitution([
            EnvironmentVariable('HOME'),
            '.ros',
            'k12_semantic_map.yaml',
        ]),
    )
    raw_pgm_arg = DeclareLaunchArgument(
        'raw_pgm',
        default_value=PathJoinSubstitution([
            EnvironmentVariable('HOME'),
            '.ros',
            'k12_semantic_map.pgm',
        ]),
    )
    clean_yaml_arg = DeclareLaunchArgument(
        'clean_yaml',
        default_value=PathJoinSubstitution([
            EnvironmentVariable('HOME'),
            '.ros',
            'k12_semantic_map_clean.yaml',
        ]),
    )
    clean_pgm_arg = DeclareLaunchArgument(
        'clean_pgm',
        default_value=PathJoinSubstitution([
            EnvironmentVariable('HOME'),
            '.ros',
            'k12_semantic_map_clean.pgm',
        ]),
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('k12_description'),
            'worlds',
            'k12_indoor.world',
        ]),
        description='World file used to refine the final navigation map geometry.',
    )
    world_resolution_arg = DeclareLaunchArgument(
        'world_resolution',
        default_value='0.05',
        description='Resolution for the world-refined map.',
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
            '-p',
            'map_topic:=/rtabmap/map',
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('save_raw_rtabmap')),
    )

    clean_map = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'semantic_navigation',
            'clean_2d_map',
            '--input-yaml',
            LaunchConfiguration('raw_yaml'),
            '--input-pgm',
            LaunchConfiguration('raw_pgm'),
            '--output-yaml',
            LaunchConfiguration('clean_yaml'),
            '--output-pgm',
            LaunchConfiguration('clean_pgm'),
            '--world-file',
            LaunchConfiguration('world'),
            '--world-resolution',
            LaunchConfiguration('world_resolution'),
        ],
        output='screen',
    )

    return LaunchDescription([
        map_output_prefix_arg,
        save_raw_rtabmap_arg,
        raw_yaml_arg,
        raw_pgm_arg,
        clean_yaml_arg,
        clean_pgm_arg,
        world_arg,
        world_resolution_arg,
        save_map,
        clean_map,
    ])
