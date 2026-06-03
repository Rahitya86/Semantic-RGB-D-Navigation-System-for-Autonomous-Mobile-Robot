#!/usr/bin/env python3
"""Run autonomous Nav2 navigation using a saved RTAB-Map localization database."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    launch_gazebo_arg = DeclareLaunchArgument(
        'launch_gazebo',
        default_value='true',
        description='Start Gazebo and spawn the robot. Set false if Gazebo is already running.',
    )
    gui_arg = DeclareLaunchArgument('gui', default_value='false')
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            EnvironmentVariable('HOME'),
            '.ros',
            'k12_semantic_map_clean.yaml',
        ]),
        description='Cleaned 2D Nav2 map yaml exported from RTAB-Map',
    )
    rtabmap_database_path_arg = DeclareLaunchArgument(
        'rtabmap_database_path',
        default_value=PathJoinSubstitution([
            EnvironmentVariable('HOME'),
            '.ros',
            'k12_semantic_rtabmap.db',
        ]),
        description='Saved RTAB-Map DB used for visual localization',
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('semantic_navigation'),
            'config',
            'nav2_k12_semantic.yaml',
        ]),
        description='Nav2 parameters with semantic obstacle costmaps',
    )
    launch_rviz_arg = DeclareLaunchArgument('launch_rviz', default_value='true')
    start_rtabmap_localization_arg = DeclareLaunchArgument(
        'start_rtabmap_localization',
        default_value='false',
        description='Start RTAB-Map visual localization. For Gazebo navigation, false uses stable odom with static map->odom.',
    )
    use_static_map_to_odom_tf_arg = DeclareLaunchArgument(
        'use_static_map_to_odom_tf',
        default_value='true',
        description='Publish identity map->odom transform for the clean Gazebo/Nav2 map.',
    )
    semantic_approach_distance_arg = DeclareLaunchArgument(
        'semantic_approach_distance',
        default_value='1.35',
        description='Meters to stop away from a semantic target object',
    )
    mapping_start_delay_arg = DeclareLaunchArgument('mapping_start_delay', default_value='10.0')
    nav2_start_delay_arg = DeclareLaunchArgument('nav2_start_delay', default_value='16.0')
    rviz_start_delay_arg = DeclareLaunchArgument('rviz_start_delay', default_value='18.0')
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
                    'rtabmap_database_path': LaunchConfiguration('rtabmap_database_path'),
                    'reset_rtabmap_db': 'false',
                    'rtabmap_incremental_memory': 'false',
                    'rtabmap_init_wm_with_all_nodes': 'true',
                    'start_segmenter': 'true',
                    'start_cloud_generator': 'true',
                    'start_gazebo_landmarks': 'true',
                    'start_goal_resolver': 'true',
                    'semantic_approach_distance': LaunchConfiguration('semantic_approach_distance'),
                    'start_rtabmap': LaunchConfiguration('start_rtabmap_localization'),
                    'launch_rtabmap_viz': 'false',
                }.items(),
            )
        ],
    )

    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=IfCondition(LaunchConfiguration('use_static_map_to_odom_tf')),
    )

    map_server = TimerAction(
        period=LaunchConfiguration('nav2_start_delay'),
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'yaml_filename': LaunchConfiguration('map')},
                ],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_map_server',
                output='screen',
                parameters=[
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'autostart': True},
                    {'node_names': ['map_server']},
                ],
            ),
        ],
    )

    nav2 = TimerAction(
        period=LaunchConfiguration('nav2_start_delay'),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('nav2_bringup'),
                        'launch',
                        'navigation_launch.py',
                    ])
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'params_file': LaunchConfiguration('params_file'),
                    'autostart': 'true',
                    'use_composition': 'False',
                }.items(),
            )
        ],
    )

    goal_bridge = TimerAction(
        period=LaunchConfiguration('rviz_start_delay'),
        actions=[
            Node(
                package='semantic_navigation',
                executable='nav2_goal_bridge',
                name='nav2_goal_bridge',
                output='screen',
                parameters=[
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                ],
            )
        ],
    )

    rviz = TimerAction(
        period=LaunchConfiguration('rviz_start_delay'),
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=[
                    '-d',
                    PathJoinSubstitution([
                        FindPackageShare('nav2_bringup'),
                        'rviz',
                        'nav2_default_view.rviz',
                    ]),
                ],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                condition=IfCondition(LaunchConfiguration('launch_rviz')),
            )
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        launch_gazebo_arg,
        gui_arg,
        map_arg,
        rtabmap_database_path_arg,
        params_file_arg,
        launch_rviz_arg,
        start_rtabmap_localization_arg,
        use_static_map_to_odom_tf_arg,
        semantic_approach_distance_arg,
        mapping_start_delay_arg,
        nav2_start_delay_arg,
        rviz_start_delay_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        spawn_yaw_arg,
        gazebo,
        static_map_to_odom,
        rtabmap_localization,
        map_server,
        nav2,
        goal_bridge,
        rviz,
    ])
