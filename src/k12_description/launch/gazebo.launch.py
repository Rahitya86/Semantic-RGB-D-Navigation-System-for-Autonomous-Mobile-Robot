from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition

import os
import xacro

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # =========================
    # Launch Arguments
    # =========================

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation clock'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('k12_description'),
            'worlds',
            'k12_indoor.world'
        ]),
        description='Gazebo world file'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Launch Gazebo GUI (disable in headless environments)'
    )


    spawn_delay_arg = DeclareLaunchArgument(
        'spawn_delay',
        default_value='8.0',
        description='Delay before spawning robot'
    )

    # =========================
    # SAFE SPAWN POSITION
    # =========================

    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x',
        default_value='0.0',
        description='Robot spawn X position'
    )

    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y',
        default_value='0.0',
        description='Robot spawn Y position'
    )

    # The wheel meshes are centered 0.2 m above base_link and have a 0.2 m
    # radius, so base_link z=0 places the tire bottoms at the floor. Spawn just
    # above contact to avoid initial penetration without dropping the robot.
    spawn_z_arg = DeclareLaunchArgument(
        'spawn_z',
        default_value='0.20',
        description='Robot spawn Z position'
    )

    spawn_yaw_arg = DeclareLaunchArgument(
        'spawn_yaw',
        default_value='0.0',
        description='Robot yaw orientation'
    )

    # =========================
    # Robot Description
    # =========================

    share_dir = get_package_share_directory('k12_description')

    xacro_file = os.path.join(
        share_dir,
        'urdf',
        'k12.xacro'
    )

    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # =========================
    # Robot State Publisher
    # =========================

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_urdf,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )

    # =========================
    # Gazebo Server
    # =========================

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'pause': 'false',
            'verbose': 'true',
            'factory': 'true',
            'server_required': 'true',
        }.items()
    )

    # =========================
    # Gazebo Client GUI
    # =========================

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # =========================
    # Spawn Robot
    # =========================

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'k12',
            '-topic', 'robot_description',
            '-timeout', '120',
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', LaunchConfiguration('spawn_z'),
            '-Y', LaunchConfiguration('spawn_yaw')
        ],
        output='screen'
    )

    # Delay robot spawn until Gazebo fully loads
    spawn_after_delay = TimerAction(
        period=LaunchConfiguration('spawn_delay'),
        actions=[spawn_robot]
    )

    # =========================
    # Launch Description
    # =========================

    return LaunchDescription([

        # Gazebo model paths
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=(
                '/usr/share/gazebo-11/models:'
                + share_dir
                + '/models:'
                + os.path.expanduser('~/.gazebo/models')
            )
        ),

        # Gazebo resource/material paths
        SetEnvironmentVariable(
            name='GAZEBO_RESOURCE_PATH',
            value='/usr/share/gazebo-11:/usr/share/gazebo-11/media'
        ),


        # Force software rendering to avoid GLX/GPU issues in headless/remote sessions
        SetEnvironmentVariable(
            name='LIBGL_ALWAYS_SOFTWARE',
            value='1'
        ),
        SetEnvironmentVariable(
            name='__GLX_VENDOR_LIBRARY_NAME',
            value='mesa'
        ),


        # Disable online model database (avoid downloads during launch)
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_DATABASE_URI',
            value=''
        ),

        # Arguments
        use_sim_time_arg,
        world_arg,
        gui_arg,
        spawn_delay_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        spawn_yaw_arg,

        # Nodes
        robot_state_publisher_node,
        gazebo_server,
        gazebo_client,
        spawn_after_delay,
    ])
