from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
import xacro

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time when visualizing with Gazebo'
    )

    use_joint_state_publisher_arg = DeclareLaunchArgument(
        'use_joint_state_publisher',
        default_value='true',
        description=(
            'Start joint_state_publisher for standalone RViz viewing. '
            'Set false when Gazebo is already publishing /joint_states.'
        )
    )

    use_robot_state_publisher_arg = DeclareLaunchArgument(
        'use_robot_state_publisher',
        default_value='true',
        description='Start robot_state_publisher. Set false when Gazebo launch already started it.'
    )

    launch_semantic_navigation_arg = DeclareLaunchArgument(
        'launch_semantic_navigation',
        default_value='false',
        description='Start RGB-D semantic navigation and RTAB-Map with this RViz view'
    )

    launch_rtabmap_viz_arg = DeclareLaunchArgument(
        'launch_rtabmap_viz',
        default_value='true',
        description='Open rtabmapviz when semantic navigation is launched from this file'
    )

    package_share = get_package_share_directory('k12_description')
    xacro_file = os.path.join(package_share, 'urdf', 'k12.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    rviz_config = PathJoinSubstitution([
        FindPackageShare('k12_description'),
        'rviz',
        'k12_robot.rviz'
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        condition=IfCondition(LaunchConfiguration('use_robot_state_publisher'))
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        condition=IfCondition(LaunchConfiguration('use_joint_state_publisher'))
    )

    semantic_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('semantic_navigation'),
                'launch',
                'semantic_navigation.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'launch_rtabmap_viz': LaunchConfiguration('launch_rtabmap_viz'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_semantic_navigation'))
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_joint_state_publisher_arg,
        use_robot_state_publisher_arg,
        launch_semantic_navigation_arg,
        launch_rtabmap_viz_arg,
        robot_state_publisher,
        joint_state_publisher,
        semantic_navigation,
        rviz2,
    ])
