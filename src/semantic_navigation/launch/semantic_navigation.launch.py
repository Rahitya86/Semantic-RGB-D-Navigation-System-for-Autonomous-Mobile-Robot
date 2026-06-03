#!/usr/bin/env python3
"""Launch RGB-D semantic navigation for the K12 robot."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation time',
    )
    rgb_topic_arg = DeclareLaunchArgument(
        'rgb_topic',
        default_value='/camera/depth_camera/image_raw',
        description='RGB image topic from the RGB-D camera',
    )
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/depth_camera/depth/image_raw',
        description='Registered depth image topic from the RGB-D camera',
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/depth_camera/camera_info',
        description='CameraInfo topic for the RGB-D camera',
    )
    start_rgbd_sync_arg = DeclareLaunchArgument(
        'start_rgbd_sync',
        default_value='true',
        description='Pre-synchronize RGB, depth, and camera info into rgbd_image for RTAB-Map',
    )
    start_rgbd_odometry_arg = DeclareLaunchArgument(
        'start_rgbd_odometry',
        default_value='false',
        description='Start RTAB-Map rgbd_odometry instead of using wheel odometry directly',
    )
    rgbd_odom_wait_imu_to_init_arg = DeclareLaunchArgument(
        'rgbd_odom_wait_imu_to_init',
        default_value='false',
        description='Wait for IMU orientation before RGB-D odometry starts',
    )
    rtabmap_rgbd_topic_arg = DeclareLaunchArgument(
        'rtabmap_rgbd_topic',
        default_value='/camera/rgbd_image',
        description='Synchronized RGBDImage topic consumed by RTAB-Map and rtabmap_viz',
    )
    rtabmap_odom_topic_arg = DeclareLaunchArgument(
        'rtabmap_odom_topic',
        default_value='/odom',
        description='Odometry topic consumed by RTAB-Map SLAM',
    )
    rgbd_sync_slop_arg = DeclareLaunchArgument(
        'rgbd_sync_slop',
        default_value='0.03',
        description='Approximate sync max interval in seconds for rgbd_sync',
    )
    semantic_topic_arg = DeclareLaunchArgument(
        'semantic_topic',
        default_value='/semantic/label_image',
        description='Semantic label image topic. mono8/16UC1 labels are preferred.',
    )
    navigation_frame_arg = DeclareLaunchArgument(
        'navigation_frame',
        default_value='base_link',
        description='Frame used for semantic point clouds and local costmaps',
    )
    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value='map',
        description='Fixed global frame published by RTAB-Map',
    )
    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame published by Gazebo diff drive',
    )
    point_decimation_arg = DeclareLaunchArgument(
        'point_decimation',
        default_value='4',
        description='Use every Nth pixel when creating semantic point clouds',
    )
    process_every_n_frames_arg = DeclareLaunchArgument(
        'process_every_n_frames',
        default_value='1',
        description='Run semantic segmentation every N RGB-D frames',
    )
    start_cmd_vel_limiter_arg = DeclareLaunchArgument(
        'start_cmd_vel_limiter',
        default_value='false',
        description='Smooth and limit /cmd_vel before it reaches the Gazebo diff drive',
    )
    cmd_vel_input_topic_arg = DeclareLaunchArgument(
        'cmd_vel_input_topic',
        default_value='/cmd_vel',
        description='Raw velocity command topic (teleop/Nav2 input)',
    )
    cmd_vel_output_topic_arg = DeclareLaunchArgument(
        'cmd_vel_output_topic',
        default_value='/cmd_vel_smoothed',
        description='Smoothed velocity command topic consumed by Gazebo diff drive',
    )
    cmd_vel_max_linear_arg = DeclareLaunchArgument(
        'cmd_vel_max_linear',
        default_value='0.25',
        description='Maximum linear speed (m/s) for mapping stability',
    )
    cmd_vel_max_angular_arg = DeclareLaunchArgument(
        'cmd_vel_max_angular',
        default_value='1.20',
        description='Maximum angular speed (rad/s) for mapping stability',
    )
    cmd_vel_linear_accel_arg = DeclareLaunchArgument(
        'cmd_vel_linear_accel',
        default_value='0.60',
        description='Maximum linear acceleration (m/s^2)',
    )
    cmd_vel_angular_accel_arg = DeclareLaunchArgument(
        'cmd_vel_angular_accel',
        default_value='2.00',
        description='Maximum angular acceleration (rad/s^2)',
    )
    cmd_vel_turn_linear_scale_arg = DeclareLaunchArgument(
        'cmd_vel_turn_linear_scale',
        default_value='0.60',
        description='Linear speed scale when angular speed is high',
    )
    rgbd_linear_update_arg = DeclareLaunchArgument(
        'rgbd_linear_update',
        default_value='0.25',
        description='Meters the robot must move before RTAB-Map creates a new keyframe',
    )
    rgbd_angular_update_arg = DeclareLaunchArgument(
        'rgbd_angular_update',
        default_value='0.30',
        description='Radians the robot must rotate before RTAB-Map creates a new keyframe',
    )
    grid_range_max_arg = DeclareLaunchArgument(
        'grid_range_max',
        default_value='4.0',
        description='Maximum depth/range used for RTAB-Map occupancy mapping in meters',
    )
    rtabmap_feature_model_arg = DeclareLaunchArgument(
        'rtabmap_feature_model',
        default_value='1',
        description=(
            'RTAB-Map visual feature model id '
            '(1=SIFT, 2=ORB, 8=GFTT/ORB, 11=SuperPoint when available)'
        ),
    )
    rtabmap_incremental_memory_arg = DeclareLaunchArgument(
        'rtabmap_incremental_memory',
        default_value='true',
        description='true=SLAM mapping mode, false=localization mode on existing map',
    )
    rtabmap_init_wm_with_all_nodes_arg = DeclareLaunchArgument(
        'rtabmap_init_wm_with_all_nodes',
        default_value='true',
        description='Load saved database nodes into working memory on startup',
    )
    start_segmenter_arg = DeclareLaunchArgument(
        'start_segmenter',
        default_value='true',
        description='Start the K12 RGB-D semantic segmenter that publishes /semantic/label_image',
    )
    start_cloud_generator_arg = DeclareLaunchArgument(
        'start_cloud_generator',
        default_value='true',
        description='Start semantic_cloud_generator to publish /semantic/points',
    )
    start_gazebo_landmarks_arg = DeclareLaunchArgument(
        'start_gazebo_landmarks',
        default_value='true',
        description='Publish named Gazebo objects as semantic landmarks for object goals',
    )
    start_goal_resolver_arg = DeclareLaunchArgument(
        'start_goal_resolver',
        default_value='true',
        description='Resolve /semantic/goal_command text into /goal_pose',
    )
    semantic_approach_distance_arg = DeclareLaunchArgument(
        'semantic_approach_distance',
        default_value='1.35',
        description='Meters to stop away from a semantic target object',
    )
    start_rtabmap_arg = DeclareLaunchArgument(
        'start_rtabmap',
        default_value='true',
        description='Start RTAB-Map RGB-D SLAM using only RGB, depth, and odometry',
    )
    launch_rtabmap_viz_arg = DeclareLaunchArgument(
        'launch_rtabmap_viz',
        default_value='true',
        description='Start rtabmap_viz in addition to RViz',
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Open RViz2 with the K12 semantic navigation view',
    )
    rtabmap_database_path_arg = DeclareLaunchArgument(
        'rtabmap_database_path',
        default_value=PathJoinSubstitution([
            EnvironmentVariable('HOME'),
            '.ros',
            'k12_semantic_rtabmap.db',
        ]),
        description='RTAB-Map database path',
    )
    reset_rtabmap_db_arg = DeclareLaunchArgument(
        'reset_rtabmap_db',
        default_value='false',
        description='Delete the RTAB-Map database on startup for a fresh mapping session',
    )

    config_file = PathJoinSubstitution([
        FindPackageShare('semantic_navigation'),
        'config',
        'rtabmap_rgbd_semantic.yaml',
    ])

    segmenter = Node(
        package='semantic_object_detection',
        executable='rgbd_semantic_segmenter',
        name='rgbd_semantic_segmenter',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
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
            {'publish_semantic_cloud': False},
            {'publish_local_costmap': False},
        ],
        condition=IfCondition(LaunchConfiguration('start_segmenter')),
    )

    semantic_cloud_generator = Node(
        package='semantic_navigation',
        executable='semantic_cloud_generator',
        name='semantic_cloud_generator',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'rgb_topic': LaunchConfiguration('rgb_topic')},
            {'depth_topic': LaunchConfiguration('depth_topic')},
            {'semantic_topic': LaunchConfiguration('semantic_topic')},
            {'camera_info_topic': LaunchConfiguration('camera_info_topic')},
            {'output_frame': LaunchConfiguration('navigation_frame')},
            {'accumulation_frame': LaunchConfiguration('odom_frame')},
            {'map_frame': LaunchConfiguration('map_frame')},
            {
                'point_decimation': ParameterValue(
                    LaunchConfiguration('point_decimation'),
                    value_type=int,
                )
            },
        ],
        condition=IfCondition(LaunchConfiguration('start_cloud_generator')),
    )

    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'approx_sync': True},
            {
                'approx_sync_max_interval': ParameterValue(
                    LaunchConfiguration('rgbd_sync_slop'),
                    value_type=float,
                )
            },
            {'sync_queue_size': 40},
            {'qos_image': 2},
            {'qos_camera_info': 2},
        ],
        remappings=[
            ('rgb/image', LaunchConfiguration('rgb_topic')),
            ('depth/image', LaunchConfiguration('depth_topic')),
            ('rgb/camera_info', LaunchConfiguration('camera_info_topic')),
            ('rgbd_image', LaunchConfiguration('rtabmap_rgbd_topic')),
        ],
        condition=IfCondition(LaunchConfiguration('start_rgbd_sync')),
    )

    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'frame_id': LaunchConfiguration('navigation_frame')},
            {'odom_frame_id': LaunchConfiguration('odom_frame')},
            {'publish_tf': False},
            {'subscribe_rgbd': True},
            {
                'wait_imu_to_init': ParameterValue(
                    LaunchConfiguration('rgbd_odom_wait_imu_to_init'),
                    value_type=bool,
                )
            },
            {'guess_frame_id': LaunchConfiguration('odom_frame')},
            {'guess_min_translation': 0.01},
            {'guess_min_rotation': 0.01},
            {'approx_sync': True},
            {
                'approx_sync_max_interval': ParameterValue(
                    LaunchConfiguration('rgbd_sync_slop'),
                    value_type=float,
                )
            },
            {'sync_queue_size': 40},
            {'qos_image': 2},
            {'qos_camera_info': 2},
            {'qos_imu': 1},
            {'Odom/AlignWithGround': 'false'},
            {'Reg/Force3DoF': 'true'},
        ],
        remappings=[
            ('rgbd_image', LaunchConfiguration('rtabmap_rgbd_topic')),
            ('imu', '/imu/data'),
            ('odom', LaunchConfiguration('rtabmap_odom_topic')),
        ],
        condition=IfCondition(LaunchConfiguration('start_rgbd_odometry')),
    )

    gazebo_semantic_landmarks = Node(
        package='semantic_navigation',
        executable='gazebo_semantic_landmarks',
        name='gazebo_semantic_landmarks',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'frame_id': LaunchConfiguration('map_frame')},
        ],
        condition=IfCondition(LaunchConfiguration('start_gazebo_landmarks')),
    )

    semantic_goal_resolver = Node(
        package='semantic_navigation',
        executable='semantic_goal_resolver',
        name='semantic_goal_resolver',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'goal_frame_id': LaunchConfiguration('map_frame')},
            {'robot_frame_id': LaunchConfiguration('navigation_frame')},
            {
                'approach_distance_m': ParameterValue(
                    LaunchConfiguration('semantic_approach_distance'),
                    value_type=float,
                )
            },
        ],
        condition=IfCondition(LaunchConfiguration('start_goal_resolver')),
    )

    reset_rtabmap_db = ExecuteProcess(
        cmd=['rm', '-f', LaunchConfiguration('rtabmap_database_path')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('reset_rtabmap_db')),
    )

    cmd_vel_limiter = Node(
        package='semantic_navigation',
        executable='cmd_vel_limiter',
        name='cmd_vel_limiter',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'input_cmd_topic': LaunchConfiguration('cmd_vel_input_topic')},
            {'output_cmd_topic': LaunchConfiguration('cmd_vel_output_topic')},
            {
                'max_linear_x': ParameterValue(
                    LaunchConfiguration('cmd_vel_max_linear'),
                    value_type=float,
                )
            },
            {
                'max_angular_z': ParameterValue(
                    LaunchConfiguration('cmd_vel_max_angular'),
                    value_type=float,
                )
            },
            {
                'max_linear_accel': ParameterValue(
                    LaunchConfiguration('cmd_vel_linear_accel'),
                    value_type=float,
                )
            },
            {
                'max_angular_accel': ParameterValue(
                    LaunchConfiguration('cmd_vel_angular_accel'),
                    value_type=float,
                )
            },
            {
                'turn_linear_scale': ParameterValue(
                    LaunchConfiguration('cmd_vel_turn_linear_scale'),
                    value_type=float,
                )
            },
        ],
        condition=IfCondition(LaunchConfiguration('start_cmd_vel_limiter')),
    )

    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'frame_id': LaunchConfiguration('navigation_frame')},
            {'map_frame_id': LaunchConfiguration('map_frame')},
            {'odom_frame_id': LaunchConfiguration('odom_frame')},
            {'database_path': LaunchConfiguration('rtabmap_database_path')},
            {'subscribe_rgb': False},
            {'subscribe_depth': False},
            {'subscribe_rgbd': True},
            {
                'RGBD/LinearUpdate': ParameterValue(
                    LaunchConfiguration('rgbd_linear_update'),
                    value_type=str,
                )
            },
            {
                'RGBD/AngularUpdate': ParameterValue(
                    LaunchConfiguration('rgbd_angular_update'),
                    value_type=str,
                )
            },
            {
                'Grid/RangeMax': ParameterValue(
                    LaunchConfiguration('grid_range_max'),
                    value_type=str,
                )
            },
            {
                'Mem/IncrementalMemory': ParameterValue(
                    LaunchConfiguration('rtabmap_incremental_memory'),
                    value_type=str,
                )
            },
            {
                'Mem/InitWMWithAllNodes': ParameterValue(
                    LaunchConfiguration('rtabmap_init_wm_with_all_nodes'),
                    value_type=str,
                )
            },
            {
                'Vis/FeatureType': ParameterValue(
                    LaunchConfiguration('rtabmap_feature_model'),
                    value_type=str,
                )
            },
            {
                'Kp/DetectorStrategy': ParameterValue(
                    LaunchConfiguration('rtabmap_feature_model'),
                    value_type=str,
                )
            },
        ],
        remappings=[
            ('rgbd_image', LaunchConfiguration('rtabmap_rgbd_topic')),
            ('odom', LaunchConfiguration('rtabmap_odom_topic')),
            ('map', 'rtabmap/map'),
            ('map_updates', 'rtabmap/map_updates'),
        ],
        condition=IfCondition(LaunchConfiguration('start_rtabmap')),
    )

    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'frame_id': LaunchConfiguration('navigation_frame')},
            {'map_frame_id': LaunchConfiguration('map_frame')},
            {'odom_frame_id': LaunchConfiguration('odom_frame')},
            {'subscribe_rgb': False},
            {'subscribe_depth': False},
            {'subscribe_rgbd': True},
        ],
        remappings=[
            ('rgbd_image', LaunchConfiguration('rtabmap_rgbd_topic')),
            ('odom', LaunchConfiguration('rtabmap_odom_topic')),
            ('map', 'rtabmap/map'),
            ('map_updates', 'rtabmap/map_updates'),
        ],
        condition=IfCondition(LaunchConfiguration('launch_rtabmap_viz')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            PathJoinSubstitution([
                FindPackageShare('k12_description'),
                'rviz',
                'k12_robot.rviz',
            ]),
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
    )

    return LaunchDescription([
        use_sim_time_arg,
        rgb_topic_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        start_rgbd_sync_arg,
        start_rgbd_odometry_arg,
        rgbd_odom_wait_imu_to_init_arg,
        rtabmap_rgbd_topic_arg,
        rtabmap_odom_topic_arg,
        rgbd_sync_slop_arg,
        semantic_topic_arg,
        navigation_frame_arg,
        map_frame_arg,
        odom_frame_arg,
        point_decimation_arg,
        process_every_n_frames_arg,
        start_cmd_vel_limiter_arg,
        cmd_vel_input_topic_arg,
        cmd_vel_output_topic_arg,
        cmd_vel_max_linear_arg,
        cmd_vel_max_angular_arg,
        cmd_vel_linear_accel_arg,
        cmd_vel_angular_accel_arg,
        cmd_vel_turn_linear_scale_arg,
        rgbd_linear_update_arg,
        rgbd_angular_update_arg,
        grid_range_max_arg,
        rtabmap_feature_model_arg,
        rtabmap_incremental_memory_arg,
        rtabmap_init_wm_with_all_nodes_arg,
        start_segmenter_arg,
        start_cloud_generator_arg,
        start_gazebo_landmarks_arg,
        start_goal_resolver_arg,
        semantic_approach_distance_arg,
        start_rtabmap_arg,
        launch_rtabmap_viz_arg,
        launch_rviz_arg,
        rtabmap_database_path_arg,
        reset_rtabmap_db_arg,
        reset_rtabmap_db,
        segmenter,
        semantic_cloud_generator,
        rgbd_sync,
        rgbd_odometry,
        gazebo_semantic_landmarks,
        semantic_goal_resolver,
        cmd_vel_limiter,
        rtabmap,
        rtabmap_viz,
        rviz,
    ])
