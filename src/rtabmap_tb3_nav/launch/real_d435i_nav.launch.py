#!/usr/bin/env python3
"""Run RGB-D RTAB-Map and Nav2 with a real Intel RealSense D435i.

The robot base driver must provide odom -> base_link and consume the safe
velocity command on /cmd_vel_safe. This launch intentionally does not create
fake wheel odometry or a robot model.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    package_share = get_package_share_directory('rtabmap_tb3_nav')
    realsense_share = get_package_share_directory('realsense2_camera')
    nav2_share = get_package_share_directory('nav2_bringup')
    collision_monitor_share = get_package_share_directory('nav2_collision_monitor')

    nav2_params = os.path.join(package_share, 'config', 'nav2_rgbd_params.yaml')
    collision_monitor_params = os.path.join(
        package_share, 'config', 'collision_monitor_rgbd_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    online = LaunchConfiguration('online').perform(context).lower() == 'true'
    localization = LaunchConfiguration('localization').perform(context).lower() == 'true'
    reset_db = LaunchConfiguration('reset_db').perform(context).lower() == 'true'
    database_path = os.path.expanduser(
        LaunchConfiguration('database_path').perform(context))

    if not online and not localization:
        raise RuntimeError(
            'online:=false disables incremental mapping; use localization:=true '
            'with an existing RTAB-Map database.')

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            realsense_share, 'launch', 'rs_launch.py')),
        launch_arguments={
            'camera_name': LaunchConfiguration('camera_name'),
            'camera_namespace': LaunchConfiguration('camera_namespace'),
            'serial_no': LaunchConfiguration('camera_serial'),
            'config_file': LaunchConfiguration('camera_config'),
            'base_frame_id': LaunchConfiguration('camera_frame'),
            'enable_sync': 'true',
            'align_depth.enable': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',
            'pointcloud.enable': 'false',
        }.items(),
    )

    # This is the only transform supplied by this launch. The RealSense node
    # publishes camera_link -> sensor/optical frames; the robot driver must
    # publish odom -> base_link.
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='d435i_mount_tf',
        output='screen',
        arguments=[
            '--x', LaunchConfiguration('camera_x'),
            '--y', LaunchConfiguration('camera_y'),
            '--z', LaunchConfiguration('camera_z'),
            '--roll', LaunchConfiguration('camera_roll'),
            '--pitch', LaunchConfiguration('camera_pitch'),
            '--yaw', LaunchConfiguration('camera_yaw'),
            '--frame-id', LaunchConfiguration('base_frame'),
            '--child-frame-id', LaunchConfiguration('camera_frame'),
        ],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            nav2_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'autostart': 'true',
        }.items(),
    )

    collision_monitor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            collision_monitor_share, 'launch', 'collision_monitor_node.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': collision_monitor_params,
        }.items(),
    )

    # D435i publishes color/* and aligned_depth_to_color/* topics. Approximate
    # synchronization is important because the two USB streams are not locked
    # to identical ROS timestamps on every host.
    rtabmap_parameters = {
        'frame_id': LaunchConfiguration('rtabmap_base_frame'),
        'use_sim_time': use_sim_time,
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan': False,
        'subscribe_scan_cloud': False,
        'use_action_for_goal': False,
        'approx_sync': True,
        'sync_queue_size': 30,
        'Reg/Force3DoF': 'true',
        'Grid/RayTracing': 'true',
        'Grid/3D': 'false',
        'Grid/RangeMax': '4',
        'Grid/NormalsSegmentation': 'false',
        'Grid/MaxGroundHeight': '0.05',
        'Grid/MaxObstacleHeight': '1.5',
        'Optimizer/GravitySigma': '0',
    }
    if online:
        rtabmap_parameters.update({
            'Mem/IncrementalMemory': 'true',
            'RGBD/LinearUpdate': '0.05',
            'RGBD/AngularUpdate': '0.05',
        })

    rtabmap_db_parameters = {
        **rtabmap_parameters,
        'database_path': database_path,
    }
    rgbd_remappings = [
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
        ('depth/camera_info', '/camera/aligned_depth_to_color/camera_info'),
    ]

    if localization:
        rtabmap_node = Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[
                rtabmap_db_parameters,
                {'Mem/IncrementalMemory': 'False', 'Mem/InitWMWithAllNodes': 'True'},
            ],
            remappings=rgbd_remappings,
        )
    else:
        rtabmap_node = Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[rtabmap_db_parameters],
            remappings=rgbd_remappings,
            arguments=['-d'] if reset_db else [],
        )

    camera_cloud = Node(
        package='rtabmap_util',
        executable='point_cloud_xyz',
        name='camera_cloud',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'decimation': 4,
            'max_depth': 3.5,
            'voxel_size': 0.04,
        }],
        remappings=[
            ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
            ('depth/camera_info', '/camera/aligned_depth_to_color/camera_info'),
            ('cloud', '/camera/cloud'),
        ],
    )

    camera_obstacles = Node(
        package='rtabmap_util',
        executable='obstacles_detection',
        name='camera_obstacles',
        output='screen',
        parameters=[rtabmap_parameters],
        remappings=[
            ('cloud', '/camera/cloud'),
            ('obstacles', '/camera/obstacles'),
            ('ground', '/camera/ground'),
        ],
    )

    rtabmap_viz = Node(
        condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[rtabmap_parameters],
        remappings=rgbd_remappings,
    )

    rviz = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return [
        realsense,
        camera_tf,
        rtabmap_node,
        camera_cloud,
        camera_obstacles,
        rtabmap_viz,
        nav2,
        collision_monitor,
        rviz,
    ]


def generate_launch_description():
    package_share = get_package_share_directory('rtabmap_tb3_nav')
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_name', default_value='camera',
            description='RealSense camera node name.'),
        DeclareLaunchArgument(
            'camera_namespace', default_value='camera',
            description='RealSense camera namespace.'),
        DeclareLaunchArgument(
            'camera_serial', default_value='',
            description='D435i serial number; empty selects the first device.'),
        DeclareLaunchArgument(
            'camera_config',
            default_value=os.path.join(package_share, 'config', 'real_d435i_camera.yaml'),
            description='RealSense parameter YAML.'),
        DeclareLaunchArgument(
            'base_frame', default_value='base_link',
            description='Robot frame that owns the fixed camera mount transform.'),
        DeclareLaunchArgument(
            'camera_frame', default_value='camera_link',
            description='Root frame published by realsense2_camera.'),
        DeclareLaunchArgument(
            'rtabmap_base_frame', default_value='base_footprint',
            description='Robot frame used by RTAB-Map.'),
        DeclareLaunchArgument('camera_x', default_value='0.18'),
        DeclareLaunchArgument('camera_y', default_value='0.0'),
        DeclareLaunchArgument('camera_z', default_value='0.28'),
        DeclareLaunchArgument('camera_roll', default_value='0.0'),
        DeclareLaunchArgument(
            'camera_pitch', default_value='0.0',
            description='Downward camera pitch in radians; measure the real mount.'),
        DeclareLaunchArgument('camera_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use false for a physical D435i and real robot.'),
        DeclareLaunchArgument(
            'online', default_value='true',
            description='Build the map while navigating.'),
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Use an existing RTAB-Map database.'),
        DeclareLaunchArgument(
            'reset_db', default_value='false',
            description='Delete the database at mapping startup; opt in explicitly.'),
        DeclareLaunchArgument(
            'database_path', default_value='~/.ros/rtabmap_d435i.db',
            description='RTAB-Map database for the physical D435i; kept separate from the simulation database.'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(package_share, 'config', 'indoor_nav.rviz')),
        OpaqueFunction(function=launch_setup),
    ])
