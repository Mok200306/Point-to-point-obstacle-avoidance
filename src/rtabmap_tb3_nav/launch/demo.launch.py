#!/usr/bin/env python3
"""Bring up Gazebo, RTAB-Map RGB-D SLAM, Nav2 and optional GUIs."""

import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def online_nav2_params(source_file):
    """Create an online-only Nav2 file without the map-resizing StaticLayer."""
    with open(source_file, 'r', encoding='utf-8') as stream:
        params = yaml.safe_load(stream)

    global_params = params['global_costmap']['global_costmap']['ros__parameters']
    global_params['plugins'] = [
        plugin for plugin in global_params['plugins'] if plugin != 'static_layer'
    ]
    global_params.pop('static_layer', None)

    rewritten = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', prefix='rtabmap_nav2_online_', delete=False)
    yaml.safe_dump(params, rewritten, sort_keys=False)
    rewritten.close()
    return rewritten.name


def launch_setup(context, *args, **kwargs):
    package_share = get_package_share_directory('rtabmap_tb3_nav')
    gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    nav2_share = get_package_share_directory('nav2_bringup')
    collision_monitor_share = get_package_share_directory('nav2_collision_monitor')

    world_name = LaunchConfiguration('world').perform(context)
    nav2_params = LaunchConfiguration('nav2_params').perform(context)
    collision_monitor_params = LaunchConfiguration(
        'collision_monitor_params').perform(context)

    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    online = LaunchConfiguration('online').perform(context).lower() == 'true'
    reset_db = LaunchConfiguration('reset_db').perform(context).lower() == 'true'

    if not online and LaunchConfiguration('localization').perform(context).lower() != 'true':
        raise RuntimeError(
            'online:=false disables incremental mapping; use localization:=true '
            'with an existing RTAB-Map database.')

    if world_name in ('obstacle_course', 'obstacle_course_large'):
        world_file = 'indoor_obstacle_course_large.world' if world_name == 'obstacle_course_large' else 'indoor_obstacle_course.world'
        custom_world = os.path.join(package_share, 'worlds', world_file)
        gazebo = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')),
                launch_arguments={'world': custom_world}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')),
                condition=IfCondition(LaunchConfiguration('gazebo_gui')),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_share, 'launch', 'robot_state_publisher.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_share, 'launch', 'spawn_turtlebot3.launch.py')),
                launch_arguments={
                    'x_pose': LaunchConfiguration('x_pose'),
                    'y_pose': LaunchConfiguration('y_pose'),
                }.items(),
            ),
        ]
    else:
        world_launch = os.path.join(gazebo_share, 'launch', f'turtlebot3_{world_name}.launch.py')
        gazebo = [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(world_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'x_pose': LaunchConfiguration('x_pose'),
                'y_pose': LaunchConfiguration('y_pose'),
            }.items(),
        )]

    # RTAB-Map starts with a small map and grows it as the robot moves. In
    # online mode remove Nav2's StaticLayer entirely: in Humble, setting its
    # enabled parameter false still lets it subscribe and resize on map growth.
    # The fixed rolling obstacle costmap keeps unknown cells traversable for
    # NavFn while the RGB-D obstacle layer updates the local route in real time.
    # Localization mode keeps the saved map and the StaticLayer.
    nav2_params_file = nav2_params
    if online:
        nav2_params_file = online_nav2_params(nav2_params)

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'true',
        }.items(),
    )

    # The Nav2 velocity smoother publishes /cmd_vel. The collision monitor
    # consumes that command and the patched Gazebo model listens on
    # /cmd_vel_safe, so a stale depth observation can only fail closed.
    collision_monitor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            collision_monitor_share, 'launch', 'collision_monitor_node.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': collision_monitor_params,
        }.items(),
    )

    rtabmap_parameters = {
        'frame_id': 'base_footprint',
        'use_sim_time': use_sim_time,
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan': False,
        'subscribe_scan_cloud': False,
        'use_action_for_goal': False,
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
        # Keep mapping responsive while Nav2 is already driving. These are
        # deliberately modest thresholds for the simulated D435-style stream.
        rtabmap_parameters.update({
            'Mem/IncrementalMemory': 'true',
            'RGBD/LinearUpdate': '0.05',
            'RGBD/AngularUpdate': '0.05',
        })
    rtabmap_db_parameters = {
        **rtabmap_parameters,
        'database_path': os.path.expanduser('~/.ros/rtabmap.db'),
    }
    rgbd_remappings = [
        ('rgb/image', '/camera/image_raw'),
        ('rgb/camera_info', '/camera/camera_info'),
        ('depth/image', '/camera/depth/image_raw'),
    ]

    if LaunchConfiguration('localization').perform(context).lower() == 'true':
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

    rtabmap_nodes = [
        rtabmap_node,
        Node(
            package='rtabmap_util',
            executable='point_cloud_xyz',
            name='camera_cloud',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                # Keep the costmap cloud small enough for a responsive
                # controller; collision_monitor also consumes this low-latency
                # cloud rather than the full raw camera stream.
                'decimation': 4,
                'max_depth': 3.5,
                'voxel_size': 0.04,
            }],
            remappings=[
                ('depth/image', '/camera/depth/image_raw'),
                ('depth/camera_info', '/camera/camera_info'),
                ('cloud', '/camera/cloud'),
            ],
        ),
        Node(
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
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[rtabmap_parameters],
            remappings=rgbd_remappings,
        ),
    ]

    rviz = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return [
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle'),
        *gazebo,
        *rtabmap_nodes,
        nav2,
        collision_monitor,
        rviz,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='obstacle_course_large',
            choices=['obstacle_course_large', 'obstacle_course', 'house', 'world'],
            description='Gazebo world. obstacle_course_large is the online RGB-D obstacle course.'),
        DeclareLaunchArgument(
            'x_pose',
            default_value='-8.5',
            description='Initial robot X pose in Gazebo. The large-course A point is -8.5, 0.0.'),
        DeclareLaunchArgument(
            'y_pose',
            default_value='0.0',
            description='Initial robot Y pose in Gazebo. The large-course A point is -8.5, 0.0.'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the Gazebo clock.'),
        DeclareLaunchArgument(
            'localization',
            default_value='false',
            description='Use the existing RTAB-Map database instead of mapping.'),
        DeclareLaunchArgument(
            'online',
            default_value='true',
            description='Map and navigate at the same time. Set false only for localization mode.'),
        DeclareLaunchArgument(
            'reset_db',
            default_value='true',
            description='Delete the RTAB-Map database at mapping startup.'),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start Nav2 RViz.'),
        DeclareLaunchArgument(
            'rtabmap_viz',
            default_value='false',
            description='Start the RTAB-Map visualization window.'),
        DeclareLaunchArgument(
            'gazebo_gui',
            default_value='true',
            description='Start the Gazebo GUI for custom obstacle-course worlds.'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                get_package_share_directory('rtabmap_tb3_nav'),
                'config',
                'indoor_nav.rviz'),
            description='RViz configuration file.'),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=os.path.join(
                get_package_share_directory('rtabmap_tb3_nav'),
                'config',
                'nav2_rgbd_params.yaml'),
            description='Nav2 parameter file. Online mode removes StaticLayer from a temporary copy.'),
        DeclareLaunchArgument(
            'collision_monitor_params',
            default_value=os.path.join(
                get_package_share_directory('rtabmap_tb3_nav'),
                'config',
                'collision_monitor_rgbd_params.yaml'),
            description='Collision monitor parameter file.'),
        OpaqueFunction(function=launch_setup),
    ])
