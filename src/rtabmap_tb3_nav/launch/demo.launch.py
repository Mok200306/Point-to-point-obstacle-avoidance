#!/usr/bin/env python3
"""Bring up Gazebo, RTAB-Map RGB-D SLAM, Nav2 and optional GUIs."""

import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def apply_navigation_profile(params, profile):
    """Apply a named, reproducible experiment profile to loaded parameters."""
    if profile in ('current', 'fast_north_045_v2'):
        return

    if profile == 'fast_north_045_v3':
        # Keep the first detour and the central-barrier passage in one fixed
        # map-frame corridor.  v2 only constrained the west-barrier window;
        # Smac could then choose the top of barrier_center, producing the
        # occasional y~=1.9 m excursion.  The lower side of the central bar
        # is open at y~=0.75 m and still leaves the padded Waffle footprint
        # outside the hard obstacle boundary.  This is a benchmark-specific
        # route hint; v2 remains the frozen baseline and is unchanged.
        planner = params['planner_server']['ros__parameters']['GridBased']
        planner['side_bias_world_x_min'] = -7.2
        planner['side_bias_world_x_max'] = 3.45
        planner['side_bias_target_world_y_enabled'] = True
        planner['side_bias_reference_world_y'] = 0.0
        planner['side_bias_target_world_y'] = 0.75
        planner['side_bias_target_offset'] = 0.75
        planner['side_bias_target_max_cost'] = 100.0
        planner['side_bias_target_distance_scale'] = 0.60
        planner['side_bias_target_exponent'] = 2.0
        return

    if profile == 'fast_goalline_045_v1':
        # Keep the safe north detour around barrier_west, then smoothly return
        # toward the goal line below barrier_center.  Raise the corridor only
        # before barrier_east, and release the hint just after that obstacle so
        # the planner can converge to y=0 for the goal.  This is a bounded
        # benchmark route preference; lethal/footprint/collision constraints
        # remain unchanged.
        planner = params['planner_server']['ros__parameters']['GridBased']
        planner['side_bias_world_x_min'] = -7.2
        planner['side_bias_world_x_max'] = 7.35
        planner['side_bias_target_world_y_enabled'] = True
        planner['side_bias_reference_world_y'] = 0.0
        planner['side_bias_target_world_y'] = 0.75
        planner['side_bias_target_offset'] = 0.75
        planner['side_bias_target_max_cost'] = 100.0
        planner['side_bias_target_distance_scale'] = 0.60
        planner['side_bias_target_exponent'] = 2.0
        planner['side_bias_target_schedule_enabled'] = True
        planner['side_bias_target_schedule_x'] = [
            -7.2, -2.9, -2.35, 2.8, 3.35, 7.35]
        planner['side_bias_target_schedule_y'] = [
            0.75, 0.75, 0.30, 0.30, 0.50, 0.50]
        return

    if profile == 'fast_goalline_045_v2':
        # Keep v1's bounded target-line corridor, but make the inflation cost
        # decay faster so the planner does not leave the corridor merely to
        # escape the outer soft-cost band.  The hard footprint, lethal cells,
        # collision monitor and route schedule remain unchanged.
        controller = params['controller_server']['ros__parameters']['FollowPath']
        controller['desired_linear_vel'] = 0.28
        controller['inflation_cost_scaling_factor'] = 4.5

        smoother = params['velocity_smoother']['ros__parameters']
        smoother['max_velocity'] = [0.28, 0.0, 0.90]

        planner = params['planner_server']['ros__parameters']['GridBased']
        planner['side_bias_world_x_min'] = -7.2
        planner['side_bias_world_x_max'] = 7.35
        planner['side_bias_target_world_y_enabled'] = True
        planner['side_bias_reference_world_y'] = 0.0
        planner['side_bias_target_world_y'] = 0.75
        planner['side_bias_target_offset'] = 0.75
        planner['side_bias_target_max_cost'] = 140.0
        planner['side_bias_target_distance_scale'] = 0.50
        planner['side_bias_target_exponent'] = 2.0
        planner['side_bias_target_schedule_enabled'] = True
        planner['side_bias_target_schedule_x'] = [
            -7.2, -2.9, -2.35, 2.8, 3.35, 7.35]
        planner['side_bias_target_schedule_y'] = [
            0.75, 0.75, 0.30, 0.30, 0.50, 0.50]

        local_inflation = params['local_costmap']['local_costmap'][
            'ros__parameters']['inflation_layer']
        global_inflation = params['global_costmap']['global_costmap'][
            'ros__parameters']['inflation_layer']
        local_inflation['cost_scaling_factor'] = 4.5
        global_inflation['cost_scaling_factor'] = 4.5
        return

    if profile == 'fast_goalline_045_v3':
        # Keep v2's speed and cost profile, but use a longer RPP carrot.  The
        # larger lookahead reduces the sharp steering impulse at barrier_west
        # while the planner and collision monitor still enforce feasibility.
        controller = params['controller_server']['ros__parameters']['FollowPath']
        controller['desired_linear_vel'] = 0.28
        controller['lookahead_dist'] = 0.85
        controller['min_lookahead_dist'] = 0.70
        controller['max_lookahead_dist'] = 1.30
        controller['lookahead_time'] = 1.8
        controller['inflation_cost_scaling_factor'] = 4.5

        smoother = params['velocity_smoother']['ros__parameters']
        smoother['max_velocity'] = [0.28, 0.0, 0.90]

        planner = params['planner_server']['ros__parameters']['GridBased']
        planner['side_bias_world_x_min'] = -7.2
        planner['side_bias_world_x_max'] = 7.35
        planner['side_bias_target_world_y_enabled'] = True
        planner['side_bias_reference_world_y'] = 0.0
        planner['side_bias_target_world_y'] = 0.75
        planner['side_bias_target_offset'] = 0.75
        planner['side_bias_target_max_cost'] = 140.0
        planner['side_bias_target_distance_scale'] = 0.50
        planner['side_bias_target_exponent'] = 2.0
        planner['side_bias_target_schedule_enabled'] = True
        planner['side_bias_target_schedule_x'] = [
            -7.2, -2.9, -2.35, 2.8, 3.35, 7.35]
        planner['side_bias_target_schedule_y'] = [
            0.75, 0.75, 0.30, 0.30, 0.50, 0.50]

        local_inflation = params['local_costmap']['local_costmap'][
            'ros__parameters']['inflation_layer']
        global_inflation = params['global_costmap']['global_costmap'][
            'ros__parameters']['inflation_layer']
        local_inflation['cost_scaling_factor'] = 4.5
        global_inflation['cost_scaling_factor'] = 4.5
        return

    if profile == 'fast_goalline_045_v4':
        # Center the first detour in the physical opening between barrier_west
        # and crate_west_north.  v2/v3 preferred y=0.75 m, which is feasible
        # but lies close to the inflated top edge of barrier_west.  This route
        # keeps a higher, centered west corridor, descends before the central
        # barrier, stays above barrier_east, and returns to the goal line only
        # after the last obstacle.  The schedule is a soft preference; lethal
        # cells and the real footprint remain authoritative.
        controller = params['controller_server']['ros__parameters']['FollowPath']
        controller['desired_linear_vel'] = 0.30
        controller['lookahead_dist'] = 0.80
        controller['min_lookahead_dist'] = 0.62
        controller['max_lookahead_dist'] = 1.20
        controller['lookahead_time'] = 1.7
        controller['inflation_cost_scaling_factor'] = 4.5

        smoother = params['velocity_smoother']['ros__parameters']
        smoother['max_velocity'] = [0.30, 0.0, 0.90]

        planner = params['planner_server']['ros__parameters']['GridBased']
        planner['side_bias_world_x_min'] = -7.2
        planner['side_bias_world_x_max'] = 7.9
        planner['side_bias_target_world_y_enabled'] = True
        planner['side_bias_reference_world_y'] = 0.0
        planner['side_bias_target_world_y'] = 0.60
        planner['side_bias_target_offset'] = 0.60
        planner['side_bias_target_max_cost'] = 200.0
        planner['side_bias_target_distance_scale'] = 0.45
        planner['side_bias_target_exponent'] = 2.0
        planner['side_bias_target_schedule_enabled'] = True
        planner['side_bias_target_schedule_x'] = [
            -7.2, -3.4, -2.6, -2.25, 2.75, 3.20, 3.50, 7.40, 7.90]
        planner['side_bias_target_schedule_y'] = [
            0.95, 0.95, 0.75, 0.60, 0.60, 0.68, 0.58, 0.58, 0.00]

        local_inflation = params['local_costmap']['local_costmap'][
            'ros__parameters']['inflation_layer']
        global_inflation = params['global_costmap']['global_costmap'][
            'ros__parameters']['inflation_layer']
        local_inflation['cost_scaling_factor'] = 4.5
        global_inflation['cost_scaling_factor'] = 4.5
        return

    if profile == 'adaptive_goal_line_045':
        # General-purpose profile: retain the current start-to-goal line
        # preference, but remove every benchmark-specific world-coordinate
        # side hint. Smac must choose the detour from the live costmap for
        # each new goal instead of replaying the large-world A->B corridor.
        planner = params['planner_server']['ros__parameters']['GridBased']
        planner['line_bias_enabled'] = True
        planner['side_bias_enabled'] = False
        planner['side_bias_apply_to_unknown'] = False
        planner['side_bias_target_world_y_enabled'] = False
        planner['side_bias_target_schedule_enabled'] = False

        controller = params['controller_server']['ros__parameters']['FollowPath']
        controller['desired_linear_vel'] = 0.28
        controller['lookahead_dist'] = 0.80
        controller['min_lookahead_dist'] = 0.62
        controller['max_lookahead_dist'] = 1.20
        controller['lookahead_time'] = 1.7
        controller['inflation_cost_scaling_factor'] = 4.5

        smoother = params['velocity_smoother']['ros__parameters']
        smoother['max_velocity'] = [0.28, 0.0, 0.90]

        local_inflation = params['local_costmap']['local_costmap'][
            'ros__parameters']['inflation_layer']
        global_inflation = params['global_costmap']['global_costmap'][
            'ros__parameters']['inflation_layer']
        local_inflation['cost_scaling_factor'] = 4.5
        global_inflation['cost_scaling_factor'] = 4.5
        return

    if profile == 'fast_north_045_v1':
        planner = params['planner_server']['ros__parameters']['GridBased']
        planner['side_bias_world_x_min'] = -8.2
        planner['side_bias_target_world_y_enabled'] = False
        planner['side_bias_reference_world_y'] = 0.0
        planner['side_bias_target_world_y'] = 0.0
        planner['side_bias_target_offset'] = 0.95
        planner['side_bias_target_max_cost'] = 180.0
        planner['side_bias_target_distance_scale'] = 0.50
        return

    if profile in ('frozen_goal_line_045_v1', 'goal_line_quad_045_v1'):
        controller = params['controller_server']['ros__parameters']['FollowPath']
        controller['desired_linear_vel'] = 0.22
        controller['lookahead_dist'] = 0.70
        controller['min_lookahead_dist'] = 0.52
        controller['max_lookahead_dist'] = 1.10
        controller['min_approach_linear_velocity'] = 0.06

        planner = params['planner_server']['ros__parameters']['GridBased']
        planner['side_bias_enabled'] = False
        planner['side_bias_apply_to_unknown'] = False
        planner['side_bias_target_world_y_enabled'] = False

        smoother = params['velocity_smoother']['ros__parameters']
        smoother['max_velocity'] = [0.22, 0.0, 0.75]
        smoother['max_accel'] = [0.8, 0.0, 2.0]
        smoother['max_decel'] = [-1.0, 0.0, -2.0]
        return

    raise RuntimeError(
        'Unknown navigation_profile={!r}; use current, fast_north_045_v1, '
        'fast_north_045_v2, fast_north_045_v3, fast_goalline_045_v1, '
        'fast_goalline_045_v2, fast_goalline_045_v3, fast_goalline_045_v4, '
        'adaptive_goal_line_045, or frozen_goal_line_045_v1.'.format(profile))


def nav2_params_for_mode(source_file, package_share, online, profile):
    """Select the online padded map or saved-map localization configuration."""
    with open(source_file, 'r', encoding='utf-8') as stream:
        params = yaml.safe_load(stream)

    apply_navigation_profile(params, profile)

    global_params = params['global_costmap']['global_costmap']['ros__parameters']
    # A fixed-size copy of RTAB-Map's growing map allows online planning to
    # retain already-observed obstacles without StaticLayer resizing the
    # global costmap on every map expansion. Localization uses the original
    # saved map topic.
    global_params['rolling_window'] = False
    global_params['static_layer']['map_topic'] = '/nav_map' if online else '/map'
    global_params['static_layer']['subscribe_to_updates'] = False

    bt_path = os.path.join(
        package_share, 'behavior_trees', 'navigate_to_pose_stable_replanning.xml')
    through_poses_bt_path = os.path.join(
        package_share, 'behavior_trees', 'navigate_through_poses_stable_replanning.xml')
    bt_params = params['bt_navigator']['ros__parameters']
    bt_params['default_nav_to_pose_bt_xml'] = bt_path
    bt_params['default_nav_through_poses_bt_xml'] = through_poses_bt_path

    rewritten = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', prefix='rtabmap_nav2_online_', delete=False)
    yaml.safe_dump(params, rewritten, sort_keys=False)
    rewritten.close()
    return rewritten.name


def collision_monitor_params_for_profile(source_file, profile):
    """Create a temporary collision-monitor YAML for a named profile."""
    with open(source_file, 'r', encoding='utf-8') as stream:
        params = yaml.safe_load(stream)

    if profile in (
        'current', 'fast_north_045_v1', 'fast_north_045_v2', 'fast_north_045_v3',
        'fast_goalline_045_v1', 'fast_goalline_045_v2', 'fast_goalline_045_v3',
        'fast_goalline_045_v4', 'adaptive_goal_line_045'):
        pass
    elif profile in ('frozen_goal_line_045_v1', 'goal_line_quad_045_v1'):
        params['collision_monitor']['ros__parameters']['PolygonSlow'][
            'slowdown_ratio'] = 0.65
    else:
        raise RuntimeError(
            'Unknown navigation_profile={!r}; use current, fast_north_045_v1, '
            'fast_north_045_v2, fast_north_045_v3, fast_goalline_045_v1, '
            'fast_goalline_045_v2, fast_goalline_045_v3, fast_goalline_045_v4, '
            'adaptive_goal_line_045, or frozen_goal_line_045_v1.'.format(profile))

    rewritten = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', prefix='rtabmap_collision_', delete=False)
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
    navigation_profile = LaunchConfiguration('navigation_profile').perform(context)

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

    # RTAB-Map starts with a small map and grows it as the robot moves. Online
    # mode uses a fixed-size padded copy of that map; localization mode uses
    # the saved RTAB-Map map through StaticLayer.
    nav2_params_file = nav2_params_for_mode(
        nav2_params, package_share, online, navigation_profile)
    collision_monitor_params_file = collision_monitor_params_for_profile(
        collision_monitor_params, navigation_profile)

    online_map_padder = []
    if online:
        online_map_padder = [Node(
            package='rtabmap_tb3_nav',
            executable='map_padder.py',
            name='online_map_padder',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'source_topic': '/map',
                'output_topic': '/nav_map',
                'width': 480,
                'height': 340,
                'resolution': 0.05,
                'origin_x': -12.0,
                'origin_y': -8.5,
            }],
        )]
        online_map_padder = [TimerAction(period=4.0, actions=online_map_padder)]

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
            'params_file': collision_monitor_params_file,
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
        Node(
            package='rtabmap_tb3_nav',
            executable='goal_line_visualizer.py',
            name='goal_line_visualizer',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
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

    delayed_collision_monitor = TimerAction(
        period=2.0, actions=[collision_monitor])

    return [
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle'),
        *gazebo,
        *rtabmap_nodes,
        *online_map_padder,
        nav2,
        delayed_collision_monitor,
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
            description='Nav2 parameter file. Online mode uses fixed /nav_map through StaticLayer.'),
        DeclareLaunchArgument(
            'collision_monitor_params',
            default_value=os.path.join(
                get_package_share_directory('rtabmap_tb3_nav'),
                'config',
                'collision_monitor_rgbd_params.yaml'),
            description='Collision monitor parameter file.'),
        DeclareLaunchArgument(
            'navigation_profile',
            default_value='adaptive_goal_line_045',
            description=(
                'Reproducible parameter profile: adaptive_goal_line_045 is the '
                'current generic-goal default; '
                'fast_north_045_v3 is the historical fixed-corridor baseline; '
                'fast_north_045_v2 restores the previous fixed-west-window baseline; '
                'fast_goalline_045_v1 tests a segmented return-to-goal corridor; '
                'fast_goalline_045_v2 tests faster inflation decay and speed; '
                'fast_goalline_045_v3 tests a longer RPP lookahead; '
                'fast_goalline_045_v4 tests a centered detour and 0.30 m/s; '
                'adaptive_goal_line_045 disables benchmark world-coordinate '
                'side hints and replans each goal from the live costmap; '
                'frozen_goal_line_045_v1 restores the pre-optimization run-03 baseline.')),
        OpaqueFunction(function=launch_setup),
    ])
