#!/usr/bin/env python3
"""Run RGB-D RTAB-Map and Nav2 with a real Intel RealSense D435i.

The robot base driver must provide odom -> base_link and consume the safe
velocity command on /cmd_vel_safe. This launch intentionally does not create
fake wheel odometry or a robot model.
"""

import os
import subprocess
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


V13_PROFILE = 'adaptive_goal_line_050_recovery_v13_line_tiebreaker'
GENERIC_PROFILE = 'adaptive_goal_line_045'


def _set_all_use_sim_time(value, enabled):
    """Rewrite every nested Nav2 ``use_sim_time`` value in-place."""
    if isinstance(value, dict):
        for key, child in value.items():
            if key == 'use_sim_time':
                value[key] = bool(enabled)
            else:
                _set_all_use_sim_time(child, enabled)
    elif isinstance(value, list):
        for child in value:
            _set_all_use_sim_time(child, enabled)


def _set_robot_frames(value, robot_base_frame):
    """Use the physical chassis frame everywhere Nav2 names a base frame."""
    if isinstance(value, dict):
        for key, child in value.items():
            if key == 'robot_base_frame':
                value[key] = robot_base_frame
            else:
                _set_robot_frames(child, robot_base_frame)
    elif isinstance(value, list):
        for child in value:
            _set_robot_frames(child, robot_base_frame)


def _apply_profile(params, profile, package_share):
    """Apply the frozen four-point profile or the generic comparison profile."""
    planner = params['planner_server']['ros__parameters']['GridBased']
    controller = params['controller_server']['ros__parameters']['FollowPath']
    smoother = params['velocity_smoother']['ros__parameters']
    local_inflation = params['local_costmap']['local_costmap'][
        'ros__parameters']['inflation_layer']
    global_inflation = params['global_costmap']['global_costmap'][
        'ros__parameters']['inflation_layer']

    if profile == V13_PROFILE:
        # v13 is a generic current-start/current-goal tie-breaker. It does not
        # carry the large-world fixed side schedule from the old v4 benchmark.
        planner['line_bias_enabled'] = True
        planner['line_bias_max_cost'] = 18.0
        planner['line_bias_distance_scale'] = 2.8
        planner['line_bias_exponent'] = 2.0
        planner['line_bias_apply_to_unknown'] = False
        planner['goal_progress_bias_enabled'] = False
        planner['goal_progress_bias_max_cost'] = 0.0
        planner['goal_progress_bias_distance_scale'] = 1.0
        planner['goal_progress_bias_exponent'] = 2.0
        planner['goal_progress_bias_apply_to_unknown'] = False
        planner['unknown_bias_enabled'] = False
        planner['unknown_bias_cost'] = 0.0
        planner['side_bias_enabled'] = False
        planner['side_bias_apply_to_unknown'] = False
        planner['side_bias_target_world_y_enabled'] = False
        planner['side_bias_target_schedule_enabled'] = False
        planner['cost_travel_multiplier'] = 6.0

        controller['desired_linear_vel'] = 0.28
        controller['lookahead_dist'] = 0.68
        controller['min_lookahead_dist'] = 0.52
        controller['max_lookahead_dist'] = 1.05
        controller['lookahead_time'] = 1.5
        controller['inflation_cost_scaling_factor'] = 4.5
        controller['use_rotate_to_heading'] = True
        controller['allow_reversing'] = False
        controller['cost_scaling_dist'] = 0.65
        controller['regulated_linear_scaling_min_radius'] = 0.80
        controller['max_allowed_time_to_collision_up_to_carrot'] = 1.5
        params['controller_server']['ros__parameters']['progress_checker'][
            'movement_time_allowance'] = 30.0

        smoother['max_velocity'] = [0.28, 0.0, 0.90]
        local_inflation['cost_scaling_factor'] = 4.5
        global_inflation['cost_scaling_factor'] = 4.5
        local_inflation['inflation_radius'] = 0.50
        global_inflation['inflation_radius'] = 0.50
        bt_name = 'navigate_to_pose_periodic_replanning_6s.xml'
        through_poses_bt_name = 'navigate_through_poses_periodic_replanning_6s.xml'
    elif profile == GENERIC_PROFILE:
        # Optional comparison only. No world-coordinate route hint is allowed.
        planner['line_bias_enabled'] = True
        planner['side_bias_enabled'] = False
        planner['side_bias_apply_to_unknown'] = False
        planner['side_bias_target_world_y_enabled'] = False
        planner['side_bias_target_schedule_enabled'] = False
        controller['desired_linear_vel'] = 0.28
        controller['lookahead_dist'] = 0.80
        controller['min_lookahead_dist'] = 0.62
        controller['max_lookahead_dist'] = 1.20
        controller['lookahead_time'] = 1.7
        controller['inflation_cost_scaling_factor'] = 4.5
        smoother['max_velocity'] = [0.28, 0.0, 0.90]
        local_inflation['cost_scaling_factor'] = 4.5
        global_inflation['cost_scaling_factor'] = 4.5
        bt_name = 'navigate_to_pose_stable_replanning.xml'
        through_poses_bt_name = 'navigate_through_poses_stable_replanning.xml'
    else:
        raise RuntimeError(
            f'Unsupported real navigation_profile={profile!r}; use '
            f'{V13_PROFILE!r} or {GENERIC_PROFILE!r}.')

    bt_params = params['bt_navigator']['ros__parameters']
    bt_params['default_nav_to_pose_bt_xml'] = os.path.join(
        package_share, 'behavior_trees', bt_name)
    bt_params['default_nav_through_poses_bt_xml'] = os.path.join(
        package_share, 'behavior_trees', through_poses_bt_name)


def _apply_real_limits(params, robot_base_frame, footprint, max_linear,
                       max_angular, linear_accel, linear_decel,
                       angular_accel, angular_decel):
    """Apply conservative first-boot limits to the physical chassis config."""
    _set_all_use_sim_time(params, False)
    _set_robot_frames(params, robot_base_frame)

    for costmap_name in ('local_costmap', 'global_costmap'):
        costmap = params[costmap_name][costmap_name]['ros__parameters']
        costmap['footprint'] = footprint

    controller = params['controller_server']['ros__parameters']['FollowPath']
    controller['desired_linear_vel'] = max_linear
    controller['rotate_to_heading_angular_vel'] = min(
        controller.get('rotate_to_heading_angular_vel', max_angular),
        max_angular)
    controller['max_angular_accel'] = angular_accel

    behavior = params['behavior_server']['ros__parameters']
    behavior['max_rotational_vel'] = max_angular
    behavior['min_rotational_vel'] = min(0.20, max_angular)
    behavior['rotational_acc_lim'] = angular_accel

    smoother = params['velocity_smoother']['ros__parameters']
    smoother['max_velocity'] = [max_linear, 0.0, max_angular]
    smoother['min_velocity'] = [-max_linear, 0.0, -max_angular]
    smoother['max_accel'] = [linear_accel, 0.0, angular_accel]
    smoother['max_decel'] = [-linear_decel, 0.0, -angular_decel]


def _write_yaml_temp(params, prefix):
    temporary = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', prefix=prefix, delete=False,
        encoding='utf-8')
    yaml.safe_dump(params, temporary, allow_unicode=True, sort_keys=False)
    temporary.close()
    return temporary.name


def _read_yaml(path):
    """Read a YAML file and normalize an empty document to an empty mapping."""
    with open(path, 'r', encoding='utf-8') as stream:
        return yaml.safe_load(stream) or {}


def _git_commit():
    try:
        return subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], stderr=subprocess.DEVNULL,
            text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        return 'unknown'


def launch_setup(context, *args, **kwargs):
    package_share = get_package_share_directory('rtabmap_tb3_nav')
    realsense_share = get_package_share_directory('realsense2_camera')
    nav2_share = get_package_share_directory('nav2_bringup')

    nav2_params = LaunchConfiguration('nav2_params').perform(context)
    collision_monitor_params = LaunchConfiguration(
        'collision_monitor_params').perform(context)

    requested_sim_time = LaunchConfiguration(
        'use_sim_time').perform(context).strip().lower()
    if requested_sim_time not in ('false', '0'):
        raise RuntimeError(
            'real_d435i_nav.launch.py requires use_sim_time:=false; '
            'refusing to start a physical robot with a Gazebo clock.')

    use_sim_time = 'false'
    online = LaunchConfiguration('online').perform(context).lower() == 'true'
    localization = LaunchConfiguration('localization').perform(context).lower() == 'true'
    reset_db = LaunchConfiguration('reset_db').perform(context).lower() == 'true'
    map_padder_enabled = LaunchConfiguration(
        'map_padder').perform(context).lower() == 'true'
    database_path = os.path.expanduser(
        LaunchConfiguration('database_path').perform(context))
    navigation_profile = LaunchConfiguration(
        'navigation_profile').perform(context)
    camera_config = LaunchConfiguration('camera_config').perform(context)
    camera_frame = LaunchConfiguration('camera_frame').perform(context)
    base_frame = LaunchConfiguration('base_frame').perform(context)
    rtabmap_base_frame = LaunchConfiguration(
        'rtabmap_base_frame').perform(context)
    collision_base_frame = LaunchConfiguration(
        'collision_base_frame').perform(context)
    odom_frame = LaunchConfiguration('odom_frame').perform(context)
    map_frame = LaunchConfiguration('map_frame').perform(context)
    odom_topic = LaunchConfiguration('odom_topic').perform(context)
    map_topic = LaunchConfiguration('map_topic').perform(context)
    navigation_map_topic = LaunchConfiguration(
        'navigation_map_topic').perform(context)
    footprint = LaunchConfiguration('robot_footprint').perform(context)
    max_linear = float(LaunchConfiguration(
        'max_linear_velocity').perform(context))
    max_angular = float(LaunchConfiguration(
        'max_angular_velocity').perform(context))
    linear_accel = float(LaunchConfiguration(
        'linear_acceleration').perform(context))
    linear_decel = float(LaunchConfiguration(
        'linear_deceleration').perform(context))
    angular_accel = float(LaunchConfiguration(
        'angular_acceleration').perform(context))
    angular_decel = float(LaunchConfiguration(
        'angular_deceleration').perform(context))
    stop_distance = float(LaunchConfiguration(
        'collision_stop_distance_m').perform(context))
    slow_distance = float(LaunchConfiguration(
        'collision_slow_distance_m').perform(context))
    collision_half_width = float(LaunchConfiguration(
        'collision_half_width_m').perform(context))
    slowdown_ratio = float(LaunchConfiguration(
        'collision_slowdown_ratio').perform(context))
    cmd_vel_in_topic = LaunchConfiguration(
        'cmd_vel_in_topic').perform(context)
    cmd_vel_safe_topic = LaunchConfiguration(
        'cmd_vel_safe_topic').perform(context)
    color_image_topic = LaunchConfiguration(
        'color_image_topic').perform(context)
    color_info_topic = LaunchConfiguration(
        'color_info_topic').perform(context)
    depth_image_topic = LaunchConfiguration(
        'depth_image_topic').perform(context)
    depth_info_topic = LaunchConfiguration(
        'depth_info_topic').perform(context)
    imu_topic = LaunchConfiguration('imu_topic').perform(context)
    cloud_topic = LaunchConfiguration('cloud_topic').perform(context)
    obstacles_topic = LaunchConfiguration(
        'obstacles_topic').perform(context)
    ground_topic = LaunchConfiguration('ground_topic').perform(context)
    use_water_bridge = LaunchConfiguration(
        'use_water_bridge').perform(context).lower() == 'true'
    water_sdk_root = LaunchConfiguration('water_sdk_root').perform(context)
    water_config_path = LaunchConfiguration(
        'water_config_path').perform(context)
    water_robot_host = LaunchConfiguration(
        'water_robot_host').perform(context)
    water_robot_port = int(LaunchConfiguration(
        'water_robot_port').perform(context))
    water_gateway_port = int(LaunchConfiguration(
        'water_gateway_port').perform(context))
    water_connect_timeout = float(LaunchConfiguration(
        'water_connect_timeout_s').perform(context))
    water_command_rate = float(LaunchConfiguration(
        'water_command_rate_hz').perform(context))
    water_state_rate = float(LaunchConfiguration(
        'water_state_rate_hz').perform(context))
    water_odom_rate = float(LaunchConfiguration(
        'water_odom_rate_hz').perform(context))
    water_command_timeout = float(LaunchConfiguration(
        'water_command_timeout_s').perform(context))
    water_state_timeout = float(LaunchConfiguration(
        'water_state_timeout_s').perform(context))
    water_auto_start_gateway = LaunchConfiguration(
        'water_auto_start_gateway').perform(context).lower() == 'true'
    water_enable_motion = LaunchConfiguration(
        'water_enable_motion').perform(context).lower() == 'true'
    water_allow_provisional_odom = LaunchConfiguration(
        'water_allow_provisional_odom').perform(context).lower() == 'true'

    if not odom_frame or not map_frame:
        raise RuntimeError('odom_frame and map_frame must be non-empty')
    for topic in (odom_topic, map_topic, navigation_map_topic):
        if not topic.startswith('/'):
            raise RuntimeError(f'frame-related topic must be absolute: {topic!r}')
    if not online and not localization:
        raise RuntimeError(
            'online:=false disables incremental mapping; use localization:=true '
            'with an existing RTAB-Map database.')
    if max_linear <= 0.0 or max_angular <= 0.0:
        raise RuntimeError('real velocity limits must be positive')
    if slow_distance <= stop_distance or collision_half_width <= 0.0:
        raise RuntimeError(
            'collision_slow_distance_m must exceed collision_stop_distance_m '
            'and collision_half_width_m must be positive')
    if not 0.0 < slowdown_ratio <= 1.0:
        raise RuntimeError('collision_slowdown_ratio must be in (0, 1]')
    if not cmd_vel_in_topic.startswith('/') or not cmd_vel_safe_topic.startswith('/'):
        raise RuntimeError('cmd_vel topics must be absolute ROS topic names')
    if cmd_vel_in_topic == cmd_vel_safe_topic:
        raise RuntimeError('cmd_vel_in_topic and cmd_vel_safe_topic must differ')
    for topic in (
            color_image_topic, color_info_topic, depth_image_topic,
            depth_info_topic, imu_topic, cloud_topic, obstacles_topic,
            ground_topic):
        if not topic.startswith('/'):
            raise RuntimeError(f'camera topic must be absolute: {topic!r}')

    camera_params = _read_yaml(camera_config) if camera_config.strip() else {}
    camera_params['base_frame_id'] = camera_frame
    effective_camera_config_file = _write_yaml_temp(
        camera_params, 'rtabmap_real_camera_')

    nav_params = _read_yaml(nav2_params)
    _apply_profile(nav_params, navigation_profile, package_share)
    _apply_real_limits(
        nav_params, base_frame, footprint, max_linear, max_angular,
        linear_accel, linear_decel, angular_accel, angular_decel)
    global_params = nav_params['global_costmap']['global_costmap'][
        'ros__parameters']
    global_params['static_layer']['map_topic'] = (
        navigation_map_topic if online and map_padder_enabled else map_topic)
    global_params['static_layer']['subscribe_to_updates'] = False
    if online and map_padder_enabled:
        global_params['width'] = int(LaunchConfiguration(
            'map_width').perform(context))
        global_params['height'] = int(LaunchConfiguration(
            'map_height').perform(context))
        global_params['resolution'] = float(LaunchConfiguration(
            'map_resolution').perform(context))
        global_params['origin_x'] = float(LaunchConfiguration(
            'map_origin_x').perform(context))
        global_params['origin_y'] = float(LaunchConfiguration(
            'map_origin_y').perform(context))
    local_params = nav_params['local_costmap']['local_costmap'][
        'ros__parameters']
    nav_params['bt_navigator']['ros__parameters']['global_frame'] = map_frame
    nav_params['bt_navigator']['ros__parameters']['odom_topic'] = odom_topic
    nav_params['behavior_server']['ros__parameters']['global_frame'] = odom_frame
    nav_params['velocity_smoother']['ros__parameters']['odom_topic'] = odom_topic
    nav_params['local_costmap']['local_costmap'][
        'ros__parameters']['global_frame'] = odom_frame
    nav_params['global_costmap']['global_costmap'][
        'ros__parameters']['global_frame'] = map_frame
    nav_params['bt_navigator']['ros__parameters']['robot_base_frame'] = base_frame
    nav_params['behavior_server']['ros__parameters']['robot_base_frame'] = base_frame
    nav_params['local_costmap']['local_costmap'][
        'ros__parameters']['robot_base_frame'] = base_frame
    nav_params['global_costmap']['global_costmap'][
        'ros__parameters']['robot_base_frame'] = base_frame
    local_params['voxel_layer']['ground']['topic'] = ground_topic
    local_params['voxel_layer']['obstacles']['topic'] = obstacles_topic
    global_params['obstacle_layer']['obstacles']['topic'] = obstacles_topic
    nav2_params_file = _write_yaml_temp(nav_params, 'rtabmap_real_nav2_')

    collision_params = _read_yaml(collision_monitor_params)
    _set_all_use_sim_time(collision_params, False)
    collision_ros = collision_params['collision_monitor']['ros__parameters']
    collision_ros['base_frame_id'] = collision_base_frame
    collision_ros['odom_frame_id'] = odom_frame
    collision_ros['cmd_vel_in_topic'] = cmd_vel_in_topic
    collision_ros['cmd_vel_out_topic'] = cmd_vel_safe_topic
    collision_ros['obstacles']['topic'] = cloud_topic
    collision_ros['PolygonStop']['points'] = [
        stop_distance, collision_half_width,
        stop_distance, -collision_half_width,
        0.05, -collision_half_width,
        0.05, collision_half_width,
    ]
    collision_ros['PolygonSlow']['points'] = [
        slow_distance, collision_half_width,
        slow_distance, -collision_half_width,
        0.05, -collision_half_width,
        0.05, collision_half_width,
    ]
    collision_ros['PolygonSlow']['slowdown_ratio'] = slowdown_ratio
    collision_monitor_params_file = _write_yaml_temp(
        collision_params, 'rtabmap_real_collision_')

    runtime_snapshot_dir = LaunchConfiguration(
        'runtime_snapshot_dir').perform(context).strip()
    if runtime_snapshot_dir:
        runtime_snapshot_dir = os.path.abspath(runtime_snapshot_dir)
        os.makedirs(runtime_snapshot_dir, exist_ok=True)
        with open(os.path.join(runtime_snapshot_dir, '导航参数.yaml'), 'w',
                  encoding='utf-8') as stream:
            yaml.safe_dump(nav_params, stream, allow_unicode=True,
                           sort_keys=False)
        with open(os.path.join(runtime_snapshot_dir, '碰撞监视参数.yaml'), 'w',
                  encoding='utf-8') as stream:
            yaml.safe_dump(collision_params, stream, allow_unicode=True,
                           sort_keys=False)
        with open(os.path.join(runtime_snapshot_dir, '相机参数.yaml'), 'w',
                  encoding='utf-8') as stream:
            yaml.safe_dump(camera_params, stream, allow_unicode=True,
                           sort_keys=False)
        metadata = {
            'real_robot_mode': True,
            'platform': 'WATER II-S',
            'navigation_profile': navigation_profile,
            'git_commit': _git_commit(),
            'use_sim_time': False,
            'online': online,
            'localization': localization,
            'reset_db': reset_db,
            'database_path': database_path,
            'map_frame': map_frame,
            'odom_frame': odom_frame,
            'map_topic': map_topic,
            'navigation_map_topic': navigation_map_topic,
            'odom_topic': odom_topic,
            'base_frame': base_frame,
            'rtabmap_base_frame': rtabmap_base_frame,
            'collision_base_frame': collision_base_frame,
            'robot_footprint': footprint,
            'max_linear_velocity_mps': max_linear,
            'max_angular_velocity_radps': max_angular,
            'linear_acceleration_mps2': linear_accel,
            'linear_deceleration_mps2': linear_decel,
            'angular_acceleration_radps2': angular_accel,
            'angular_deceleration_radps2': angular_decel,
            'collision_stop_distance_m': stop_distance,
            'collision_slow_distance_m': slow_distance,
            'collision_half_width_m': collision_half_width,
            'collision_slowdown_ratio': slowdown_ratio,
            'base_driver_external': True,
            'base_driver_required_topics': [odom_topic, '/tf', cmd_vel_safe_topic],
            'water_bridge_enabled': use_water_bridge,
            'water_bridge_motion_enabled': (
                use_water_bridge and water_enable_motion),
            'water_bridge_odom_source': (
                'sdk_velocity_integrated_provisional'
                if use_water_bridge and water_allow_provisional_odom
                else 'external_vendor_driver'),
            'water_sdk_root': water_sdk_root,
            'water_robot_host': water_robot_host,
            'water_robot_port': water_robot_port,
            'water_gateway_port': water_gateway_port,
            'cmd_vel_in_topic': cmd_vel_in_topic,
            'cmd_vel_safe_topic': cmd_vel_safe_topic,
            'color_image_topic': color_image_topic,
            'color_info_topic': color_info_topic,
            'depth_image_topic': depth_image_topic,
            'depth_info_topic': depth_info_topic,
            'cloud_topic': cloud_topic,
            'obstacles_topic': obstacles_topic,
            'ground_topic': ground_topic,
            'imu_topic': imu_topic,
            'camera_config_source': camera_config,
            'camera_config_effective': effective_camera_config_file,
            'hardware_estop_checked_by_launch': False,
        }
        with open(os.path.join(runtime_snapshot_dir, '运行时元数据.yaml'), 'w',
                  encoding='utf-8') as stream:
            yaml.safe_dump(metadata, stream, allow_unicode=True, sort_keys=False)

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            realsense_share, 'launch', 'rs_launch.py')),
        launch_arguments={
            'camera_name': LaunchConfiguration('camera_name'),
            'camera_namespace': LaunchConfiguration('camera_namespace'),
            'serial_no': LaunchConfiguration('camera_serial'),
            'config_file': effective_camera_config_file,
            'base_frame_id': camera_frame,
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
            '--child-frame-id', camera_frame,
        ],
    )

    if online and map_padder_enabled:
        map_padder = [TimerAction(period=4.0, actions=[Node(
            package='rtabmap_tb3_nav',
            executable='map_padder.py',
            name='online_map_padder',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'source_topic': map_topic,
                'output_topic': navigation_map_topic,
                'width': int(LaunchConfiguration(
                    'map_width').perform(context)),
                'height': int(LaunchConfiguration(
                    'map_height').perform(context)),
                'resolution': float(LaunchConfiguration(
                    'map_resolution').perform(context)),
                'origin_x': float(LaunchConfiguration(
                    'map_origin_x').perform(context)),
                'origin_y': float(LaunchConfiguration(
                    'map_origin_y').perform(context)),
            }],
        )])]
    else:
        map_padder = []

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            nav2_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'true',
        }.items(),
    )

    collision_monitor_node = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        emulate_tty=True,
        parameters=[collision_monitor_params_file],
    )
    collision_monitor_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='collision_monitor_lifecycle_manager',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['collision_monitor'],
        }],
    )

    goal_line_visualizer = Node(
        package='rtabmap_tb3_nav',
        executable='goal_line_visualizer.py',
        name='goal_line_visualizer',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # Keep the lifecycle manager behind the node creation, matching the
    # simulation launch and avoiding an inactive safety output during startup.
    delayed_collision_monitor = TimerAction(
        period=4.0,
        actions=[
            collision_monitor_node,
            TimerAction(
                period=2.0,
                actions=[collision_monitor_lifecycle_manager]),
        ])

    water_bridge = Node(
        condition=IfCondition(LaunchConfiguration('use_water_bridge')),
        package='rtabmap_tb3_nav',
        executable='water_chassis_ros_bridge.py',
        name='water_chassis_ros_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'sdk_root': water_sdk_root,
            'config_path': water_config_path,
            'robot_host': water_robot_host,
            'robot_port': water_robot_port,
            'gateway_port': water_gateway_port,
            'connect_timeout_s': water_connect_timeout,
            'auto_start_gateway': water_auto_start_gateway,
            'enable_motion': water_enable_motion,
            'cmd_vel_topic': cmd_vel_safe_topic,
            'odom_topic': odom_topic,
            'odom_frame': odom_frame,
            'base_frame': base_frame,
            'command_rate_hz': water_command_rate,
            'state_rate_hz': water_state_rate,
            'odom_rate_hz': water_odom_rate,
            'command_timeout_s': water_command_timeout,
            'state_timeout_s': water_state_timeout,
            'max_linear_velocity': max_linear,
            'max_angular_velocity': max_angular,
            'publish_tf': True,
            'allow_provisional_odom': water_allow_provisional_odom,
            'status_topic': '/water_chassis/status',
        }],
    )

    # D435i publishes color/* and aligned_depth_to_color/* topics. Approximate
    # synchronization is important because the two USB streams are not locked
    # to identical ROS timestamps on every host.
    rtabmap_parameters = {
        'frame_id': rtabmap_base_frame,
        'map_frame_id': map_frame,
        'odom_frame_id': odom_frame,
        'publish_tf': True,
        'use_sim_time': False,
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan': False,
        'subscribe_scan_cloud': False,
        'use_action_for_goal': False,
        'approx_sync': True,
        'sync_queue_size': 30,
        # RealSense image topics are sensor-data/best-effort in the stock
        # driver. RTAB-Map's numeric QoS=2 selects best effort for image,
        # camera-info, odom and IMU subscriptions.
        'qos_image': 2,
        'qos_camera_info': 2,
        'qos_odom': 2,
        'qos_imu': 2,
        'Reg/Force3DoF': 'true',
        'Grid/RayTracing': 'true',
        'Grid/3D': 'false',
        'Grid/RangeMax': LaunchConfiguration(
            'camera_max_depth_m').perform(context),
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
        ('map', map_topic),
        ('rgb/image', color_image_topic),
        ('rgb/camera_info', color_info_topic),
        ('depth/image', depth_image_topic),
        ('depth/camera_info', depth_info_topic),
        ('odom', odom_topic),
        ('imu', imu_topic),
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
            'use_sim_time': False,
            'decimation': int(LaunchConfiguration(
                'cloud_decimation').perform(context)),
            'max_depth': float(LaunchConfiguration(
                'camera_max_depth_m').perform(context)),
            'voxel_size': float(LaunchConfiguration(
                'cloud_voxel_size_m').perform(context)),
        }],
        remappings=[
            ('depth/image', depth_image_topic),
            ('depth/camera_info', depth_info_topic),
            ('cloud', cloud_topic),
        ],
    )

    camera_obstacles = Node(
        package='rtabmap_util',
        executable='obstacles_detection',
        name='camera_obstacles',
        output='screen',
        parameters=[rtabmap_parameters],
        remappings=[
            ('cloud', cloud_topic),
            ('obstacles', obstacles_topic),
            ('ground', ground_topic),
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
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    return [
        water_bridge,
        realsense,
        camera_tf,
        rtabmap_node,
        camera_cloud,
        camera_obstacles,
        rtabmap_viz,
        goal_line_visualizer,
        nav2,
        *map_padder,
        delayed_collision_monitor,
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
            'rtabmap_base_frame', default_value='base_link',
            description='Robot frame used by RTAB-Map; normally the physical base frame.'),
        DeclareLaunchArgument(
            'collision_base_frame', default_value='base_link',
            description='Robot frame used by collision_monitor; normally the physical base frame.'),
        DeclareLaunchArgument(
            'map_frame', default_value='map',
            description='Global SLAM/navigation frame.'),
        DeclareLaunchArgument(
            'odom_frame', default_value='odom',
            description='Continuous local odometry frame from the chassis driver.'),
        DeclareLaunchArgument(
            'odom_topic', default_value='/odom',
            description='nav_msgs/Odometry topic from the chassis driver.'),
        DeclareLaunchArgument(
            'map_topic', default_value='/map',
            description='RTAB-Map OccupancyGrid topic.'),
        DeclareLaunchArgument(
            'navigation_map_topic', default_value='/nav_map',
            description='Fixed-size map topic consumed by online global costmap.'),
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
        DeclareLaunchArgument(
            'navigation_profile', default_value=V13_PROFILE,
            description=(
                'Real-robot profile. The default is the frozen four-point-loop '
                'v13; adaptive_goal_line_045 is retained only as a generic '
                'comparison profile.')),
        DeclareLaunchArgument(
            'robot_footprint',
            default_value='[[0.2525, 0.0], [0.1785, 0.1785], [0.0, 0.2525], [-0.1785, 0.1785], [-0.2525, 0.0], [-0.1785, -0.1785], [0.0, -0.2525], [0.1785, -0.1785]]',
            description='Provisional 0.505 m diameter footprint; replace after measuring the loaded chassis.'),
        DeclareLaunchArgument(
            'max_linear_velocity', default_value='0.12',
            description='Initial physical-platform linear speed cap in m/s.'),
        DeclareLaunchArgument(
            'max_angular_velocity', default_value='0.35',
            description='Initial physical-platform angular speed cap in rad/s.'),
        DeclareLaunchArgument(
            'linear_acceleration', default_value='0.15',
            description='Initial linear acceleration cap in m/s^2.'),
        DeclareLaunchArgument(
            'linear_deceleration', default_value='0.20',
            description='Initial linear deceleration cap in m/s^2.'),
        DeclareLaunchArgument(
            'angular_acceleration', default_value='0.50',
            description='Initial angular acceleration cap in rad/s^2.'),
        DeclareLaunchArgument(
            'angular_deceleration', default_value='0.70',
            description='Initial angular deceleration cap in rad/s^2.'),
        DeclareLaunchArgument(
            'collision_stop_distance_m', default_value='0.55',
            description='Provisional forward hard-stop polygon length in m.'),
        DeclareLaunchArgument(
            'collision_slow_distance_m', default_value='1.20',
            description='Provisional forward slowdown polygon length in m.'),
        DeclareLaunchArgument(
            'collision_half_width_m', default_value='0.36',
            description='Provisional safety polygon half-width in m.'),
        DeclareLaunchArgument(
            'collision_slowdown_ratio', default_value='0.50',
            description='Velocity ratio inside the provisional slowdown polygon.'),
        DeclareLaunchArgument(
            'cmd_vel_in_topic', default_value='/cmd_vel',
            description='Nav2 velocity input consumed by collision_monitor.'),
        DeclareLaunchArgument(
            'cmd_vel_safe_topic', default_value='/cmd_vel_safe',
            description='Collision-monitor output that the chassis driver must consume.'),
        DeclareLaunchArgument(
            'color_image_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument(
            'color_info_topic', default_value='/camera/color/camera_info'),
        DeclareLaunchArgument(
            'depth_image_topic',
            default_value='/camera/aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument(
            'depth_info_topic',
            default_value='/camera/aligned_depth_to_color/camera_info'),
        DeclareLaunchArgument(
            'imu_topic', default_value='/camera/imu',
            description='D435i combined IMU topic; the chassis odom remains authoritative for TF.'),
        DeclareLaunchArgument(
            'cloud_topic', default_value='/camera/cloud'),
        DeclareLaunchArgument(
            'obstacles_topic', default_value='/camera/obstacles'),
        DeclareLaunchArgument(
            'ground_topic', default_value='/camera/ground'),
        DeclareLaunchArgument(
            'use_water_bridge', default_value='false',
            description=(
                'Start the local WATER SDK ROS bridge. Set false only when a '
                'separate vendor ROS driver provides /odom, odom->base_link '
                'and subscribes to /cmd_vel_safe.')),
        DeclareLaunchArgument(
            'water_sdk_root',
            default_value=os.path.join(
                package_share, 'water_chassis_sdk_cn_v5_1'),
            description='Installed/source root containing water_chassis_sdk/.'),
        DeclareLaunchArgument(
            'water_config_path', default_value='',
            description='Optional WATER SDK config.json override.'),
        DeclareLaunchArgument(
            'water_robot_host', default_value='192.168.10.10',
            description='WATER chassis IP; confirm on site.'),
        DeclareLaunchArgument(
            'water_robot_port', default_value='31001',
            description='WATER vendor TCP port; confirm on site.'),
        DeclareLaunchArgument(
            'water_gateway_port', default_value='8080',
            description='Local SDK Gateway HTTP port.'),
        DeclareLaunchArgument(
            'water_connect_timeout_s', default_value='12.0'),
        DeclareLaunchArgument(
            'water_command_rate_hz', default_value='10.0',
            description='Safe velocity forwarding rate.'),
        DeclareLaunchArgument(
            'water_state_rate_hz', default_value='5.0',
            description='SDK status polling rate.'),
        DeclareLaunchArgument(
            'water_odom_rate_hz', default_value='20.0',
            description='Provisional integrated odom publication rate.'),
        DeclareLaunchArgument(
            'water_command_timeout_s', default_value='0.25',
            description='Stop if no fresh /cmd_vel_safe arrives.'),
        DeclareLaunchArgument(
            'water_state_timeout_s', default_value='1.2',
            description='Stop if SDK status becomes stale.'),
        DeclareLaunchArgument(
            'water_auto_start_gateway', default_value='true',
            description='Let the bridge own one local SDK Gateway process.'),
        DeclareLaunchArgument(
            'water_enable_motion', default_value='false',
            description=(
                'Explicitly allow the WATER bridge to forward /cmd_vel_safe. '
                'Keep false for diagnostics-only startup.')),
        DeclareLaunchArgument(
            'water_allow_provisional_odom', default_value='false',
            description=(
                'Explicit software-integration mode: integrate SDK reported '
                'velocity into a provisional /odom. Never use for final '
                'physical-robot results; real encoder odom remains required.')),
        DeclareLaunchArgument(
            'camera_max_depth_m', default_value='3.5',
            description='Maximum depth used for RTAB-Map and the obstacle cloud.'),
        DeclareLaunchArgument(
            'cloud_decimation', default_value='4',
            description='Depth decimation factor for the low-latency obstacle cloud.'),
        DeclareLaunchArgument(
            'cloud_voxel_size_m', default_value='0.04',
            description='Obstacle-cloud voxel size in m.'),
        DeclareLaunchArgument(
            'map_padder', default_value='true',
            description='Copy /map into a fixed /nav_map envelope for online global planning.'),
        DeclareLaunchArgument('map_width', default_value='800'),
        DeclareLaunchArgument('map_height', default_value='600'),
        DeclareLaunchArgument('map_resolution', default_value='0.05'),
        DeclareLaunchArgument('map_origin_x', default_value='-20.0'),
        DeclareLaunchArgument('map_origin_y', default_value='-15.0'),
        DeclareLaunchArgument(
            'runtime_snapshot_dir', default_value='',
            description='Optional directory for effective parameter and launch metadata snapshots.'),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=os.path.join(
                package_share, 'config', 'nav2_rgbd_params.yaml'),
            description='Base Nav2 YAML; profile and physical limits are rewritten at launch.'),
        DeclareLaunchArgument(
            'collision_monitor_params',
            default_value=os.path.join(
                package_share, 'config', 'collision_monitor_rgbd_params.yaml'),
            description='Base collision-monitor YAML; frame and safety polygons are rewritten at launch.'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(package_share, 'config', 'indoor_nav.rviz')),
        OpaqueFunction(function=launch_setup),
    ])
