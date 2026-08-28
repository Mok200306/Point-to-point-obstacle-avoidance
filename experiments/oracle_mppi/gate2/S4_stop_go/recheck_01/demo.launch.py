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

    if profile == 'reactive_mppi_static':
        # Gate 1 uses an independent MPPI YAML. Keep this profile generic even
        # when a caller accidentally points at the historical base YAML: the
        # live costmap and the current goal must remain the only route inputs.
        planner = params['planner_server']['ros__parameters']['GridBased']
        planner['line_bias_enabled'] = True
        planner['side_bias_enabled'] = False
        planner['side_bias_apply_to_unknown'] = False
        planner['side_bias_target_world_y_enabled'] = False
        planner['side_bias_target_schedule_enabled'] = False
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

    if profile == 'adaptive_goal_line_045_recovery_v1':
        # Generic online-planning profile with recovery-only changes. Keep
        # all benchmark/world-coordinate side hints disabled; the live map
        # and the current goal still determine every route.
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
        # Permit a short reverse maneuver when the live path requires the
        # robot to leave a locally blocked pocket before turning to the goal.
        controller['allow_reversing'] = True

        params['controller_server']['ros__parameters']['progress_checker'][
            'movement_time_allowance'] = 30.0

        smoother = params['velocity_smoother']['ros__parameters']
        smoother['max_velocity'] = [0.28, 0.0, 0.90]

        local_inflation = params['local_costmap']['local_costmap'][
            'ros__parameters']['inflation_layer']
        global_inflation = params['global_costmap']['global_costmap'][
            'ros__parameters']['inflation_layer']
        local_inflation['cost_scaling_factor'] = 4.5
        global_inflation['cost_scaling_factor'] = 4.5
        return

    if profile == 'adaptive_goal_line_045_recovery_v2':
        # Recovery v2 keeps the generic, live-costmap goal-line planner but
        # gives clearance a stronger priority in the cross-scene world.  The
        # 0.45 m inflation radius and robot footprint stay unchanged; only the
        # soft cost gradient and the planner's accumulated traversal cost are
        # strengthened so a route grazing a crate is less attractive.
        planner = params['planner_server']['ros__parameters']['GridBased']
        planner['line_bias_enabled'] = True
        planner['line_bias_max_cost'] = 30.0
        planner['line_bias_distance_scale'] = 2.5
        planner['side_bias_enabled'] = False
        planner['side_bias_apply_to_unknown'] = False
        planner['side_bias_target_world_y_enabled'] = False
        planner['side_bias_target_schedule_enabled'] = False
        planner['cost_travel_multiplier'] = 12.0

        controller = params['controller_server']['ros__parameters']['FollowPath']
        controller['desired_linear_vel'] = 0.26
        controller['lookahead_dist'] = 0.80
        controller['min_lookahead_dist'] = 0.62
        controller['max_lookahead_dist'] = 1.20
        controller['lookahead_time'] = 1.7
        controller['inflation_cost_scaling_factor'] = 1.8
        # RPP forbids reversing when rotate-to-heading is enabled.  Disable
        # that mutually exclusive mode so BackUp and any reverse cusp can be
        # used in a genuine dead-end recovery.
        controller['use_rotate_to_heading'] = False
        controller['allow_reversing'] = True

        params['controller_server']['ros__parameters']['progress_checker'][
            'movement_time_allowance'] = 30.0

        smoother = params['velocity_smoother']['ros__parameters']
        smoother['max_velocity'] = [0.26, 0.0, 0.90]

        local_inflation = params['local_costmap']['local_costmap'][
            'ros__parameters']['inflation_layer']
        global_inflation = params['global_costmap']['global_costmap'][
            'ros__parameters']['inflation_layer']
        # Keep inflation_radius=0.45 m; make the same radius decay more slowly
        # into cost so the planner prefers the middle of an opening.
        local_inflation['cost_scaling_factor'] = 1.8
        global_inflation['cost_scaling_factor'] = 1.8
        return

    if profile == 'adaptive_goal_line_045_recovery_v3':
        # Balanced candidate after v2 made narrow passages unavailable:
        # moderate clearance preference, reduced corner cutting, and the
        # genuinely reverse-capable RPP mode.
        planner = params['planner_server']['ros__parameters']['GridBased']
        planner['line_bias_enabled'] = True
        planner['line_bias_max_cost'] = 40.0
        planner['line_bias_distance_scale'] = 2.5
        planner['line_bias_exponent'] = 2.0
        planner['side_bias_enabled'] = False
        planner['side_bias_apply_to_unknown'] = False
        planner['side_bias_target_world_y_enabled'] = False
        planner['side_bias_target_schedule_enabled'] = False
        planner['cost_travel_multiplier'] = 8.0

        controller = params['controller_server']['ros__parameters']['FollowPath']
        controller['desired_linear_vel'] = 0.26
        controller['lookahead_dist'] = 0.70
        controller['min_lookahead_dist'] = 0.54
        controller['max_lookahead_dist'] = 1.10
        controller['lookahead_time'] = 1.5
        controller['inflation_cost_scaling_factor'] = 2.5
        controller['use_rotate_to_heading'] = False
        controller['allow_reversing'] = True

        params['controller_server']['ros__parameters']['progress_checker'][
            'movement_time_allowance'] = 30.0

        smoother = params['velocity_smoother']['ros__parameters']
        smoother['max_velocity'] = [0.26, 0.0, 0.90]

        local_inflation = params['local_costmap']['local_costmap'][
            'ros__parameters']['inflation_layer']
        global_inflation = params['global_costmap']['global_costmap'][
            'ros__parameters']['inflation_layer']
        # Keep inflation_radius=0.45 m; only the soft gradient changes.
        local_inflation['cost_scaling_factor'] = 2.5
        global_inflation['cost_scaling_factor'] = 2.5
        return

    if profile == 'adaptive_goal_line_050_recovery_v4':
        # Balanced safety candidate: keep RPP's proven forward/rotate mode
        # from v1, add only a small 0.05 m inflation margin, and reduce the
        # direct-line pull so the live costmap can select a wider detour.
        planner = params['planner_server']['ros__parameters']['GridBased']
        planner['line_bias_enabled'] = True
        planner['line_bias_max_cost'] = 25.0
        planner['line_bias_distance_scale'] = 2.5
        planner['line_bias_exponent'] = 2.0
        planner['side_bias_enabled'] = False
        planner['side_bias_apply_to_unknown'] = False
        planner['side_bias_target_world_y_enabled'] = False
        planner['side_bias_target_schedule_enabled'] = False
        planner['cost_travel_multiplier'] = 7.0

        controller = params['controller_server']['ros__parameters']['FollowPath']
        controller['desired_linear_vel'] = 0.28
        controller['lookahead_dist'] = 0.80
        controller['min_lookahead_dist'] = 0.62
        controller['max_lookahead_dist'] = 1.20
        controller['lookahead_time'] = 1.7
        controller['inflation_cost_scaling_factor'] = 4.5
        controller['use_rotate_to_heading'] = True
        controller['allow_reversing'] = False

        params['controller_server']['ros__parameters']['progress_checker'][
            'movement_time_allowance'] = 30.0

        smoother = params['velocity_smoother']['ros__parameters']
        smoother['max_velocity'] = [0.28, 0.0, 0.90]

        local_inflation = params['local_costmap']['local_costmap'][
            'ros__parameters']['inflation_layer']
        global_inflation = params['global_costmap']['global_costmap'][
            'ros__parameters']['inflation_layer']
        local_inflation['cost_scaling_factor'] = 4.5
        global_inflation['cost_scaling_factor'] = 4.5
        local_inflation['inflation_radius'] = 0.50
        global_inflation['inflation_radius'] = 0.50
        return

    if profile == 'adaptive_goal_line_050_recovery_v5':
        # Keep the generic online-planning topology that was successful in
        # recovery_v1.  v4 weakened the line preference too much and entered
        # the south dead-end in cross-scene-02.  This profile restores the
        # ordinary start-to-goal preference, while adding only a small
        # clearance margin and earlier controller collision projection to
        # prevent the return leg from cutting into a crate corner.
        planner = params['planner_server']['ros__parameters']['GridBased']
        planner['line_bias_enabled'] = True
        planner['line_bias_max_cost'] = 60.0
        planner['line_bias_distance_scale'] = 2.0
        planner['line_bias_exponent'] = 2.0
        planner['side_bias_enabled'] = False
        planner['side_bias_apply_to_unknown'] = False
        planner['side_bias_target_world_y_enabled'] = False
        planner['side_bias_target_schedule_enabled'] = False
        planner['cost_travel_multiplier'] = 6.0

        controller = params['controller_server']['ros__parameters']['FollowPath']
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

        smoother = params['velocity_smoother']['ros__parameters']
        smoother['max_velocity'] = [0.28, 0.0, 0.90]

        local_inflation = params['local_costmap']['local_costmap'][
            'ros__parameters']['inflation_layer']
        global_inflation = params['global_costmap']['global_costmap'][
            'ros__parameters']['inflation_layer']
        local_inflation['cost_scaling_factor'] = 4.5
        global_inflation['cost_scaling_factor'] = 4.5
        local_inflation['inflation_radius'] = 0.50
        global_inflation['inflation_radius'] = 0.50
        return

    if profile == 'adaptive_goal_line_050_recovery_v6_goal_directed':
        # Generic live-costmap profile for the cross-scene-02 dead-end case.
        # These terms are evaluated in the current start/goal frame on every
        # planning call. They do not encode obstacle coordinates or a recorded
        # route: short lateral/backward manoeuvres remain possible, while a
        # long branch through stale unknown space becomes unattractive.
        planner = params['planner_server']['ros__parameters']['GridBased']
        planner['line_bias_enabled'] = True
        planner['line_bias_max_cost'] = 80.0
        planner['line_bias_distance_scale'] = 1.8
        planner['line_bias_exponent'] = 2.0
        planner['line_bias_apply_to_unknown'] = True
        planner['goal_progress_bias_enabled'] = True
        planner['goal_progress_bias_max_cost'] = 90.0
        planner['goal_progress_bias_distance_scale'] = 1.0
        planner['goal_progress_bias_exponent'] = 2.0
        planner['goal_progress_bias_apply_to_unknown'] = True
        planner['unknown_bias_enabled'] = True
        # Unknown cells remain traversable, but a newly observed free cell is
        # preferred to a long detour through stale unknown space.
        planner['unknown_bias_cost'] = 120.0
        planner['side_bias_enabled'] = False
        planner['side_bias_apply_to_unknown'] = False
        planner['side_bias_target_world_y_enabled'] = False
        planner['side_bias_target_schedule_enabled'] = False
        planner['cost_travel_multiplier'] = 6.0

        controller = params['controller_server']['ros__parameters']['FollowPath']
        controller['desired_linear_vel'] = 0.28
        controller['lookahead_dist'] = 0.72
        controller['min_lookahead_dist'] = 0.54
        controller['max_lookahead_dist'] = 1.10
        controller['lookahead_time'] = 1.5
        controller['inflation_cost_scaling_factor'] = 4.5
        controller['use_rotate_to_heading'] = True
        controller['allow_reversing'] = False
        controller['cost_scaling_dist'] = 0.65
        controller['regulated_linear_scaling_min_radius'] = 0.80
        controller['max_allowed_time_to_collision_up_to_carrot'] = 1.25

        params['controller_server']['ros__parameters']['progress_checker'][
            'movement_time_allowance'] = 30.0

        smoother = params['velocity_smoother']['ros__parameters']
        smoother['max_velocity'] = [0.28, 0.0, 0.90]

        local_inflation = params['local_costmap']['local_costmap'][
            'ros__parameters']['inflation_layer']
        global_inflation = params['global_costmap']['global_costmap'][
            'ros__parameters']['inflation_layer']
        local_inflation['cost_scaling_factor'] = 4.5
        global_inflation['cost_scaling_factor'] = 4.5
        local_inflation['inflation_radius'] = 0.50
        global_inflation['inflation_radius'] = 0.50
        return

    if profile == 'adaptive_goal_line_050_recovery_v7_unknown_line':
        # v6 showed that making every unknown cell expensive can prefer a
        # known but very long branch.  v7 keeps unknown space traversable and
        # lets the current start-goal line distinguish the alternatives:
        # unknown cells close to the goal line stay affordable, while a long
        # off-line south-side detour becomes expensive.  No world-coordinate
        # obstacle hint or recorded route is used.
        planner = params['planner_server']['ros__parameters']['GridBased']
        planner['line_bias_enabled'] = True
        planner['line_bias_max_cost'] = 80.0
        planner['line_bias_distance_scale'] = 1.8
        planner['line_bias_exponent'] = 2.0
        planner['line_bias_apply_to_unknown'] = True
        planner['goal_progress_bias_enabled'] = True
        planner['goal_progress_bias_max_cost'] = 60.0
        planner['goal_progress_bias_distance_scale'] = 1.0
        planner['goal_progress_bias_exponent'] = 2.0
        planner['goal_progress_bias_apply_to_unknown'] = True
        planner['unknown_bias_enabled'] = True
        planner['unknown_bias_cost'] = 5.0
        planner['side_bias_enabled'] = False
        planner['side_bias_apply_to_unknown'] = False
        planner['side_bias_target_world_y_enabled'] = False
        planner['side_bias_target_schedule_enabled'] = False
        planner['cost_travel_multiplier'] = 6.0

        controller = params['controller_server']['ros__parameters']['FollowPath']
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

        smoother = params['velocity_smoother']['ros__parameters']
        smoother['max_velocity'] = [0.28, 0.0, 0.90]

        local_inflation = params['local_costmap']['local_costmap'][
            'ros__parameters']['inflation_layer']
        global_inflation = params['global_costmap']['global_costmap'][
            'ros__parameters']['inflation_layer']
        local_inflation['cost_scaling_factor'] = 4.5
        global_inflation['cost_scaling_factor'] = 4.5
        local_inflation['inflation_radius'] = 0.50
        global_inflation['inflation_radius'] = 0.50
        return

    if profile == 'adaptive_goal_line_050_recovery_v8_cross_track':
        # v7 applied the line preference to unknown cells, but its 80/1.8
        # quadratic cost saturated both candidate detours at 252.  v8 keeps
        # the same live unknown-space policy and widens the distance scale so
        # the planner can still distinguish a shorter north-side detour from
        # a much farther south-side dead-end.
        planner = params['planner_server']['ros__parameters']['GridBased']
        planner['line_bias_enabled'] = True
        planner['line_bias_max_cost'] = 70.0
        planner['line_bias_distance_scale'] = 3.0
        planner['line_bias_exponent'] = 2.0
        planner['line_bias_apply_to_unknown'] = True
        planner['goal_progress_bias_enabled'] = True
        planner['goal_progress_bias_max_cost'] = 60.0
        planner['goal_progress_bias_distance_scale'] = 1.0
        planner['goal_progress_bias_exponent'] = 2.0
        planner['goal_progress_bias_apply_to_unknown'] = True
        planner['unknown_bias_enabled'] = True
        planner['unknown_bias_cost'] = 5.0
        planner['side_bias_enabled'] = False
        planner['side_bias_apply_to_unknown'] = False
        planner['side_bias_target_world_y_enabled'] = False
        planner['side_bias_target_schedule_enabled'] = False
        planner['cost_travel_multiplier'] = 6.0

        controller = params['controller_server']['ros__parameters']['FollowPath']
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

        smoother = params['velocity_smoother']['ros__parameters']
        smoother['max_velocity'] = [0.28, 0.0, 0.90]

        local_inflation = params['local_costmap']['local_costmap'][
            'ros__parameters']['inflation_layer']
        global_inflation = params['global_costmap']['global_costmap'][
            'ros__parameters']['inflation_layer']
        local_inflation['cost_scaling_factor'] = 4.5
        global_inflation['cost_scaling_factor'] = 4.5
        local_inflation['inflation_radius'] = 0.50
        global_inflation['inflation_radius'] = 0.50
        return

    if profile == 'adaptive_goal_line_050_recovery_v9_continuous_replanning':
        # Generic online profile for the cross-scene-02 N->X dead-end case.
        # v5's successful 0.50 m / 0.28 m/s settings are retained.  The v9
        # change is deliberately limited to the BT selection below: Nav2
        # recomputes a path from the live costmap at 1 Hz instead of waiting
        # for the previous path to expire or become invalid.  No world-frame
        # obstacle coordinates, corridor schedule, or recorded route is used.
        planner = params['planner_server']['ros__parameters']['GridBased']
        planner['line_bias_enabled'] = True
        planner['line_bias_max_cost'] = 60.0
        planner['line_bias_distance_scale'] = 2.0
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

        controller = params['controller_server']['ros__parameters']['FollowPath']
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

        smoother = params['velocity_smoother']['ros__parameters']
        smoother['max_velocity'] = [0.28, 0.0, 0.90]

        local_inflation = params['local_costmap']['local_costmap'][
            'ros__parameters']['inflation_layer']
        global_inflation = params['global_costmap']['global_costmap'][
            'ros__parameters']['inflation_layer']
        local_inflation['cost_scaling_factor'] = 4.5
        global_inflation['cost_scaling_factor'] = 4.5
        local_inflation['inflation_radius'] = 0.50
        global_inflation['inflation_radius'] = 0.50
        return

    if profile == 'adaptive_goal_line_050_recovery_v10_continuous_goal_line':
        # v9 continuously replans and fixes the N->X dead-end, but one pilot
        # selected a long south branch already during M->N because early
        # unknown cells had no goal-line cost.  v10 keeps the same continuous
        # BT and applies a bounded, current start-goal line preference to
        # traversable unknown cells.  The distance scale is deliberately
        # wider than v7/v8 so both candidate detours remain distinguishable
        # instead of saturating at cost 252.  This is still map-independent:
        # no scene coordinates, side schedule, or route waypoint is encoded.
        planner = params['planner_server']['ros__parameters']['GridBased']
        planner['line_bias_enabled'] = True
        planner['line_bias_max_cost'] = 60.0
        planner['line_bias_distance_scale'] = 2.50
        planner['line_bias_exponent'] = 2.0
        planner['line_bias_apply_to_unknown'] = True
        planner['goal_progress_bias_enabled'] = True
        planner['goal_progress_bias_max_cost'] = 30.0
        planner['goal_progress_bias_distance_scale'] = 1.50
        planner['goal_progress_bias_exponent'] = 2.0
        planner['goal_progress_bias_apply_to_unknown'] = True
        planner['unknown_bias_enabled'] = True
        planner['unknown_bias_cost'] = 2.0
        planner['side_bias_enabled'] = False
        planner['side_bias_apply_to_unknown'] = False
        planner['side_bias_target_world_y_enabled'] = False
        planner['side_bias_target_schedule_enabled'] = False
        planner['cost_travel_multiplier'] = 6.0

        controller = params['controller_server']['ros__parameters']['FollowPath']
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

        smoother = params['velocity_smoother']['ros__parameters']
        smoother['max_velocity'] = [0.28, 0.0, 0.90]

        local_inflation = params['local_costmap']['local_costmap'][
            'ros__parameters']['inflation_layer']
        global_inflation = params['global_costmap']['global_costmap'][
            'ros__parameters']['inflation_layer']
        local_inflation['cost_scaling_factor'] = 4.5
        global_inflation['cost_scaling_factor'] = 4.5
        local_inflation['inflation_radius'] = 0.50
        global_inflation['inflation_radius'] = 0.50
        return

    if profile == 'adaptive_goal_line_050_recovery_v11_periodic_replanning':
        # v9's 1 Hz unconditional path replacement could switch to a longer
        # topological branch while the RGB-D map was still changing. v11
        # keeps the generic live-costmap planner and v5 motion settings, but
        # uses the 6 s periodic tree below so each path can be followed long
        # enough for the next observation to become meaningful.
        planner = params['planner_server']['ros__parameters']['GridBased']
        planner['line_bias_enabled'] = True
        planner['line_bias_max_cost'] = 60.0
        planner['line_bias_distance_scale'] = 2.0
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

        controller = params['controller_server']['ros__parameters']['FollowPath']
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

        smoother = params['velocity_smoother']['ros__parameters']
        smoother['max_velocity'] = [0.28, 0.0, 0.90]

        local_inflation = params['local_costmap']['local_costmap'][
            'ros__parameters']['inflation_layer']
        global_inflation = params['global_costmap']['global_costmap'][
            'ros__parameters']['inflation_layer']
        local_inflation['cost_scaling_factor'] = 4.5
        global_inflation['cost_scaling_factor'] = 4.5
        local_inflation['inflation_radius'] = 0.50
        global_inflation['inflation_radius'] = 0.50
        return

    if profile == 'adaptive_goal_line_050_recovery_v12_path_first':
        # v11 still allowed a replanning cycle to select a long branch behind
        # the robot after it had already made progress toward the goal. v12
        # keeps the generic live-costmap and 6 s replanning behavior, but
        # makes the direct-line term a tie-breaker and adds a bounded penalty
        # for cells behind the *current* start-to-goal direction. Both terms
        # are recomputed for every planning call; no world coordinates or
        # recorded route are used.
        planner = params['planner_server']['ros__parameters']['GridBased']
        planner['line_bias_enabled'] = True
        planner['line_bias_max_cost'] = 18.0
        planner['line_bias_distance_scale'] = 2.8
        planner['line_bias_exponent'] = 2.0
        planner['line_bias_apply_to_unknown'] = False
        planner['goal_progress_bias_enabled'] = True
        planner['goal_progress_bias_max_cost'] = 50.0
        planner['goal_progress_bias_distance_scale'] = 1.2
        planner['goal_progress_bias_exponent'] = 2.0
        planner['goal_progress_bias_apply_to_unknown'] = True
        planner['unknown_bias_enabled'] = False
        planner['unknown_bias_cost'] = 0.0
        planner['side_bias_enabled'] = False
        planner['side_bias_apply_to_unknown'] = False
        planner['side_bias_target_world_y_enabled'] = False
        planner['side_bias_target_schedule_enabled'] = False
        planner['cost_travel_multiplier'] = 6.0

        controller = params['controller_server']['ros__parameters']['FollowPath']
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

        smoother = params['velocity_smoother']['ros__parameters']
        smoother['max_velocity'] = [0.28, 0.0, 0.90]

        local_inflation = params['local_costmap']['local_costmap'][
            'ros__parameters']['inflation_layer']
        global_inflation = params['global_costmap']['global_costmap'][
            'ros__parameters']['inflation_layer']
        local_inflation['cost_scaling_factor'] = 4.5
        global_inflation['cost_scaling_factor'] = 4.5
        local_inflation['inflation_radius'] = 0.50
        global_inflation['inflation_radius'] = 0.50
        return

    if profile == 'adaptive_goal_line_050_recovery_v13_line_tiebreaker':
        # v12's generic backward-space penalty was too strong while the live
        # RGB-D map still contained unknown branches. v13 isolates the useful
        # part of the experiment: the current start-to-goal line is only a
        # light tie-breaker, while Smac's actual path length and costmap
        # feasibility remain dominant. The planner is still fully generic and
        # recomputes this line for every goal and every replanning call.
        planner = params['planner_server']['ros__parameters']['GridBased']
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

        controller = params['controller_server']['ros__parameters']['FollowPath']
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

        smoother = params['velocity_smoother']['ros__parameters']
        smoother['max_velocity'] = [0.28, 0.0, 0.90]

        local_inflation = params['local_costmap']['local_costmap'][
            'ros__parameters']['inflation_layer']
        global_inflation = params['global_costmap']['global_costmap'][
            'ros__parameters']['inflation_layer']
        local_inflation['cost_scaling_factor'] = 4.5
        global_inflation['cost_scaling_factor'] = 4.5
        local_inflation['inflation_radius'] = 0.50
        global_inflation['inflation_radius'] = 0.50
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
        'adaptive_goal_line_045, reactive_mppi_static, '
        'adaptive_goal_line_045_recovery_v1, '
        'adaptive_goal_line_045_recovery_v2, '
        'adaptive_goal_line_045_recovery_v3, or '
        'adaptive_goal_line_050_recovery_v4, '
        'adaptive_goal_line_050_recovery_v5, '
        'adaptive_goal_line_050_recovery_v6_goal_directed, or '
        'adaptive_goal_line_050_recovery_v7_unknown_line, or '
        'adaptive_goal_line_050_recovery_v8_cross_track, '
        'adaptive_goal_line_050_recovery_v9_continuous_replanning, '
        'adaptive_goal_line_050_recovery_v10_continuous_goal_line, or '
        'adaptive_goal_line_050_recovery_v11_periodic_replanning, or '
        'adaptive_goal_line_050_recovery_v12_path_first, or '
        'adaptive_goal_line_050_recovery_v13_line_tiebreaker, or '
        'frozen_goal_line_045_v1.'.format(profile))


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

    if profile in (
        'adaptive_goal_line_045_recovery_v1',
        'adaptive_goal_line_045_recovery_v2',
        'adaptive_goal_line_045_recovery_v3',
        'adaptive_goal_line_050_recovery_v4',
        'adaptive_goal_line_050_recovery_v5',
        'adaptive_goal_line_050_recovery_v6_goal_directed',
        'adaptive_goal_line_050_recovery_v7_unknown_line',
        'adaptive_goal_line_050_recovery_v8_cross_track',
        'adaptive_goal_line_050_recovery_v9_continuous_replanning',
        'adaptive_goal_line_050_recovery_v10_continuous_goal_line',
        'adaptive_goal_line_050_recovery_v11_periodic_replanning',
        'adaptive_goal_line_050_recovery_v12_path_first',
        'adaptive_goal_line_050_recovery_v13_line_tiebreaker'):
        bt_path = os.path.join(
            package_share, 'behavior_trees',
            ('navigate_to_pose_goal_directed_recovery.xml'
             if profile in (
                 'adaptive_goal_line_050_recovery_v6_goal_directed',
                 'adaptive_goal_line_050_recovery_v7_unknown_line',
                 'adaptive_goal_line_050_recovery_v8_cross_track')
             else 'navigate_to_pose_continuous_replanning.xml'
             if profile in (
                 'adaptive_goal_line_050_recovery_v9_continuous_replanning',
                 'adaptive_goal_line_050_recovery_v10_continuous_goal_line')
             else 'navigate_to_pose_periodic_replanning_6s.xml'
             if profile in (
                 'adaptive_goal_line_050_recovery_v11_periodic_replanning',
                 'adaptive_goal_line_050_recovery_v12_path_first',
                 'adaptive_goal_line_050_recovery_v13_line_tiebreaker')
             else 'navigate_to_pose_recovery_replanning.xml'))
        through_poses_bt_path = os.path.join(
            package_share, 'behavior_trees',
            ('navigate_through_poses_goal_directed_recovery.xml'
             if profile in (
                 'adaptive_goal_line_050_recovery_v6_goal_directed',
                 'adaptive_goal_line_050_recovery_v7_unknown_line',
                 'adaptive_goal_line_050_recovery_v8_cross_track')
             else 'navigate_through_poses_continuous_replanning.xml'
             if profile in (
                 'adaptive_goal_line_050_recovery_v9_continuous_replanning',
                 'adaptive_goal_line_050_recovery_v10_continuous_goal_line')
             else 'navigate_through_poses_periodic_replanning_6s.xml'
             if profile in (
                 'adaptive_goal_line_050_recovery_v11_periodic_replanning',
                 'adaptive_goal_line_050_recovery_v12_path_first',
                 'adaptive_goal_line_050_recovery_v13_line_tiebreaker')
             else 'navigate_through_poses_recovery_replanning.xml'))
    else:
        bt_path = os.path.join(
            package_share, 'behavior_trees',
            'navigate_to_pose_stable_replanning.xml')
        through_poses_bt_path = os.path.join(
            package_share, 'behavior_trees',
            'navigate_through_poses_stable_replanning.xml')
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
        'fast_goalline_045_v4', 'adaptive_goal_line_045',
        'reactive_mppi_static',
        'adaptive_goal_line_045_recovery_v1',
        'adaptive_goal_line_045_recovery_v2',
        'adaptive_goal_line_045_recovery_v3',
        'adaptive_goal_line_050_recovery_v4',
        'adaptive_goal_line_050_recovery_v5',
        'adaptive_goal_line_050_recovery_v6_goal_directed',
        'adaptive_goal_line_050_recovery_v7_unknown_line',
        'adaptive_goal_line_050_recovery_v8_cross_track',
        'adaptive_goal_line_050_recovery_v9_continuous_replanning',
        'adaptive_goal_line_050_recovery_v10_continuous_goal_line',
        'adaptive_goal_line_050_recovery_v11_periodic_replanning',
        'adaptive_goal_line_050_recovery_v12_path_first',
        'adaptive_goal_line_050_recovery_v13_line_tiebreaker'):
        pass
    elif profile in ('frozen_goal_line_045_v1', 'goal_line_quad_045_v1'):
        params['collision_monitor']['ros__parameters']['PolygonSlow'][
            'slowdown_ratio'] = 0.65
    else:
        raise RuntimeError(
            'Unknown navigation_profile={!r}; use current, fast_north_045_v1, '
            'fast_north_045_v2, fast_north_045_v3, fast_goalline_045_v1, '
            'fast_goalline_045_v2, fast_goalline_045_v3, fast_goalline_045_v4, '
            'adaptive_goal_line_045, reactive_mppi_static, '
            'adaptive_goal_line_045_recovery_v1, '
            'adaptive_goal_line_045_recovery_v2, adaptive_goal_line_045_recovery_v3, '
            'adaptive_goal_line_050_recovery_v4, adaptive_goal_line_050_recovery_v5, '
            'or frozen_goal_line_045_v1.'.format(profile))

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

    world_name = LaunchConfiguration('world').perform(context)
    world_file_override = LaunchConfiguration('world_file').perform(context).strip()
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

    custom_world = None
    if world_file_override:
        custom_world = world_file_override
        if not os.path.isabs(custom_world):
            custom_world = os.path.join(package_share, 'worlds', custom_world)
        custom_world = os.path.realpath(custom_world)
        if not os.path.isfile(custom_world):
            raise RuntimeError(
                f'world_file does not exist: {custom_world}')
    elif world_name in ('obstacle_course', 'obstacle_course_large'):
        world_file = 'indoor_obstacle_course_large.world' if world_name == 'obstacle_course_large' else 'indoor_obstacle_course.world'
        custom_world = os.path.join(package_share, 'worlds', world_file)

    if custom_world is not None:
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
    # Launch the monitor before its lifecycle manager. The upstream include
    # starts the manager first, which can leave collision_monitor inactive when
    # Nav2 is also bringing up its lifecycle services. Inactive monitoring
    # means /cmd_vel_safe has no publisher and the Gazebo base remains still.
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
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['collision_monitor']},
        ],
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

    # Keep a generous gap between process creation and the independent
    # lifecycle manager.  On a cold Gazebo/RTAB-Map start the collision
    # monitor can take longer to create its PointCloud source; starting its
    # manager too soon races the change_state service and leaves the safety
    # chain inactive.  The runner also waits for the active state before
    # sending a navigation goal.
    delayed_collision_monitor = TimerAction(
        period=5.0,
        actions=[
            collision_monitor_node,
            TimerAction(
                period=5.0,
                actions=[collision_monitor_lifecycle_manager]),
        ])

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
            description=(
                'Gazebo world. obstacle_course_large is the online RGB-D obstacle '
                'course; world_file takes precedence when supplied.')),
        DeclareLaunchArgument(
            'world_file',
            default_value='',
            description=(
                'Optional custom SDF filename under the package worlds directory '
                'or an absolute SDF path. It takes precedence over world.')),
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
                'reactive_mppi_static selects the independent Gate 1 MPPI '
                'parameter snapshot without future information; '
                'adaptive_goal_line_045_recovery_v1 adds stronger recovery '
                'actions; adaptive_goal_line_045_recovery_v2 adds clearance '
                'cost weighting and true reverse recovery; '
                'adaptive_goal_line_045_recovery_v3 balances clearance, '
                'feasibility and reduced corner cutting; '
                'adaptive_goal_line_050_recovery_v4 adds a small 0.50 m '
                'inflation margin and weaker line pull; '
                'adaptive_goal_line_050_recovery_v5 restores the generic '
                'line preference and checks collisions earlier; '
                'adaptive_goal_line_050_recovery_v6_goal_directed adds '
                'generic goal-progress and unknown-space costs plus short '
                'back-up/right-turn recovery; '
                'adaptive_goal_line_050_recovery_v7_unknown_line keeps '
                'unknown cells cheap while applying the current goal-line '
                'preference to unknown space; '
                'adaptive_goal_line_050_recovery_v8_cross_track widens the '
                'line distance scale to avoid cost saturation; '
                'adaptive_goal_line_050_recovery_v9_continuous_replanning '
                'recomputes the live path at 1 Hz; '
                'adaptive_goal_line_050_recovery_v10_continuous_goal_line '
                'also applies a bounded current goal-line cost to unknown cells; '
                'adaptive_goal_line_050_recovery_v11_periodic_replanning '
                'replans from the live costmap every 6 seconds to reduce '
                'topological path switching; '
                'adaptive_goal_line_050_recovery_v12_path_first keeps that '
                'cadence, makes the current goal line a soft tie-breaker, '
                'and penalizes generic backward branches; '
                'adaptive_goal_line_050_recovery_v13_line_tiebreaker keeps '
                'only the light current goal-line tie-breaker; '
                'frozen_goal_line_045_v1 restores the pre-optimization run-03 baseline.')),
        OpaqueFunction(function=launch_setup),
    ])
