#!/usr/bin/env python3
"""Run an arbitrary sequence of live Nav2 goals and save split evidence.

Each ``--goal`` creates a fresh NavigateToPose action from the robot's current
pose to the requested waypoint. The same online RGB-D RTAB-Map and Nav2
process is kept alive for the whole sequence; no route or waypoint path is
replayed from a recorded map.
"""

import argparse
import os
import sys
from types import SimpleNamespace

import rclpy
import yaml

from multi_goal_trial import SEGMENT_COLORS, MultiGoalEvidence, safe_metrics
from navigation_trial import NavigationTrial


def parse_goal_spec(raw):
    """Parse NAME:X:Y:YAW while allowing signed numeric coordinates."""
    parts = raw.split(':')
    if len(parts) != 4 or not parts[0].strip():
        raise argparse.ArgumentTypeError(
            f'invalid goal {raw!r}; expected NAME:X:Y:YAW')
    try:
        x = float(parts[1])
        y = float(parts[2])
        yaw = float(parts[3])
    except ValueError as exception:
        raise argparse.ArgumentTypeError(
            f'invalid goal {raw!r}; coordinates must be numeric') from exception
    return {
        'name': parts[0].strip(),
        'x': x,
        'y': y,
        'yaw': yaw,
    }


def build_stages(args, goals):
    stages = []
    for index, goal in enumerate(goals):
        if index == 0:
            start_name = args.start_name
            start_x = args.start_x
            start_y = args.start_y
        else:
            previous = goals[index - 1]
            start_name = previous['name']
            start_x = previous['x']
            start_y = previous['y']
        stages.append({
            # Keep plot legends ASCII-compatible; waypoint labels remain the
            # concise M/A/B/C/D names requested for the comparison figure.
            'name': f'{start_name}->{goal["name"]}',
            'goal_label': goal['name'],
            'start_label': start_name if index == 0 else None,
            'start_x_m': start_x,
            'start_y_m': start_y,
            'goal_x_m': goal['x'],
            'goal_y_m': goal['y'],
            'goal_yaw_rad': goal['yaw'],
            'color': SEGMENT_COLORS[index % len(SEGMENT_COLORS)],
        })
    return stages


def main():
    parser = argparse.ArgumentParser(
        description='Run sequential live RGB-D Nav2 goals and save evidence.')
    parser.add_argument(
        '--goal', action='append', required=True, type=parse_goal_spec,
        help='Waypoint in NAME:X:Y:YAW form; repeat for each destination.')
    parser.add_argument('--start-name', default='M')
    parser.add_argument('--start-x', type=float, default=-8.5)
    parser.add_argument('--start-y', type=float, default=0.0)
    parser.add_argument('--frame', default='map')
    parser.add_argument('--settle-seconds', type=float, default=5.0)
    parser.add_argument('--label', default='多目标导航实验')
    parser.add_argument(
        '--output-dir', default='/workspaces/rtabmap_tb3_nav/results')
    parser.add_argument(
        '--world-file',
        default='/workspaces/rtabmap_tb3_nav/src/rtabmap_tb3_nav/worlds/'
                'indoor_obstacle_course_large.world')
    parser.add_argument(
        '--profile',
        default='adaptive_goal_line_050_recovery_v13_line_tiebreaker')
    parser.add_argument(
        '--dynamic-obstacle-model', default='',
        help='Optional Gazebo model name whose ground-truth path is recorded.')
    args = parser.parse_args()

    if len(args.goal) < 1:
        parser.error('at least one --goal is required')

    output_dir = os.path.join(args.output_dir, args.label)
    os.makedirs(output_dir, exist_ok=True)
    stages = build_stages(args, args.goal)
    trials = []
    stage_metrics = []

    rclpy.init()
    try:
        for index, stage in enumerate(stages):
            trial_args = SimpleNamespace(
                x=stage['goal_x_m'],
                y=stage['goal_y_m'],
                yaw=stage['goal_yaw_rad'],
                frame=args.frame,
                settle_seconds=args.settle_seconds,
                label=f'{args.label}/segment_{index + 1}',
                world_file=args.world_file,
                dynamic_obstacle_model=args.dynamic_obstacle_model,
            )
            trial = NavigationTrial(trial_args)
            error = None
            try:
                trial.run()
            except Exception as exception:
                error = exception
                trial.node.get_logger().error(str(exception))
            stage_record = {
                'name': stage['name'],
                **safe_metrics(trial, error),
            }
            trials.append(trial)
            stage_metrics.append(stage_record)
            trial.node.destroy_node()
            if not stage_record['succeeded']:
                break

        evidence = MultiGoalEvidence(trials, stages)
        evidence.write_all_csv(output_dir)
        for index, (trial, stage) in enumerate(
                zip(trials, stages), start=1):
            evidence.write_csv(
                os.path.join(output_dir, f'第{index}段_{stage["name"]}_轨迹.csv'),
                trial.path, stage['name'], index)
        evidence.write_plots(output_dir, args.label, stage_metrics)

        overall_success = len(stage_metrics) == len(stages) and all(
            stage['succeeded'] for stage in stage_metrics)
        summary = {
            'label': args.label,
            'profile': args.profile,
            'world_file': args.world_file,
            'goal_frame': args.frame,
            'overall_succeeded': overall_success,
            'completed_segments': len(stage_metrics),
            'requested_segments': len(stages),
            'total_wall_duration_s': sum(
                stage['wall_duration_s'] or 0.0 for stage in stage_metrics),
            'total_trajectory_length_m': sum(
                stage['trajectory_length_m'] for stage in stage_metrics),
            'segments': stage_metrics,
        }
        with open(os.path.join(output_dir, '多目标指标.yaml'), 'w',
                  encoding='utf-8') as stream:
            yaml.safe_dump(summary, stream, allow_unicode=True, sort_keys=False)
        with open(os.path.join(output_dir, 'metrics.yaml'), 'w',
                  encoding='utf-8') as stream:
            yaml.safe_dump(summary, stream, allow_unicode=True, sort_keys=False)
        with open(os.path.join(output_dir, '实验参数.yaml'), 'w',
                  encoding='utf-8') as stream:
            yaml.safe_dump({
                'label': args.label,
                'profile': args.profile,
                'world_file': args.world_file,
                'goal_frame': args.frame,
                'settle_seconds': args.settle_seconds,
                'start': {
                    'name': args.start_name,
                    'x_m': args.start_x,
                    'y_m': args.start_y,
                },
                'waypoints': args.goal,
                'stages': stages,
                'trajectory_colors': {
                    stage['name']: stage['color'] for stage in stages},
                'black_reference': '每段实际起点到当前终点直线',
                'mapping_mode': 'online RGB-D RTAB-Map SLAM',
                'dynamic_obstacle_model': args.dynamic_obstacle_model or None,
            }, stream, allow_unicode=True, sort_keys=False)
        exit_code = 0 if overall_success else 5
        print(f'label={args.label}')
        print(f'overall_succeeded={overall_success}')
        for stage in stage_metrics:
            print(
                f"{stage['name']}: status={stage['nav2_status']} "
                f"wall={stage['wall_duration_s']} "
                f"length={stage['trajectory_length_m']:.3f}m")
        return exit_code
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
