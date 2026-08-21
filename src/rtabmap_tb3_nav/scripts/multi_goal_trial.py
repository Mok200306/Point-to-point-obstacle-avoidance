#!/usr/bin/env python3
"""Run two sequential Nav2 goals and save red/blue split trajectory evidence.

The same online RTAB-Map and Nav2 process is used for both legs. Each leg is
still a new NavigateToPose action, so the second goal is planned from the
current pose and live costmap rather than from a recorded route.
"""

import argparse
import csv
import math
import os
import sys
from types import SimpleNamespace

os.environ.setdefault('MPLCONFIGDIR', '/tmp/rtabmap_matplotlib')
os.makedirs(os.environ['MPLCONFIGDIR'], exist_ok=True)

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import yaml
from matplotlib.patches import Circle, Polygon

import rclpy

from navigation_trial import NavigationTrial, path_length


SEGMENT_COLORS = [
    '#d62728',  # red
    '#1f77b4',  # blue
    '#9467bd',  # purple
    '#ff7f0e',  # orange
    '#2ca02c',  # green
    '#8c564b',
]


def safe_metrics(trial, error=None):
    if trial.wall_start_mono is None:
        return {
            'goal_frame': trial.args.frame,
            'goal_x_m': trial.args.x,
            'goal_y_m': trial.args.y,
            'goal_yaw_rad': trial.args.yaw,
            'nav2_status': -1,
            'succeeded': False,
            'wall_duration_s': None,
            'simulation_duration_s': None,
            'samples': len(trial.path),
            'trajectory_length_m': path_length(trial.path),
            'gazebo_samples': len(trial.gazebo_path),
            'gazebo_trajectory_length_m': path_length(trial.gazebo_path),
            'final_xy_error_m': None,
            'error': str(error) if error else None,
        }
    metrics = trial.metrics()
    if error:
        metrics['error'] = str(error)
    return metrics


class MultiGoalEvidence:
    def __init__(self, trials, stages):
        self.trials = trials
        self.stages = stages
        self.world_objects = trials[0].world_objects if trials else []
        self.world_markers = trials[0].world_markers if trials else {}
        self.map_message = next(
            (trial.map_message for trial in reversed(trials)
             if trial.map_message is not None), None)
        self.costmap_message = next(
            (trial.costmap_message for trial in reversed(trials)
             if trial.costmap_message is not None), None)
        self.path_segments = [trial.path for trial in trials]
        self.gazebo_segments = [trial.gazebo_path for trial in trials]
        while len(self.path_segments) < len(stages):
            self.path_segments.append([])
        while len(self.gazebo_segments) < len(stages):
            self.gazebo_segments.append([])
        self.path = [sample for segment in self.path_segments for sample in segment]
        self.gazebo_path = [
            sample for segment in self.gazebo_segments for sample in segment]

    def world_bounds(self):
        points = []
        for obstacle in self.world_objects:
            if obstacle['kind'] == 'circle':
                radius = obstacle['radius']
                points.extend([
                    (obstacle['x'] - radius, obstacle['y'] - radius),
                    (obstacle['x'] + radius, obstacle['y'] + radius),
                ])
                continue
            for local_x, local_y in (
                    (-obstacle['size_x'] / 2.0, -obstacle['size_y'] / 2.0),
                    (-obstacle['size_x'] / 2.0, obstacle['size_y'] / 2.0),
                    (obstacle['size_x'] / 2.0, -obstacle['size_y'] / 2.0),
                    (obstacle['size_x'] / 2.0, obstacle['size_y'] / 2.0)):
                points.append((
                    obstacle['x'] + math.cos(obstacle['yaw']) * local_x -
                    math.sin(obstacle['yaw']) * local_y,
                    obstacle['y'] + math.sin(obstacle['yaw']) * local_x +
                    math.cos(obstacle['yaw']) * local_y))
        for stage in self.stages:
            points.append((stage['goal_x_m'], stage['goal_y_m']))
        for samples in self.path_segments + self.gazebo_segments:
            points.extend((sample['x'], sample['y']) for sample in samples)
        if not points:
            return (-10.5, 10.5, -7.5, 7.5)
        x_values, y_values = zip(*points)
        return (
            min(x_values) - 0.5, max(x_values) + 0.5,
            min(y_values) - 0.5, max(y_values) + 0.5)

    def set_axis_style(self, axis):
        min_x, max_x, min_y, max_y = self.world_bounds()
        axis.set_xlim(min_x, max_x)
        axis.set_ylim(min_y, max_y)
        axis.set_xlabel('x [m]')
        axis.set_ylabel('y [m]')
        axis.set_aspect('equal', adjustable='box')
        axis.grid(True, alpha=0.25)

    @staticmethod
    def draw_grid(axis, message, alpha, label):
        return NavigationTrial.draw_grid(axis, message, alpha, label)

    @staticmethod
    def draw_costmap(axis, message, alpha=0.48):
        return NavigationTrial.draw_costmap(axis, message, alpha)

    def draw_obstacles(self, axis):
        for obstacle in self.world_objects:
            if obstacle['kind'] == 'circle':
                axis.add_patch(Circle(
                    (obstacle['x'], obstacle['y']), obstacle['radius'],
                    facecolor=obstacle['color'], edgecolor='#263238',
                    linewidth=1.0, alpha=0.88, zorder=3))
                continue
            corners = []
            for local_x, local_y in (
                    (-obstacle['size_x'] / 2.0, -obstacle['size_y'] / 2.0),
                    (-obstacle['size_x'] / 2.0, obstacle['size_y'] / 2.0),
                    (obstacle['size_x'] / 2.0, obstacle['size_y'] / 2.0),
                    (obstacle['size_x'] / 2.0, -obstacle['size_y'] / 2.0)):
                corners.append((
                    obstacle['x'] + math.cos(obstacle['yaw']) * local_x -
                    math.sin(obstacle['yaw']) * local_y,
                    obstacle['y'] + math.sin(obstacle['yaw']) * local_x +
                    math.cos(obstacle['yaw']) * local_y))
            axis.add_patch(Polygon(
                corners, closed=True, facecolor=obstacle['color'],
                edgecolor='#263238', linewidth=1.0, alpha=0.88, zorder=3))
            if not obstacle['name'].startswith('wall'):
                axis.text(obstacle['x'], obstacle['y'], obstacle['name'],
                          fontsize=6, ha='center', va='center', zorder=4)

        for marker_name, (x, y, _) in self.world_markers.items():
            color = '#2ca02c' if marker_name == 'start_marker' else '#d62728'
            axis.scatter(x, y, color=color, s=90, marker='o', zorder=6)

    def segment_start(self, index, gazebo=False):
        samples = (self.gazebo_segments if gazebo else self.path_segments)[index]
        if samples:
            return samples[0]['x'], samples[0]['y']
        if index == 0:
            return None
        previous = (self.gazebo_segments if gazebo else self.path_segments)[index - 1]
        if previous:
            return previous[-1]['x'], previous[-1]['y']
        return self.stages[index - 1]['goal_x_m'], self.stages[index - 1]['goal_y_m']

    def draw_segmented_paths(self, axis, gazebo=False):
        segments = self.gazebo_segments if gazebo else self.path_segments
        for index, stage in enumerate(self.stages):
            samples = segments[index]
            color = stage.get(
                'color', SEGMENT_COLORS[index % len(SEGMENT_COLORS)])
            segment_title = stage.get('name', f'segment {index + 1}')
            if samples:
                axis.plot(
                    [sample['x'] for sample in samples],
                    [sample['y'] for sample in samples],
                    color=color, linewidth=2.8,
                    label=f"{segment_title} actual trajectory", zorder=8)
                axis.scatter(
                    samples[0]['x'], samples[0]['y'], color=color, s=55,
                    marker='o', zorder=9)
            start = self.segment_start(index, gazebo=gazebo)
            goal = (stage['goal_x_m'], stage['goal_y_m'])
            if start is not None:
                axis.plot(
                    [start[0], goal[0]], [start[1], goal[1]],
                    color='#111111', linestyle='--', linewidth=1.8,
                    label=f"{segment_title} direct start-goal line"
                    if index == 0 else None,
                    zorder=7)
            axis.scatter(
                goal[0], goal[1], facecolors='none', edgecolors=color,
                s=160, linewidths=2.0, marker='*',
                label=f"{segment_title} goal", zorder=10)
            goal_label = stage.get('goal_label')
            if goal_label:
                axis.annotate(
                    goal_label, goal, xytext=(7, 7),
                    textcoords='offset points', fontsize=10,
                    fontweight='bold', color='#111111', zorder=11,
                    bbox={
                        'boxstyle': 'round,pad=0.15',
                        'facecolor': 'white',
                        'edgecolor': color,
                        'alpha': 0.9,
                    })
            if index == 0 and start is not None and stage.get('start_label'):
                axis.annotate(
                    stage['start_label'], start, xytext=(7, -16),
                    textcoords='offset points', fontsize=10,
                    fontweight='bold', color='#111111', zorder=11,
                    bbox={
                        'boxstyle': 'round,pad=0.15',
                        'facecolor': 'white',
                        'edgecolor': '#111111',
                        'alpha': 0.9,
                    })

    def draw_gazebo_view(self, axis):
        axis.set_facecolor('#edf1f4')
        self.draw_obstacles(axis)
        self.draw_segmented_paths(axis, gazebo=True)
        self.set_axis_style(axis)
        axis.set_title('Gazebo top-down view + ground truth trajectory')
        axis.legend(loc='upper right', fontsize=8)

    def draw_rviz_view(self, axis):
        axis.set_facecolor('#edf1f4')
        drew_map = self.draw_grid(
            axis, self.map_message, 0.86, 'RTAB-Map occupancy')
        drew_costmap = self.draw_costmap(axis, self.costmap_message)
        if not drew_map and not drew_costmap:
            axis.text(0.5, 0.5, 'No map or costmap received',
                      transform=axis.transAxes, ha='center')
        self.draw_segmented_paths(axis, gazebo=False)
        self.set_axis_style(axis)
        axis.set_title('RViz-style /map + global costmap')
        axis.legend(loc='upper right', fontsize=8)

    def write_plots(self, output_dir, label, stage_metrics):
        figure, axes = plt.subplots(
            1, 2, figsize=(19, 8), constrained_layout=True)
        self.draw_gazebo_view(axes[0])
        self.draw_rviz_view(axes[1])
        statuses = '/'.join(
            str(stage.get('nav2_status', 'not_run'))
            for stage in stage_metrics)
        total_wall = sum(
            stage.get('wall_duration_s') or 0.0
            for stage in stage_metrics)
        color_summary = ', '.join(
            f"{stage.get('goal_label', stage.get('name', index + 1))}="
            f"{stage.get('color', SEGMENT_COLORS[index % len(SEGMENT_COLORS)])}"
            for index, stage in enumerate(self.stages))
        figure.suptitle(
            f'adaptive multi-goal experiment | status={statuses} | '
            f'total wall={total_wall:.1f}s | segments: {color_summary} | '
            f'black dashed=direct start-goal reference', fontsize=14)
        figure.savefig(os.path.join(output_dir, 'trajectory_comparison.png'),
                       dpi=170)
        plt.close(figure)

        figure, axis = plt.subplots(figsize=(11, 8), constrained_layout=True)
        self.draw_rviz_view(axis)
        figure.savefig(os.path.join(output_dir, 'trajectory.png'), dpi=160)
        plt.close(figure)

    @staticmethod
    def write_csv(path, samples, segment_name, segment_index):
        fields = [
            'segment', 'segment_name', 'wall_time', 'wall_elapsed_s',
            'sim_time', 'x', 'y', 'yaw']
        with open(path, 'w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for sample in samples:
                writer.writerow({
                    'segment': segment_index,
                    'segment_name': segment_name,
                    **{field: sample.get(field) for field in fields[2:]},
                })

    def write_all_csv(self, output_dir):
        all_fields = [
            'segment', 'segment_name', 'wall_time', 'wall_elapsed_s',
            'sim_time', 'x', 'y', 'yaw']
        with open(os.path.join(output_dir, '多目标轨迹.csv'), 'w',
                  newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=all_fields)
            writer.writeheader()
            for index, (stage, samples) in enumerate(
                    zip(self.stages, self.path_segments), start=1):
                for sample in samples:
                    writer.writerow({
                        'segment': index,
                        'segment_name': stage['name'],
                        **{field: sample.get(field) for field in all_fields[2:]},
                    })
        with open(os.path.join(output_dir, 'Gazebo真值多目标轨迹.csv'), 'w',
                  newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=all_fields)
            writer.writeheader()
            for index, (stage, samples) in enumerate(
                    zip(self.stages, self.gazebo_segments), start=1):
                for sample in samples:
                    writer.writerow({
                        'segment': index,
                        'segment_name': stage['name'],
                        **{field: sample.get(field) for field in all_fields[2:]},
                    })


def main():
    parser = argparse.ArgumentParser(
        description='Run 起点->A->B sequential goals and save split evidence.')
    parser.add_argument('--goal-a-x', type=float, required=True)
    parser.add_argument('--goal-a-y', type=float, required=True)
    parser.add_argument('--goal-a-yaw', type=float, default=0.0)
    parser.add_argument('--goal-b-x', type=float, required=True)
    parser.add_argument('--goal-b-y', type=float, required=True)
    parser.add_argument('--goal-b-yaw', type=float, default=0.0)
    parser.add_argument('--frame', default='map')
    parser.add_argument('--settle-seconds', type=float, default=5.0)
    parser.add_argument('--label', default='多目标导航实验')
    parser.add_argument(
        '--output-dir', default='/workspaces/rtabmap_tb3_nav/results')
    parser.add_argument(
        '--world-file',
        default='/workspaces/rtabmap_tb3_nav/src/rtabmap_tb3_nav/worlds/'
                'indoor_obstacle_course_large.world')
    parser.add_argument('--profile', default='adaptive_goal_line_045')
    args = parser.parse_args()

    output_dir = os.path.join(args.output_dir, args.label)
    os.makedirs(output_dir, exist_ok=True)
    stages = [
        {
            'name': '起点到A',
            'goal_x_m': args.goal_a_x,
            'goal_y_m': args.goal_a_y,
            'goal_yaw_rad': args.goal_a_yaw,
        },
        {
            'name': 'A到B',
            'goal_x_m': args.goal_b_x,
            'goal_y_m': args.goal_b_y,
            'goal_yaw_rad': args.goal_b_yaw,
        },
    ]
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
                'goals': stages,
                'red_segment': '起点到A',
                'blue_segment': 'A到B',
                'black_reference': '每段实际起点到当前终点直线',
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
