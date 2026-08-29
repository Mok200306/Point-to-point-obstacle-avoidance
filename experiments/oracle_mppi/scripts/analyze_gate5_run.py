#!/usr/bin/env python3
"""Analyze one Gate 5 dynamic run and render its time-aligned evidence.

This post-processor never feeds information back to Nav2.  It uses the saved
Gazebo ground truth, commanded velocity stream and the deterministic scenario
schedule only after a run has finished.  The resulting metrics make the Gate 5
questions auditable: did control change before the closest interaction, how
often did the robot stop/reverse, and did Oracle actually publish/score data?
"""

import argparse
import csv
import math
import re
import sys
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.patches import Polygon

SCRIPT_DIR = Path(__file__).resolve().parent
PUBLISHER_ROOT = SCRIPT_DIR.parent.parent.parent / 'src' / 'oracle_prediction_publisher'
if str(PUBLISHER_ROOT) not in sys.path:
    sys.path.insert(0, str(PUBLISHER_ROOT))

from oracle_prediction_publisher.trajectory import WaypointSchedule  # noqa: E402

from plot_gate2_dynamic_run import (  # noqa: E402
    box_corners,
    color_for,
    load_boxes,
)


def load_yaml(path):
    if not path.exists():
        return {}
    try:
        return yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    except yaml.YAMLError:
        result = {}
        for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
            if ': ' not in line:
                continue
            key, value = line.split(': ', 1)
            value = value.strip().strip('"\'')
            if value == 'true':
                result[key] = True
            elif value == 'false':
                result[key] = False
            else:
                try:
                    result[key] = float(value)
                except ValueError:
                    result[key] = value
        return result


def number(row, key):
    value = row.get(key, '')
    try:
        result = float(value)
    except (TypeError, ValueError):
        return math.nan
    return result if math.isfinite(result) else math.nan


def finite_number(value):
    """Return whether a value is a finite real number, including None-safe."""
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def read_dynamic(path):
    rows = []
    with path.open(newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            rows.append({key: number(row, key) for key in (
                'sim_time_s', 'planned_x_m', 'planned_y_m',
                'obstacle_x_m', 'obstacle_y_m', 'robot_x_m', 'robot_y_m',
                'center_distance_m', 'robot_obstacle_clearance_m')})
    return [row for row in rows if math.isfinite(row['sim_time_s'])]


def read_commands(path):
    rows = []
    if not path.exists():
        return rows
    with path.open(newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            stamp = number(row, 'sim_stamp_s')
            vx = number(row, 'linear_x_mps')
            wz = number(row, 'angular_z_rps')
            if math.isfinite(stamp) and math.isfinite(vx) and math.isfinite(wz):
                rows.append({'t': stamp, 'vx': vx, 'wz': wz})
    rows.sort(key=lambda row: row['t'])
    return rows


def group_commands(commands, precision=3):
    """Average duplicate command messages from the same simulation tick."""
    groups = {}
    for row in commands:
        key = round(row['t'], precision)
        groups.setdefault(key, []).append(row)
    return [
        {'t': key,
         'vx': sum(item['vx'] for item in values) / len(values),
         'wz': sum(item['wz'] for item in values) / len(values)}
        for key, values in sorted(groups.items())
    ]


def finite_pair(row, x_key, y_key):
    return (math.isfinite(row[x_key]) and math.isfinite(row[y_key]))


def interp_command(commands, t):
    if not commands:
        return math.nan, math.nan
    nearest = min(commands, key=lambda row: abs(row['t'] - t))
    return nearest['vx'], nearest['wz']


def align_command_clock(commands, reference):
    """Keep only /cmd_vel samples in the Gazebo simulation-time domain."""
    if not commands:
        return [], 'missing'
    stamps = np.asarray([row['t'] for row in commands], dtype=float)
    median_stamp = float(np.median(stamps))
    # A wall-clock stamp is around 1e9 seconds, whereas the Gazebo clock in
    # these experiments is a small simulation-time value.  Do not silently
    # subtract the two domains: mark the command evidence unusable.
    if not finite_number(reference) or abs(median_stamp - float(reference)) > 1000.0:
        return [], 'incompatible_with_gazebo_sim_time'
    return commands, 'sim_aligned'


def command_motion_metrics(commands, reference, conflict_time):
    """Derive transition-based speed metrics without counting startup as a stop.

    A command stream normally begins with zero or very small commands while
    Nav2, TF and the first plan settle.  Treating the first sample below
    0.15 m/s as a slowdown therefore reports a false event at t=0.  Estimate
    cruise speed from the upper quartile of post-startup forward commands and
    require a real high-to-low transition with consecutive low samples.
    """
    startup_exclusion_s = 5.0
    minimum_motion_vx_mps = 0.05
    minimum_low_samples = 3
    commands = [
        row for row in commands
        if math.isfinite(row['t']) and math.isfinite(row['vx']) and
        math.isfinite(row['wz'])
    ]
    elapsed_rows = [
        {**row, 'elapsed_s': row['t'] - reference}
        for row in commands
        if row['t'] - reference >= startup_exclusion_s
    ]
    moving = [row['vx'] for row in elapsed_rows
              if row['vx'] >= minimum_motion_vx_mps]
    if not moving:
        return {
            'startup_exclusion_s': startup_exclusion_s,
            'minimum_motion_vx_mps': minimum_motion_vx_mps,
            'minimum_low_samples': minimum_low_samples,
            'baseline_cruise_vx_mps': None,
            'effective_slowdown_threshold_mps': None,
            'first_effective_slowdown_elapsed_s': None,
            'first_proactive_slowdown_elapsed_s': None,
            'first_post_conflict_slowdown_elapsed_s': None,
            'cruise_samples': 0,
        }

    baseline = float(np.percentile(np.asarray(moving, dtype=float), 75.0))
    # Keep the threshold below a normal cruise command while avoiding a
    # threshold so low that only a full stop is classified as a slowdown.
    slowdown_threshold = max(0.10, min(0.15, 0.75 * baseline))
    high_threshold = max(minimum_motion_vx_mps, 0.90 * baseline)

    first_effective = None
    first_proactive = None
    first_post_conflict = None
    for index, row in enumerate(elapsed_rows):
        if row['vx'] > slowdown_threshold:
            continue
        previous = elapsed_rows[max(0, index - 5):index]
        had_cruise = any(item['vx'] >= high_threshold for item in previous)
        if not had_cruise:
            continue
        following = elapsed_rows[index:index + minimum_low_samples]
        if len(following) < minimum_low_samples or any(
                item['vx'] > slowdown_threshold for item in following):
            continue
        event_time = row['elapsed_s']
        if first_effective is None:
            first_effective = event_time
        if conflict_time is not None and math.isfinite(conflict_time):
            if event_time < conflict_time and first_proactive is None:
                first_proactive = event_time
            if event_time >= conflict_time and first_post_conflict is None:
                first_post_conflict = event_time

    return {
        'startup_exclusion_s': startup_exclusion_s,
        'minimum_motion_vx_mps': minimum_motion_vx_mps,
        'minimum_low_samples': minimum_low_samples,
        'baseline_cruise_vx_mps': baseline,
        'effective_slowdown_threshold_mps': slowdown_threshold,
        'first_effective_slowdown_elapsed_s': first_effective,
        'first_proactive_slowdown_elapsed_s': first_proactive,
        'first_post_conflict_slowdown_elapsed_s': first_post_conflict,
        'cruise_samples': len(moving),
    }


def event_count(text, patterns):
    return sum(len(re.findall(pattern, text, flags=re.IGNORECASE)) for pattern in patterns)


def analyze(run):
    scenario = load_yaml(run / 'scenario.yaml')
    dynamic = load_yaml(run / 'dynamic_summary.yaml')
    metrics = load_yaml(run / 'metrics.yaml')
    experiment = load_yaml(run / 'experiment.yaml')
    rows = read_dynamic(run / 'dynamic_groundtruth.csv')
    if not rows:
        raise RuntimeError(f'no dynamic ground truth rows: {run}')

    reference = dynamic.get('reference_sim_time_s')
    if reference is None:
        reference = rows[0]['sim_time_s']
    reference = float(reference)
    commands, command_clock_domain = align_command_clock(
        group_commands(read_commands(run / 'cmd_vel.csv')), reference)
    schedule = WaypointSchedule(scenario, str(
        scenario.get('difficulty', experiment.get('difficulty', 'medium'))))
    # The threshold is an analysis threshold, not a control parameter.  It is
    # intentionally written to the output so it cannot be changed silently.
    robot_radius = math.hypot(0.33, 0.27)
    obstacle_radius = math.hypot(schedule.obstacle_half_size_m,
                                 schedule.obstacle_half_size_m)
    interaction_threshold = robot_radius + obstacle_radius + 0.15
    horizon = 3.0
    future_samples = np.arange(0.0, horizon + 1.0e-9, 0.1)

    timeline = []
    for row in rows:
        robot_x = row['robot_x_m']
        robot_y = row['robot_y_m']
        elapsed = row['sim_time_s'] - reference
        future_distances = []
        if math.isfinite(robot_x) and math.isfinite(robot_y):
            for tau in future_samples:
                obstacle_x, obstacle_y, _ = schedule.pose_at_elapsed(
                    max(0.0, elapsed + float(tau)))
                future_distances.append(math.hypot(
                    robot_x - obstacle_x, robot_y - obstacle_y))
        future_min = (min(future_distances)
                      if future_distances else math.nan)
        vx, wz = interp_command(commands, row['sim_time_s'])
        timeline.append({
            'sim_time_s': row['sim_time_s'],
            'elapsed_s': elapsed,
            'robot_x_m': robot_x,
            'robot_y_m': robot_y,
            'obstacle_x_m': row['obstacle_x_m'],
            'obstacle_y_m': row['obstacle_y_m'],
            'planned_x_m': row['planned_x_m'],
            'planned_y_m': row['planned_y_m'],
            'center_distance_m': row['center_distance_m'],
            'clearance_m': row['robot_obstacle_clearance_m'],
            'future_min_center_distance_m': future_min,
            'linear_x_mps': vx,
            'angular_z_rps': wz,
        })

    valid_timeline = [row for row in timeline if math.isfinite(row['elapsed_s'])]
    future_conflict = [row for row in valid_timeline
                       if math.isfinite(row['future_min_center_distance_m']) and
                       row['future_min_center_distance_m'] <= interaction_threshold]
    stops = [row for row in valid_timeline
             if math.isfinite(row['linear_x_mps']) and
             abs(row['linear_x_mps']) <= 0.01 and
             abs(row['angular_z_rps']) <= 0.10]
    reverse = [row for row in valid_timeline
               if math.isfinite(row['linear_x_mps']) and
               row['linear_x_mps'] < -0.01]
    valid_clearance = [row for row in valid_timeline
                       if math.isfinite(row['clearance_m'])]
    min_clearance_row = (min(valid_clearance, key=lambda row: row['clearance_m'])
                         if valid_clearance else None)

    first_conflict = future_conflict[0] if future_conflict else None
    first_stop = stops[0] if stops else None
    first_reverse = reverse[0] if reverse else None
    min_clearance_time = (min_clearance_row['elapsed_s']
                          if min_clearance_row else math.nan)
    first_conflict_time = first_conflict['elapsed_s'] if first_conflict else math.nan

    command_metrics = command_motion_metrics(
        commands, reference,
        first_conflict_time if math.isfinite(first_conflict_time) else None)
    first_slow_time = command_metrics['first_effective_slowdown_elapsed_s']
    first_post_conflict_slow_time = command_metrics[
        'first_post_conflict_slowdown_elapsed_s']

    navigation_log = (run / 'navigation.log').read_text(
        encoding='utf-8', errors='replace') if (run / 'navigation.log').exists() else ''
    launch_log = (run / 'launch.log').read_text(
        encoding='utf-8', errors='replace') if (run / 'launch.log').exists() else ''
    all_logs = navigation_log + '\n' + launch_log
    method = 'oracle' if str(experiment.get('oracle_enabled', '')).lower() == 'true' \
        or (run / 'oracle_publisher.log').exists() else 'reactive'

    analysis = {
        'run': str(run),
        'scenario_id': scenario.get('scenario_id', run.parent.name),
        'method': method,
        'interaction_threshold_m': interaction_threshold,
        'analysis_prediction_horizon_s': horizon,
        'analysis_prediction_dt_s': 0.1,
        'reference_sim_time_s': reference,
        'command_clock_domain': command_clock_domain,
        'first_future_conflict_elapsed_s': first_conflict_time,
        # This is a transition-based event, not the first low command in the
        # file.  The raw startup threshold and baseline are recorded below.
        'first_slowdown_elapsed_s': first_slow_time,
        'first_proactive_slowdown_elapsed_s': command_metrics[
            'first_proactive_slowdown_elapsed_s'],
        'first_post_conflict_slowdown_elapsed_s': first_post_conflict_slow_time,
        'startup_exclusion_s': command_metrics['startup_exclusion_s'],
        'baseline_cruise_vx_mps': command_metrics['baseline_cruise_vx_mps'],
        'effective_slowdown_threshold_mps': command_metrics[
            'effective_slowdown_threshold_mps'],
        'minimum_motion_vx_mps': command_metrics['minimum_motion_vx_mps'],
        'minimum_low_samples': command_metrics['minimum_low_samples'],
        'cruise_command_samples': command_metrics['cruise_samples'],
        'slowdown_after_future_conflict_s': (
            first_post_conflict_slow_time - first_conflict_time
            if first_post_conflict_slow_time is not None and
            math.isfinite(first_conflict_time)
            else None),
        'lead_to_min_clearance_s': (
            min_clearance_time - first_slow_time
            if finite_number(min_clearance_time) and finite_number(first_slow_time)
            else None),
        'minimum_recorded_clearance_m': (
            min_clearance_row['clearance_m'] if min_clearance_row else None),
        'minimum_clearance_elapsed_s': min_clearance_time,
        'first_stop_elapsed_s': first_stop['elapsed_s'] if first_stop else None,
        'first_reverse_elapsed_s': first_reverse['elapsed_s'] if first_reverse else None,
        'command_samples_grouped': len(commands),
        'command_slowdown_samples': sum(
            1 for row in valid_timeline
            if math.isfinite(row['linear_x_mps']) and
            command_metrics['effective_slowdown_threshold_mps'] is not None and
            row['linear_x_mps'] <= command_metrics[
                'effective_slowdown_threshold_mps']),
        'command_stop_samples': len(stops),
        'command_reverse_samples': len(reverse),
        'command_stop_ratio': len(stops) / len(valid_timeline)
        if valid_timeline else None,
        'command_reverse_ratio': len(reverse) / len(valid_timeline)
        if valid_timeline else None,
        'maximum_abs_angular_command_rps': max(
            (abs(row['angular_z_rps']) for row in valid_timeline
             if math.isfinite(row['angular_z_rps'])), default=None),
        'progress_recovery_events': event_count(
            navigation_log, [r'Failed to make progress', r'progress checker']),
        'controller_failure_events': event_count(
            navigation_log, [r'controller failed', r'failed to compute control']),
        'collision_monitor_stop_events': event_count(
            all_logs, [r'collision monitor.*stop', r'collision.*detected']),
        'oracle_message_validation_exit': experiment.get(
            'oracle_message_validation_exit', 0 if method == 'reactive' else None),
        'oracle_active_log_lines': metrics.get('oracle_active_log_lines', 0),
        'oracle_stale_log_lines': metrics.get('oracle_stale_log_lines', 0),
        'oracle_publisher_ready_log_lines': metrics.get(
            'oracle_publisher_ready_log_lines', 0),
        'oracle_first_message_log_lines': metrics.get(
            'oracle_first_message_log_lines', 0),
        'nav2_status': metrics.get('nav2_status'),
        'succeeded': metrics.get('succeeded'),
        'gazebo_robot_dynamic_contact': metrics.get(
            'gazebo_robot_dynamic_contact', experiment.get(
                'gazebo_robot_dynamic_contact', False)),
    }
    return scenario, dynamic, metrics, experiment, timeline, analysis


def write_timeline(path, timeline):
    if not timeline:
        return
    fields = list(timeline[0].keys())
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(timeline)


def render(run, scenario, metrics, timeline, analysis, output):
    boxes = load_boxes(run / 'world.sdf') if (run / 'world.sdf').exists() else []
    fig, axes = plt.subplots(2, 2, figsize=(17, 12), constrained_layout=True)

    # Top-left: spatial evidence.
    ax = axes[0, 0]
    ax.set_facecolor('#edf1f4')
    for box in boxes:
        ax.add_patch(Polygon(
            box_corners(box), closed=True, facecolor=color_for(box['name']),
            edgecolor='#263238', linewidth=0.8, alpha=0.82, zorder=1))
        if not box['name'].startswith('wall'):
            ax.text(box['x'], box['y'], box['name'], fontsize=5.5,
                    ha='center', va='center', zorder=2)
    robot = [(row['robot_x_m'], row['robot_y_m']) for row in timeline
             if math.isfinite(row['robot_x_m']) and math.isfinite(row['robot_y_m'])]
    planned = [(row['planned_x_m'], row['planned_y_m']) for row in timeline
               if math.isfinite(row['planned_x_m']) and math.isfinite(row['planned_y_m'])]
    obstacle = [(row['obstacle_x_m'], row['obstacle_y_m']) for row in timeline
                if math.isfinite(row['obstacle_x_m']) and math.isfinite(row['obstacle_y_m'])]
    if len(robot) > 1:
        ax.plot(*zip(*robot), color='#d62728', linewidth=2.0, label='robot Gazebo')
    if len(planned) > 1:
        ax.plot(*zip(*planned), '--', color='#1f77b4', linewidth=1.2,
                label='obstacle schedule')
    if len(obstacle) > 1:
        ax.plot(*zip(*obstacle), color='#9467bd', linewidth=1.5,
                label='obstacle actual')
    if robot:
        ax.scatter(*robot[0], color='#2ca02c', s=55, label='robot start')
        ax.scatter(*robot[-1], color='#1f77b4', marker='x', s=65, label='robot end')
    goal = (float(scenario['robot']['goal_x']), float(scenario['robot']['goal_y']))
    ax.scatter(*goal, facecolors='none', edgecolors='#ff7f0e', marker='*',
               s=140, linewidths=1.8, label='goal')
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.grid(True, alpha=0.25)
    ax.set_title('Gazebo top-down trajectory')
    ax.legend(fontsize=7, loc='upper right')

    times = np.asarray([row['elapsed_s'] for row in timeline], dtype=float)
    actual_clearance = np.asarray([row['clearance_m'] for row in timeline], dtype=float)
    center_distance = np.asarray([row['center_distance_m'] for row in timeline], dtype=float)
    future_distance = np.asarray([
        row['future_min_center_distance_m'] for row in timeline], dtype=float)
    ax = axes[0, 1]
    valid = np.isfinite(times) & np.isfinite(actual_clearance)
    if np.any(valid):
        ax.plot(times[valid], actual_clearance[valid], color='#d62728',
                label='actual boundary clearance')
    valid_center = np.isfinite(times) & np.isfinite(center_distance)
    if np.any(valid_center):
        ax.plot(times[valid_center], center_distance[valid_center], color='#9467bd',
                alpha=0.75, label='center distance')
    valid_future = np.isfinite(times) & np.isfinite(future_distance)
    if np.any(valid_future):
        ax.plot(times[valid_future], future_distance[valid_future], color='#1f77b4',
                linestyle='--', label='min future center distance (3 s)')
    ax.axhline(analysis['interaction_threshold_m'], color='#ff7f0e',
               linestyle=':', label='analysis interaction threshold')
    if finite_number(analysis['first_future_conflict_elapsed_s']):
        ax.axvline(analysis['first_future_conflict_elapsed_s'], color='#1f77b4',
                    alpha=0.6, linestyle='--')
    if finite_number(analysis['first_slowdown_elapsed_s']):
        ax.axvline(analysis['first_slowdown_elapsed_s'], color='#2ca02c',
                    alpha=0.6, linestyle='-.')
    ax.set_xlabel('elapsed simulation time [s]')
    ax.set_ylabel('distance [m]')
    ax.grid(True, alpha=0.25)
    ax.set_title('Dynamic interaction and prediction window')
    ax.legend(fontsize=7, loc='best')

    vx = np.asarray([row['linear_x_mps'] for row in timeline], dtype=float)
    wz = np.asarray([row['angular_z_rps'] for row in timeline], dtype=float)
    ax = axes[1, 0]
    valid_vx = np.isfinite(times) & np.isfinite(vx)
    if np.any(valid_vx):
        ax.plot(times[valid_vx], vx[valid_vx], color='#2ca02c', label='cmd vx')
    ax.axhline(0.15, color='#ff7f0e', linestyle=':', label='slowdown threshold')
    ax.axhline(0.0, color='black', linewidth=0.8)
    ax.set_xlabel('elapsed simulation time [s]')
    ax.set_ylabel('linear command [m/s]')
    ax.grid(True, alpha=0.25)
    ax2 = ax.twinx()
    valid_wz = np.isfinite(times) & np.isfinite(wz)
    if np.any(valid_wz):
        ax2.plot(times[valid_wz], wz[valid_wz], color='#8c564b', alpha=0.7,
                 label='cmd wz')
    ax2.set_ylabel('angular command [rad/s]')
    if finite_number(analysis['first_future_conflict_elapsed_s']):
        ax.axvline(analysis['first_future_conflict_elapsed_s'], color='#1f77b4',
                   alpha=0.6, linestyle='--')
    if finite_number(analysis['first_slowdown_elapsed_s']):
        ax.axvline(analysis['first_slowdown_elapsed_s'], color='#2ca02c',
                   alpha=0.6, linestyle='-.')
    ax.set_title('Command response')
    handles, labels = ax.get_legend_handles_labels()
    handles2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(handles + handles2, labels + labels2, fontsize=7, loc='best')

    ax = axes[1, 1]
    ax.axis('off')
    method = analysis['method']
    lines = [
        f"method: {method}",
        f"status/success: {analysis.get('nav2_status')} / {analysis.get('succeeded')}",
        f"min clearance: {analysis.get('minimum_recorded_clearance_m')}",
        f"future conflict first: {analysis.get('first_future_conflict_elapsed_s')}",
        f"slowdown first: {analysis.get('first_slowdown_elapsed_s')}",
        f"lead to min clearance: {analysis.get('lead_to_min_clearance_s')}",
        f"stop samples / reverse samples: {analysis.get('command_stop_samples')} / {analysis.get('command_reverse_samples')}",
        f"progress recoveries: {analysis.get('progress_recovery_events')}",
        f"dynamic contact: {analysis.get('gazebo_robot_dynamic_contact')}",
    ]
    if method == 'oracle':
        lines.extend([
            f"Oracle active log lines: {analysis.get('oracle_active_log_lines')}",
            f"Oracle stale log lines: {analysis.get('oracle_stale_log_lines')}",
            f"message validation exit: {analysis.get('oracle_message_validation_exit')}",
        ])
    ax.text(0.02, 0.98, '\n'.join(lines), va='top', family='monospace', fontsize=10)
    ax.set_title('Gate 5 audit summary')

    scenario_id = scenario.get('scenario_id', run.parent.name)
    fig.suptitle(
        f'{scenario_id} | {run.name} | {method} | '
        f"status={analysis.get('nav2_status')} success={analysis.get('succeeded')}",
        fontsize=14)
    fig.savefig(output, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', required=True)
    parser.add_argument('--output', default='gate5_timeline.png')
    args = parser.parse_args()
    run = Path(args.run).resolve()
    scenario, dynamic, metrics, experiment, timeline, analysis = analyze(run)
    write_timeline(run / 'gate5_timeseries.csv', timeline)
    (run / 'gate5_analysis.yaml').write_text(
        yaml.safe_dump(analysis, sort_keys=False), encoding='utf-8')
    render(run, scenario, metrics, timeline, analysis, run / args.output)
    print(run / args.output)


if __name__ == '__main__':
    main()
