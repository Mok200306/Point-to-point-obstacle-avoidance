#!/usr/bin/env python3
"""Summarize Gate 5 paired smoke evidence without deleting any run."""

import argparse
import csv
import re
from pathlib import Path

import yaml


def load(path):
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


def value(*sources, key, default=''):
    for source in sources:
        if key in source:
            return source[key]
    return default


RUN_NAME_RE = re.compile(r'^(?:reactive|oracle)_run_[0-9]+$')


def as_bool(value_):
    if isinstance(value_, bool):
        return value_
    if isinstance(value_, str):
        lowered = value_.strip().lower()
        if lowered in {'true', 'yes', '1'}:
            return True
        if lowered in {'false', 'no', '0'}:
            return False
    return None


def run_directories(root):
    """Find planned run directories, including startup-failure runs.

    A completed run has ``gate5_analysis.yaml``, but a startup failure exits
    before metrics, analysis, or trajectory files are produced.  The runner
    still writes ``experiment.yaml`` and ``scenario.yaml``.  Scanning only
    the analysis file therefore silently dropped exactly the failures that a
    Gate decision must count.
    """
    directories = set()
    for marker in ('experiment.yaml', 'scenario.yaml'):
        for marker_path in root.rglob(marker):
            directory = marker_path.parent
            if RUN_NAME_RE.fullmatch(directory.name):
                directories.add(directory)
    return sorted(directories)


def load_status(path):
    """Map runner matrix rows by normalized artifact label."""
    if not path:
        return {}
    path = Path(path)
    if not path.exists():
        return {}
    rows = {}
    with path.open(newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            label = row.get('label', '')
            if not label:
                continue
            label_path = Path(label)
            if not label_path.is_absolute():
                label_path = Path.cwd() / label_path
            rows[str(label_path.resolve())] = row
    return rows


def method_for(run, analysis, experiment, metrics):
    for source in (analysis, experiment, metrics):
        method = str(source.get('method', '')).strip().lower()
        if method in {'reactive', 'oracle'}:
            return method
    profile = str(value(experiment, metrics, key='profile', default=''))
    oracle_enabled = as_bool(value(
        experiment, metrics, key='oracle_enabled', default=False))
    if oracle_enabled or 'oracle' in profile.lower() or \
            (run / 'oracle_publisher_gate3.yaml').exists() or \
            (run / 'oracle_publisher.log').exists():
        return 'oracle'
    return 'reactive'


def outcome_for(*, startup_failure, succeeded, timed_out, dynamic_contact,
                non_ground_contact, oracle_validation_exit):
    """Classify the observable run outcome without hiding orthogonal facts."""
    if startup_failure:
        return 'STARTUP_FAILURE'
    if oracle_validation_exit not in ('', None, 0, 0.0, '0', '0.0'):
        return 'ORACLE_INTERFACE_FAILURE'
    if succeeded is True and dynamic_contact is True:
        return 'NAV_SUCCESS_WITH_DYNAMIC_CONTACT'
    if succeeded is True and non_ground_contact is True:
        return 'NAV_SUCCESS_WITH_NON_GROUND_CONTACT'
    if succeeded is False and timed_out is True and dynamic_contact is True:
        return 'NAV_TIMEOUT_WITH_DYNAMIC_CONTACT'
    if succeeded is False and timed_out is True:
        return 'NAV_TIMEOUT'
    if succeeded is True:
        return 'PASS'
    if succeeded is False:
        return 'NAV_FAILURE'
    return 'INCOMPLETE_EVIDENCE'


def run_rows(root, status_path=''):
    rows = []
    status_rows = load_status(status_path)
    for run in run_directories(root):
        analysis = load(run / 'gate5_analysis.yaml')
        metrics = load(run / 'metrics.yaml')
        experiment = load(run / 'experiment.yaml')
        run_status = status_rows.get(str(run.resolve()), {})
        method = method_for(run, analysis, experiment, metrics)
        succeeded = as_bool(value(analysis, metrics, key='succeeded', default=''))
        timed_out = as_bool(value(metrics, key='goal_timed_out', default=''))
        dynamic_contact = as_bool(value(
            metrics, experiment, key='gazebo_robot_dynamic_contact', default=''))
        non_ground_contact = as_bool(value(
            metrics, experiment, key='gazebo_non_ground_contact', default=''))
        oracle_validation_exit = value(
            analysis, experiment, metrics,
            key='oracle_message_validation_exit', default='')
        startup_failure = as_bool(value(
            experiment, key='startup_failure', default=False))
        if startup_failure is not True and not (run / 'metrics.yaml').exists():
            # The runner records startup failures in matrix_status even when
            # it exits before the metrics writer is reached.  Do not label an
            # absent-metrics directory as a navigation failure.
            status_exit = run_status.get('exit_code', '')
            if status_exit not in {'', '0'}:
                startup_failure = True
        if startup_failure is None:
            startup_failure = False
        acceptance_pass = succeeded is True and \
            dynamic_contact is False and non_ground_contact is False
        outcome = outcome_for(
            startup_failure=startup_failure,
            succeeded=succeeded,
            timed_out=timed_out,
            dynamic_contact=dynamic_contact,
            non_ground_contact=non_ground_contact,
            oracle_validation_exit=oracle_validation_exit,
        )
        rows.append({
            'scenario_id': value(analysis, experiment, metrics,
                                 key='scenario_id', default=run.parent.name),
            'method': method,
            'run': run.name,
            # Reactive and Oracle evidence directories intentionally carry
            # different method prefixes.  Keep the raw directory name above,
            # but also expose a stable pair key for side-by-side aggregation.
            'pair_group': pair_group(run),
            'run_path': str(run),
            'matrix_recorded': bool(run_status),
            'matrix_exit_code': run_status.get('exit_code', ''),
            'matrix_started_at': run_status.get('started_at', ''),
            'matrix_finished_at': run_status.get('finished_at', ''),
            'git_commit': value(experiment, metrics, key='git_commit'),
            'profile': value(experiment, key='profile'),
            'nav2_params': value(experiment, key='nav2_params'),
            'prediction_cost_weight': value(
                analysis, experiment, metrics, key='prediction_cost_weight'),
            'world_file': value(experiment, metrics, key='world_file'),
            'succeeded': value(analysis, metrics, key='succeeded'),
            'nav2_status': value(analysis, metrics, key='nav2_status'),
            'simulation_duration_s': value(metrics, key='simulation_duration_s'),
            'wall_duration_s': value(metrics, key='wall_duration_s'),
            'gazebo_trajectory_length_m': value(
                metrics, key='gazebo_trajectory_length_m'),
            'final_xy_error_m': value(metrics, key='final_xy_error_m'),
            'minimum_robot_obstacle_clearance_m': value(
                experiment, metrics, key='minimum_robot_obstacle_clearance_m'),
            'minimum_recorded_clearance_m': value(
                analysis, key='minimum_recorded_clearance_m'),
            'first_future_conflict_elapsed_s': value(
                analysis, key='first_future_conflict_elapsed_s'),
            'first_slowdown_elapsed_s': value(
                analysis, key='first_slowdown_elapsed_s'),
            'slowdown_after_future_conflict_s': value(
                analysis, key='slowdown_after_future_conflict_s'),
            'lead_to_min_clearance_s': value(
                analysis, key='lead_to_min_clearance_s'),
            'command_stop_ratio': value(analysis, key='command_stop_ratio'),
            'command_reverse_ratio': value(analysis, key='command_reverse_ratio'),
            'progress_recovery_events': value(
                analysis, key='progress_recovery_events'),
            'controller_failure_events': value(
                analysis, key='controller_failure_events'),
            'collision_monitor_stop_events': value(
                analysis, key='collision_monitor_stop_events'),
            'gazebo_robot_dynamic_contact': value(
                metrics, experiment, key='gazebo_robot_dynamic_contact'),
            'gazebo_non_ground_contact': value(
                metrics, experiment, key='gazebo_non_ground_contact'),
            'oracle_message_validation_exit': value(
                analysis, experiment, key='oracle_message_validation_exit'),
            'oracle_active_log_lines': value(analysis, metrics,
                                             key='oracle_active_log_lines'),
            'oracle_stale_log_lines': value(analysis, metrics,
                                            key='oracle_stale_log_lines'),
            'oracle_publisher_ready_log_lines': value(
                analysis, metrics, key='oracle_publisher_ready_log_lines'),
            'evidence_complete': all((run / name).exists() for name in (
                'experiment.yaml', 'metrics.yaml', 'dynamic_groundtruth.csv',
                'dynamic_summary.yaml', 'cmd_vel.csv', 'gate5_analysis.yaml',
                'gate5_timeseries.csv', 'gate5_timeline.png')),
            'trial_exit_code': value(
                experiment, metrics, key='trial_exit_code', default=value(
                    metrics, key='wrapper_trial_exit', default='')),
            'startup_failure': startup_failure,
            'nav2_succeeded': succeeded if succeeded is not None else '',
            'goal_timed_out': timed_out if timed_out is not None else '',
            'acceptance_pass': acceptance_pass,
            'run_outcome': outcome,
        })
    return rows


def write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0]) if rows else [
        'scenario_id', 'method', 'run', 'run_path', 'evidence_complete']
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)


def write_pairs(path, rows):
    grouped = {}
    for row in rows:
        grouped.setdefault((row['scenario_id'], row['pair_group']), {})[
            row['method']] = row
    pair_rows = []
    for (scenario_id, pair_group_name), methods in sorted(grouped.items()):
        reactive = methods.get('reactive', {})
        oracle = methods.get('oracle', {})
        pair_rows.append({
            'scenario_id': scenario_id,
            'pair_group': pair_group_name,
            'reactive_run': reactive.get('run', ''),
            'oracle_run': oracle.get('run', ''),
            'reactive_present': bool(reactive),
            'oracle_present': bool(oracle),
            'reactive_success': reactive.get('succeeded', ''),
            'oracle_success': oracle.get('succeeded', ''),
            'reactive_nav2_succeeded': reactive.get('nav2_succeeded', ''),
            'oracle_nav2_succeeded': oracle.get('nav2_succeeded', ''),
            'reactive_acceptance_pass': reactive.get('acceptance_pass', False),
            'oracle_acceptance_pass': oracle.get('acceptance_pass', False),
            'reactive_outcome': reactive.get('run_outcome', ''),
            'oracle_outcome': oracle.get('run_outcome', ''),
            'reactive_runner_exit_code': reactive.get('matrix_exit_code', ''),
            'oracle_runner_exit_code': oracle.get('matrix_exit_code', ''),
            'reactive_trial_exit_code': reactive.get('trial_exit_code', ''),
            'oracle_trial_exit_code': oracle.get('trial_exit_code', ''),
            'reactive_prediction_cost_weight': reactive.get(
                'prediction_cost_weight', ''),
            'oracle_prediction_cost_weight': oracle.get(
                'prediction_cost_weight', ''),
            'reactive_simulation_duration_s': reactive.get('simulation_duration_s', ''),
            'oracle_simulation_duration_s': oracle.get('simulation_duration_s', ''),
            'delta_oracle_minus_reactive_simulation_duration_s': _delta(
                oracle, reactive, 'simulation_duration_s'),
            'reactive_wall_duration_s': reactive.get('wall_duration_s', ''),
            'oracle_wall_duration_s': oracle.get('wall_duration_s', ''),
            'delta_oracle_minus_reactive_wall_duration_s': _delta(
                oracle, reactive, 'wall_duration_s'),
            'reactive_minimum_recorded_clearance_m': reactive.get(
                'minimum_recorded_clearance_m', ''),
            'oracle_minimum_recorded_clearance_m': oracle.get(
                'minimum_recorded_clearance_m', ''),
            'delta_oracle_minus_reactive_clearance_m': _delta(
                oracle, reactive, 'minimum_recorded_clearance_m'),
            'reactive_first_slowdown_elapsed_s': reactive.get(
                'first_slowdown_elapsed_s', ''),
            'oracle_first_slowdown_elapsed_s': oracle.get(
                'first_slowdown_elapsed_s', ''),
            'delta_oracle_minus_reactive_slowdown_time_s': _delta(
                oracle, reactive, 'first_slowdown_elapsed_s'),
            'reactive_stop_ratio': reactive.get('command_stop_ratio', ''),
            'oracle_stop_ratio': oracle.get('command_stop_ratio', ''),
            'reactive_dynamic_contact': reactive.get(
                'gazebo_robot_dynamic_contact', ''),
            'oracle_dynamic_contact': oracle.get(
                'gazebo_robot_dynamic_contact', ''),
        })
    write_csv(path, pair_rows)


def pair_group(run):
    """Return a method-independent key for a paired evidence run.

    The normal layout is ``S2_oncoming/reactive_run_01``.  Cost sweeps add an
    intermediate directory such as ``cost_010``.  Preserve that condition
    name while removing only the Reactive/Oracle prefix, so a pair can never
    accidentally combine different weights or repetitions.
    """
    run_name = re.sub(r'^(?:reactive|oracle)_', '', run.name)
    # The dated root ``cost_sweep_YYYYMMDD_NN`` also starts with ``cost_`` but
    # is not a parameter condition.  Keep only actual condition directories.
    parents = [part for part in run.parts[:-1]
               if re.fullmatch(r'(?:cost|horizon)_[0-9]+', part)]
    return '/'.join(parents + [run_name])


def _delta(left, right, key):
    try:
        return float(left[key]) - float(right[key])
    except (KeyError, TypeError, ValueError):
        return ''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--status', default='')
    parser.add_argument('--pairs-output', default='')
    args = parser.parse_args()
    root = Path(args.root)
    rows = run_rows(root, args.status)
    write_csv(Path(args.output), rows)
    pairs_output = Path(args.pairs_output) if args.pairs_output else \
        Path(args.output).with_name('gate5_paired_summary.csv')
    write_pairs(pairs_output, rows)
    print(f'wrote {len(rows)} run rows to {args.output}')
    print(f'wrote paired rows to {pairs_output}')


if __name__ == '__main__':
    main()
