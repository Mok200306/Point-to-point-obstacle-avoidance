#!/usr/bin/env python3
"""Summarize all Gate 2 smoke runs without deleting failures."""

import argparse
import csv
import math
import re
from pathlib import Path

import yaml


def fallback_metrics(path):
    result = {}
    for line in path.read_text(encoding='utf-8').splitlines():
        if ': ' not in line:
            continue
        key, value = line.split(': ', 1)
        value = value.strip().strip('"\'')
        if value == 'true':
            value = True
        elif value == 'false':
            value = False
        else:
            try:
                value = float(value) if any(c in value for c in '.eE') else int(value)
            except ValueError:
                pass
        result[key] = value
    return result


def load_yaml_or_fallback(path):
    try:
        return yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    except yaml.YAMLError:
        return fallback_metrics(path)


def value(data, key, default=''):
    return data.get(key, default)


def infer_failure_reason(run, metrics, experiment, scenario):
    """Return a short audit label without hiding incomplete runs."""
    if not metrics:
        if experiment.get('startup_failure') is True:
            return 'Nav2 startup/lifecycle readiness failure'
        try:
            if int(experiment.get('spawn_exit_code', 0)) != 0:
                return 'dynamic obstacle spawn failure'
        except (TypeError, ValueError):
            pass
        if experiment.get('dynamic_controller_startup_failure') is True:
            return 'dynamic controller startup failure'
        runner_log = run / 'runner.log'
        if runner_log.exists():
            text = runner_log.read_text(encoding='utf-8', errors='replace')
            if 'Nav2 startup failed' in text:
                return 'Nav2 startup/lifecycle readiness failure'
            if 'Dynamic obstacle spawn failed' in text:
                return 'dynamic obstacle spawn failure'
            if 'Dynamic controller failed startup' in text:
                return 'dynamic controller startup failure'
        return 'incomplete run: metrics.yaml missing'

    if value(metrics, 'gazebo_robot_dynamic_contact', False) is True:
        return 'robot-dynamic physical collision'
    if value(metrics, 'gazebo_non_ground_contact', False) is True:
        return 'non-ground physical contact'
    if not (run / 'dynamic_groundtruth.csv').exists() or \
            not (run / 'dynamic_summary.yaml').exists():
        return 'incomplete run: dynamic evidence missing'
    if value(metrics, 'succeeded', False) is not True:
        status = value(metrics, 'nav2_status', '')
        return f'Nav2 trial failure (status={status})'
    return ''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', default='experiments/oracle_mppi/gate2')
    parser.add_argument('--output', default='experiments/oracle_mppi/gate2/summary.csv')
    args = parser.parse_args()
    root = Path(args.root)
    rows = []
    # Include every evidence directory, including startup/spawn failures that
    # do not have metrics.yaml.  The scenario.yaml is the run-directory marker
    # and the recursive pattern intentionally keeps every failure in the CSV.
    for scenario_path in sorted(root.glob('S*/**/scenario.yaml')):
        run = scenario_path.parent
        metrics_path = run / 'metrics.yaml'
        experiment_path = run / 'experiment.yaml'
        metrics = load_yaml_or_fallback(metrics_path) \
            if metrics_path.exists() else {}
        experiment = load_yaml_or_fallback(experiment_path) \
            if experiment_path.exists() else {}
        scenario = load_yaml_or_fallback(run / 'scenario.yaml')
        dynamic = load_yaml_or_fallback(run / 'dynamic_summary.yaml') \
            if (run / 'dynamic_summary.yaml').exists() else {}
        contact_pairs = str(value(metrics, 'gazebo_contact_pairs',
                                  value(experiment, 'gazebo_contact_pairs', '(none)')))
        dynamic_pair = str(value(
            metrics, 'gazebo_robot_dynamic_contact_pairs',
            value(experiment, 'gazebo_robot_dynamic_contact_pairs', '(none)')))
        source = metrics or experiment
        failure_reason = infer_failure_reason(
            run, metrics, experiment, scenario)
        rows.append({
            'scenario_id': value(source, 'scenario_id', value(scenario, 'scenario_id', '')),
            'run': run.name,
            'run_path': str(run),
            'run_class': run.name.rsplit('_', 1)[0]
                if '_' in run.name else run.name,
            'git_commit': value(source, 'git_commit', ''),
            'nav2_status': value(metrics, 'nav2_status', ''),
            'succeeded': value(metrics, 'succeeded', ''),
            'wrapper_trial_exit': value(metrics, 'wrapper_trial_exit',
                                        value(experiment, 'trial_exit_code', '')),
            'wall_duration_s': value(metrics, 'wall_duration_s', ''),
            'simulation_duration_s': value(metrics, 'simulation_duration_s', ''),
            'trajectory_length_m': value(metrics, 'gazebo_trajectory_length_m',
                                         value(metrics, 'trajectory_length_m', '')),
            'final_xy_error_m': value(metrics, 'final_xy_error_m', ''),
            'dynamic_min_clearance_m': value(
                metrics, 'minimum_robot_obstacle_clearance_m',
                value(dynamic, 'minimum_robot_obstacle_clearance_m', '')),
            'script_to_gazebo_error_m': value(
                metrics, 'maximum_script_to_gazebo_position_error_m',
                value(dynamic, 'maximum_script_to_gazebo_position_error_m', '')),
            'dynamic_service_failures': value(
                metrics, 'dynamic_service_failures', value(dynamic, 'service_failures', '')),
            'gazebo_non_ground_contact': value(metrics, 'gazebo_non_ground_contact', ''),
            'gazebo_robot_dynamic_contact': value(
                metrics, 'gazebo_robot_dynamic_contact', ''),
            'gazebo_contact_pairs': contact_pairs,
            'gazebo_robot_dynamic_contact_pairs': dynamic_pair,
            'dynamic_plot': str(run / 'dynamic_trajectory_comparison.png')
                if (run / 'dynamic_trajectory_comparison.png').exists() else '',
            'evidence_complete': bool(metrics and
                                      (run / 'dynamic_groundtruth.csv').exists() and
                                      (run / 'dynamic_summary.yaml').exists()),
            'failure_reason': failure_reason,
        })
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if rows:
        with output.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]),
                                    lineterminator='\n')
            writer.writeheader()
            writer.writerows(rows)
    else:
        output.write_text('', encoding='utf-8')
    print(f'wrote {len(rows)} rows to {output}')


if __name__ == '__main__':
    main()
