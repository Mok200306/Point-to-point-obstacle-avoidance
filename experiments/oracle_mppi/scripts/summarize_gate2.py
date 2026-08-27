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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', default='experiments/oracle_mppi/gate2')
    parser.add_argument('--output', default='experiments/oracle_mppi/gate2/summary.csv')
    args = parser.parse_args()
    root = Path(args.root)
    rows = []
    # Include both the exploratory smoke runs and the later formal medium
    # runs.  The recursive pattern intentionally keeps every evidence
    # directory; no failed run is filtered out.
    for metrics_path in sorted(root.glob('S*/**/metrics.yaml')):
        run = metrics_path.parent
        metrics = load_yaml_or_fallback(metrics_path)
        scenario = load_yaml_or_fallback(run / 'scenario.yaml')
        dynamic = load_yaml_or_fallback(run / 'dynamic_summary.yaml') \
            if (run / 'dynamic_summary.yaml').exists() else {}
        contact_pairs = str(value(metrics, 'gazebo_contact_pairs', '(none)'))
        dynamic_pair = str(value(metrics, 'gazebo_robot_dynamic_contact_pairs', '(none)'))
        rows.append({
            'scenario_id': value(metrics, 'scenario_id', value(scenario, 'scenario_id', '')),
            'run': run.name,
            'run_path': str(run),
            'nav2_status': value(metrics, 'nav2_status', ''),
            'succeeded': value(metrics, 'succeeded', ''),
            'wrapper_trial_exit': value(metrics, 'wrapper_trial_exit', ''),
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
        })
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if rows:
        with output.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    else:
        output.write_text('', encoding='utf-8')
    print(f'wrote {len(rows)} rows to {output}')


if __name__ == '__main__':
    main()
