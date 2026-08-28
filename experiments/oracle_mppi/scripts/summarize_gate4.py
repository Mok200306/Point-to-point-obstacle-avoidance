#!/usr/bin/env python3
"""Summarize Gate 4 Reactive versus zero-risk Oracle static regressions."""

import argparse
import csv
from pathlib import Path

import yaml


def load(path):
    if not path.exists():
        return {}
    value = yaml.safe_load(path.read_text(encoding='utf-8'))
    return value if isinstance(value, dict) else {}


def as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() == 'true'
    return bool(value)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--root', default='experiments/oracle_mppi/gate4/zero_risk')
    parser.add_argument(
        '--output', default='experiments/oracle_mppi/gate4/zero_risk_summary.csv')
    args = parser.parse_args()

    root = Path(args.root)
    rows = []
    for run in sorted(root.glob('*/')):
        metrics = load(run / 'metrics.yaml')
        experiment = load(run / 'experiment.yaml')
        if not metrics and not experiment:
            continue
        source = {**experiment, **metrics}
        oracle = run.name.startswith('oracle_') or as_bool(
            source.get('oracle_enabled', False))
        rows.append({
            'run': run.name,
            'method': 'oracle_zero_risk' if oracle else 'reactive',
            'run_path': str(run),
            'git_commit': source.get('git_commit', ''),
            'profile': source.get('profile', ''),
            'nav2_params': source.get('nav2_params', ''),
            'world_file': source.get('world_file', ''),
            'start_x_m': source.get('start_x_m', ''),
            'start_y_m': source.get('start_y_m', ''),
            'goal_x_m': source.get('goal_x_m', source.get('goal_x', '')),
            'goal_y_m': source.get('goal_y_m', source.get('goal_y', '')),
            'succeeded': source.get('succeeded', ''),
            'nav2_status': source.get('nav2_status', ''),
            'simulation_duration_s': source.get('simulation_duration_s', ''),
            'wall_duration_s': source.get('wall_duration_s', ''),
            'trajectory_length_m': source.get(
                'gazebo_trajectory_length_m',
                source.get('trajectory_length_m', '')),
            'final_xy_error_m': source.get('final_xy_error_m', ''),
            'gazebo_non_ground_contact': source.get(
                'gazebo_non_ground_contact', ''),
            'gazebo_contact_messages': source.get(
                'gazebo_contact_messages', ''),
            'oracle_message_validation_exit': source.get(
                'oracle_message_validation_exit', 0 if not oracle else ''),
            'oracle_active_log_lines': source.get(
                'oracle_active_log_lines', 0 if not oracle else ''),
            'oracle_stale_log_lines': source.get(
                'oracle_stale_log_lines', 0 if not oracle else ''),
            'control_p95_period_s': source.get('control_p95_period_s', ''),
            'evidence_complete': all((run / filename).exists() for filename in (
                'metrics.yaml', 'experiment.yaml', 'trajectory.csv',
                'gazebo_trajectory.csv', 'trajectory_comparison.png')),
        })

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0]) if rows else [
        'run', 'method', 'run_path', 'git_commit', 'succeeded']
    with output.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)
    print(f'wrote {len(rows)} rows to {output}')

    expected = {'reactive': 3, 'oracle_zero_risk': 3}
    counts = {key: sum(row['method'] == key for row in rows)
              for key in expected}
    complete = all(row['evidence_complete'] for row in rows)
    successes = all(as_bool(row['succeeded']) for row in rows)
    print(f"counts={counts} complete={complete} all_success={successes}")
    return 0 if counts == expected and complete and successes else 1


if __name__ == '__main__':
    raise SystemExit(main())
