#!/usr/bin/env python3
"""Offline Gate 3 alignment tests for all deterministic dynamic schedules.

The test uses the same schedule interpolation as the Gate 2 controller and
the same local-costmap dimensions/resolution as the frozen MPPI YAML.  It
checks that the center of the occupied raster is within one grid cell of the
scheduled obstacle center at tau=0, 0.5, 1.0 and 1.5 seconds.  It does not
start Gazebo and does not publish any information to Nav2.
"""

import argparse
import csv
import math
from pathlib import Path
import sys

import yaml

SCRIPT_ROOT = Path(__file__).resolve().parents[3]
PUBLISHER_ROOT = SCRIPT_ROOT / 'src' / 'oracle_prediction_publisher'
sys.path.insert(0, str(PUBLISHER_ROOT))

from oracle_prediction_publisher.grid import (  # noqa: E402
    GridSpec,
    load_costmap_origin,
    load_costmap_grid_spec,
    occupied_cell_centroid,
    rasterize_rotated_box,
)
from oracle_prediction_publisher.trajectory import WaypointSchedule  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--scenario-dir',
        default='experiments/oracle_mppi/configs/scenarios')
    parser.add_argument(
        '--nav2-params',
        default='experiments/oracle_mppi/configs/nav2_mppi_reactive_10hz_params.yaml')
    parser.add_argument(
        '--costmap-name', default='global_costmap',
        help='Use the fixed global envelope for schedule alignment; the live '
             'publisher still uses its configured rolling local grid.')
    parser.add_argument(
        '--output',
        default='experiments/oracle_mppi/gate3/oracle_grid_alignment_test.csv')
    parser.add_argument('--difficulty', default='medium')
    parser.add_argument(
        '--origin-x', type=float, default=None,
        help='Override the origin read from the selected costmap parameters.')
    parser.add_argument(
        '--origin-y', type=float, default=None,
        help='Override the origin read from the selected costmap parameters.')
    parser.add_argument(
        '--origin-mode', choices=('configured', 'schedule_center'),
        default='configured',
        help='Use the configured/fixed origin, or recenter the selected '
             'rolling grid on the scheduled obstacle for every test time.')
    parser.add_argument('--horizon-s', type=float, default=1.5)
    parser.add_argument('--dt', type=float, default=0.1)
    parser.add_argument(
        '--taus', default='0.0,0.5,1.0,1.5',
        help='Comma-separated elapsed times to test. The default is the four '
             'minimum Gate 3 checkpoints.')
    args = parser.parse_args()

    grid = load_costmap_grid_spec(args.nav2_params, args.costmap_name)
    configured_origin_x, configured_origin_y = load_costmap_origin(
        args.nav2_params, args.costmap_name)
    fixed_origin_x = (
        configured_origin_x if args.origin_x is None else args.origin_x)
    fixed_origin_y = (
        configured_origin_y if args.origin_y is None else args.origin_y)
    taus = [float(value.strip()) for value in args.taus.split(',')
            if value.strip()]
    if not taus or any(tau < 0.0 for tau in taus):
        raise ValueError('--taus must contain at least one non-negative time')
    if args.dt <= 0.0:
        raise ValueError('--dt must be positive')

    scenario_paths = sorted(Path(args.scenario_dir).glob('s*.yaml'))
    rows = []
    max_error = 0.0
    all_pass = True
    for scenario_path in scenario_paths:
        scenario = yaml.safe_load(scenario_path.read_text(encoding='utf-8'))
        schedule = WaypointSchedule(scenario, args.difficulty)
        for tau in taus:
            x, y, yaw = schedule.pose_at_elapsed(tau)
            if args.origin_mode == 'schedule_center':
                origin_x = x - grid.size_x_m / 2.0
                origin_y = y - grid.size_y_m / 2.0
            else:
                origin_x, origin_y = fixed_origin_x, fixed_origin_y
            raster = rasterize_rotated_box(
                grid, origin_x, origin_y, x, y, yaw,
                schedule.obstacle_half_size_m,
                schedule.obstacle_half_size_m,
                padding_m=0.0,
                conservative_cell=True)
            cx, cy, occupied = occupied_cell_centroid(
                grid, origin_x, origin_y, raster)
            error = math.hypot(cx - x, cy - y) if occupied else math.inf
            threshold = grid.resolution * math.sqrt(2.0)
            passed = bool(occupied and error <= threshold + 1e-9)
            max_error = max(max_error, error)
            all_pass = all_pass and passed
            rows.append({
                'scenario_id': scenario['scenario_id'],
                'difficulty': args.difficulty,
                'tau_s': tau,
                'frame_id': 'odom',
                'resolution_m': grid.resolution,
                'width_cells': grid.width,
                'height_cells': grid.height,
                'origin_x_m': origin_x,
                'origin_y_m': origin_y,
                'scheduled_x_m': x,
                'scheduled_y_m': y,
                'raster_centroid_x_m': cx,
                'raster_centroid_y_m': cy,
                'centroid_error_m': error,
                'one_cell_diagonal_threshold_m': threshold,
                'occupied_cells': occupied,
                'passed': passed,
                'source': 'deterministic waypoint schedule; not velocity extrapolation',
            })

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0]) if rows else [
        'scenario_id', 'difficulty', 'tau_s', 'passed']
    with output.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    print(f'wrote {len(rows)} alignment rows to {output}')
    print(f'grid={grid.width}x{grid.height} resolution={grid.resolution:.3f} m')
    print(f'max_centroid_error_m={max_error:.6f}')
    print(f'passed={all_pass}')
    return 0 if all_pass else 1


if __name__ == '__main__':
    raise SystemExit(main())
