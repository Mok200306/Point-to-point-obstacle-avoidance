#!/usr/bin/env python3
"""Plot Gate 3 schedule positions and their rasterized Oracle layers."""

import argparse
import math
from pathlib import Path
import sys

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml

SCRIPT_ROOT = Path(__file__).resolve().parents[3]
PUBLISHER_ROOT = SCRIPT_ROOT / 'src' / 'oracle_prediction_publisher'
sys.path.insert(0, str(PUBLISHER_ROOT))

from oracle_prediction_publisher.grid import (  # noqa: E402
    load_costmap_origin,
    load_costmap_grid_spec,
    rasterize_rotated_box,
)
from oracle_prediction_publisher.trajectory import WaypointSchedule  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', required=True)
    parser.add_argument('--nav2-params', default='experiments/oracle_mppi/configs/nav2_mppi_reactive_10hz_params.yaml')
    parser.add_argument(
        '--costmap-name', default='global_costmap',
        help='Use global_costmap for a fixed full-scene plot. For a local '
             'rolling grid, supply an origin explicitly.')
    parser.add_argument('--output', default='experiments/oracle_mppi/gate3/oracle_grid_layers.png')
    parser.add_argument('--origin-x', type=float, default=None)
    parser.add_argument('--origin-y', type=float, default=None)
    parser.add_argument('--difficulty', default='medium')
    parser.add_argument('--dt', type=float, default=0.5)
    parser.add_argument('--horizon-s', type=float, default=1.5)
    args = parser.parse_args()

    scenario_path = Path(args.scenario)
    scenario = yaml.safe_load(scenario_path.read_text(encoding='utf-8'))
    schedule = WaypointSchedule(scenario, args.difficulty)
    grid = load_costmap_grid_spec(args.nav2_params, args.costmap_name)
    configured_origin_x, configured_origin_y = load_costmap_origin(
        args.nav2_params, args.costmap_name)
    origin_x = (configured_origin_x if args.origin_x is None else args.origin_x)
    origin_y = (configured_origin_y if args.origin_y is None else args.origin_y)
    steps = int(round(args.horizon_s / args.dt)) + 1

    figure, axes = plt.subplots(1, steps, figsize=(4.2 * steps, 4.5), squeeze=False)
    axes = axes[0]
    for index, axis in enumerate(axes):
        tau = index * args.dt
        x, y, yaw = schedule.pose_at_elapsed(tau)
        data = rasterize_rotated_box(
            grid, origin_x, origin_y, x, y, yaw,
            schedule.obstacle_half_size_m, schedule.obstacle_half_size_m,
            conservative_cell=True)
        array = np.asarray(data, dtype=float).reshape(grid.height, grid.width)
        axis.imshow(
            array, origin='lower', interpolation='nearest', cmap='Reds',
            extent=(origin_x, origin_x + grid.size_x_m,
                    origin_y, origin_y + grid.size_y_m),
            vmin=0.0, vmax=1.0)
        axis.scatter([x], [y], marker='x', color='black', s=70,
                     label='schedule center')
        axis.set_title(f'tau={tau:.1f} s')
        axis.set_xlabel('x [m]')
        axis.set_ylabel('y [m]')
        axis.set_aspect('equal', adjustable='box')
        axis.grid(alpha=0.2)
    figure.suptitle(
        f"{scenario['scenario_id']} Oracle layers | frame=odom | "
        f"{grid.width}x{grid.height}@{grid.resolution:.2f} m")
    figure.tight_layout()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180)
    plt.close(figure)
    print(output)


if __name__ == '__main__':
    main()
