#!/usr/bin/env python3
"""Create a self-contained Gate 2 dynamic-scene evidence figure.

The left panel is a Gazebo-style top-down view with static collision boxes,
the robot ground-truth path, and the planned/actual dynamic-obstacle path.
The right panel aligns the robot-obstacle geometric clearance to simulation
time.  It intentionally uses only per-run CSV/SDF evidence and never exposes
the future schedule to Nav2.
"""

import argparse
import csv
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Polygon
import numpy as np
import yaml


def parse_pose(text):
    values = [float(value) for value in (text or '0 0 0 0 0 0').split()]
    values += [0.0] * (6 - len(values))
    return values[0], values[1], values[5]


def compose_pose(parent, child):
    px, py, pyaw = parent
    cx, cy, cyaw = child
    return (
        px + math.cos(pyaw) * cx - math.sin(pyaw) * cy,
        py + math.sin(pyaw) * cx + math.cos(pyaw) * cy,
        math.atan2(math.sin(pyaw + cyaw), math.cos(pyaw + cyaw)),
    )


def box_corners(box):
    x, y, yaw = box['x'], box['y'], box['yaw']
    hx, hy = box['size_x'] / 2.0, box['size_y'] / 2.0
    return [
        (x + math.cos(yaw) * lx - math.sin(yaw) * ly,
         y + math.sin(yaw) * lx + math.cos(yaw) * ly)
        for lx, ly in ((-hx, -hy), (-hx, hy), (hx, hy), (hx, -hy))
    ]


def load_boxes(world_file):
    root = ET.parse(world_file).getroot()
    world = root.find('world')
    boxes = []
    if world is None:
        return boxes
    for model in world.findall('model'):
        if model.findtext('static', 'false').strip().lower() not in (
                'true', '1', 'yes'):
            continue
        model_pose = parse_pose(model.findtext('pose'))
        for link in model.findall('link'):
            link_pose = parse_pose(link.findtext('pose'))
            for collision in link.findall('collision'):
                geometry = collision.find('geometry')
                if geometry is None or geometry.findtext('box/size') is None:
                    continue
                size = [float(value) for value in
                        geometry.findtext('box/size').split()]
                pose = compose_pose(
                    compose_pose(model_pose, link_pose),
                    parse_pose(collision.findtext('pose')))
                boxes.append({
                    'name': model.get('name', 'model'),
                    'x': pose[0], 'y': pose[1], 'yaw': pose[2],
                    'size_x': size[0], 'size_y': size[1],
                })
    return boxes


def color_for(name):
    if name.startswith('wall'):
        return '#697684'
    if name.startswith('barrier'):
        return '#c97928'
    if name.startswith('crate'):
        return '#3d9272'
    if name.startswith('pillar'):
        return '#565b66'
    return '#9aa4ad'


def read_rows(path):
    rows = []
    with Path(path).open(newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            def number(key):
                value = row.get(key, '')
                return float(value) if value not in ('', None) else math.nan
            rows.append({key: number(key) for key in (
                'sim_time_s', 'planned_x_m', 'planned_y_m', 'obstacle_x_m',
                'obstacle_y_m', 'robot_x_m', 'robot_y_m',
                'center_distance_m', 'robot_obstacle_clearance_m')})
    return rows


def finite(rows, x_key, y_key):
    return [
        (row[x_key], row[y_key]) for row in rows
        if math.isfinite(row[x_key]) and math.isfinite(row[y_key])]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', required=True,
                        help='Gate 2 run directory')
    parser.add_argument('--output', default='dynamic_trajectory_comparison.png')
    args = parser.parse_args()
    run = Path(args.run)
    scenario = yaml.safe_load((run / 'scenario.yaml').read_text(encoding='utf-8'))
    metrics = {}
    metrics_path = run / 'metrics.yaml'
    if metrics_path.exists():
        try:
            metrics = yaml.safe_load(metrics_path.read_text(encoding='utf-8')) or {}
        except yaml.YAMLError:
            # The first pre-fix collision runs had unescaped nested quotes.
            # The raw contacts file remains authoritative for those runs.
            for line in metrics_path.read_text(encoding='utf-8').splitlines():
                if ': ' in line:
                    key, value = line.split(': ', 1)
                    metrics[key] = value.strip("'\"")
    world_file = run / 'world.sdf'
    rows = read_rows(run / 'dynamic_groundtruth.csv')
    boxes = load_boxes(world_file) if world_file.exists() else []
    robot = finite(rows, 'robot_x_m', 'robot_y_m')
    planned = finite(rows, 'planned_x_m', 'planned_y_m')
    obstacle = finite(rows, 'obstacle_x_m', 'obstacle_y_m')
    actual = [
        (row['obstacle_x_m'], row['obstacle_y_m']) for row in rows
        if math.isfinite(row['obstacle_x_m']) and math.isfinite(row['obstacle_y_m'])
    ]
    times = np.asarray([row['sim_time_s'] for row in rows], dtype=float)
    clearance = np.asarray([
        row['robot_obstacle_clearance_m'] for row in rows], dtype=float)
    valid_clearance = np.isfinite(clearance)
    min_clearance = float(np.min(clearance[valid_clearance])) \
        if np.any(valid_clearance) else math.nan
    min_index = int(np.nanargmin(clearance)) if np.any(valid_clearance) else None

    points = []
    for box in boxes:
        points.extend(box_corners(box))
    points.extend(robot + planned + obstacle)
    if points:
        xs, ys = zip(*points)
        margin = 0.7
        limits = (min(xs) - margin, max(xs) + margin,
                  min(ys) - margin, max(ys) + margin)
    else:
        limits = (-10.5, 10.5, -7.5, 7.5)

    figure, axes = plt.subplots(1, 2, figsize=(18, 8), constrained_layout=True)
    axis = axes[0]
    axis.set_facecolor('#edf1f4')
    for box in boxes:
        axis.add_patch(Polygon(
            box_corners(box), closed=True, facecolor=color_for(box['name']),
            edgecolor='#263238', linewidth=0.8, alpha=0.82, zorder=1))
        if not box['name'].startswith('wall'):
            axis.text(box['x'], box['y'], box['name'], fontsize=5.5,
                      ha='center', va='center', zorder=2)

    if len(robot) > 1:
        axis.plot(*zip(*robot), color='#d62728', linewidth=2.1,
                  label='robot Gazebo ground truth', zorder=4)
    if len(planned) > 1:
        axis.plot(*zip(*planned), color='#1f77b4', linestyle='--',
                  linewidth=1.2, alpha=0.75, label='obstacle planned', zorder=3)
    if len(actual) > 1:
        axis.plot(*zip(*actual), color='#9467bd', linewidth=1.7,
                  alpha=0.9, label='obstacle Gazebo actual', zorder=4)
    if robot:
        axis.scatter(*robot[0], color='#2ca02c', s=70, label='robot start', zorder=6)
    if robot:
        axis.scatter(*robot[-1], color='#1f77b4', s=70, marker='x',
                      label='robot end', zorder=6)
    goal = (float(scenario['robot']['goal_x']),
            float(scenario['robot']['goal_y']))
    axis.scatter(*goal, facecolors='none', edgecolors='#ff7f0e', s=150,
                  marker='*', linewidths=2, label='goal', zorder=6)
    if min_index is not None:
        row = rows[min_index]
        axis.scatter(row['robot_x_m'], row['robot_y_m'], color='black',
                     marker='x', s=90, linewidths=2, label='minimum clearance',
                     zorder=7)
        axis.scatter(row['obstacle_x_m'], row['obstacle_y_m'], color='black',
                     marker='o', facecolors='none', s=120, linewidths=1.5,
                     zorder=7)
    axis.set_xlim(limits[0], limits[1])
    axis.set_ylim(limits[2], limits[3])
    axis.set_aspect('equal', adjustable='box')
    axis.set_xlabel('x [m]')
    axis.set_ylabel('y [m]')
    axis.grid(True, alpha=0.25)
    axis.set_title('Gazebo top-down: robot + dynamic obstacle')
    axis.legend(loc='upper right', fontsize=7)

    axis = axes[1]
    if np.any(valid_clearance):
        axis.plot(times[valid_clearance], clearance[valid_clearance],
                  color='#d62728', linewidth=1.7, label='geometric clearance')
        axis.axhline(0.0, color='black', linewidth=0.9, linestyle='--',
                     label='contact / overlap')
        axis.axhline(0.5, color='#ff7f0e', linewidth=0.9, linestyle=':',
                     label='0.50 m reference')
    axis.set_xlabel('simulation time [s]')
    axis.set_ylabel('robot-obstacle boundary clearance [m]')
    axis.grid(True, alpha=0.25)
    axis.set_title('Time-aligned dynamic interaction')
    axis.legend(loc='best', fontsize=8)
    success = metrics.get('succeeded', 'unknown')
    status = metrics.get('nav2_status', 'unknown')
    scenario_id = scenario.get('scenario_id', run.name)
    figure.suptitle(
        f'{scenario_id} | {run.name} | status={status} | success={success} | '
        f'min dynamic clearance={min_clearance:.3f} m', fontsize=14)
    figure.savefig(run / args.output, dpi=170)
    plt.close(figure)
    print(run / args.output)


if __name__ == '__main__':
    main()
