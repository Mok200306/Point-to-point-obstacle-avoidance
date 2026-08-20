#!/usr/bin/env python3
"""Run one NavigateToPose trial and save timing, trajectory and map evidence.

The recorded path is transformed from odom into map using the TF available at
each odometry sample. The final RTAB-Map OccupancyGrid is drawn behind the
path, so each trial leaves a reviewable artifact without rosbag post-process.
"""

import argparse
import csv
import math
import os
import sys
import time

os.environ.setdefault('MPLCONFIGDIR', '/tmp/rtabmap_matplotlib')
os.makedirs(os.environ['MPLCONFIGDIR'], exist_ok=True)

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def quaternion_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def transform_xy(x, y, transform):
    rotation = transform.transform.rotation
    yaw = yaw_from_quaternion(rotation)
    translation = transform.transform.translation
    return (
        translation.x + math.cos(yaw) * x - math.sin(yaw) * y,
        translation.y + math.sin(yaw) * x + math.cos(yaw) * y,
    )


def map_array(message):
    if message is None or message.info.width == 0 or message.info.height == 0:
        return None
    values = np.asarray(message.data, dtype=np.int16)
    return values.reshape((message.info.height, message.info.width))


class NavigationTrial:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node('rtabmap_navigation_trial')
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(
            self.tf_buffer, self.node, spin_thread=False)
        self.action_client = ActionClient(
            self.node, NavigateToPose, 'navigate_to_pose')
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        costmap_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.odom_subscription = self.node.create_subscription(
            Odometry, '/odom', self.odom_callback, 50)
        self.map_subscription = self.node.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)
        self.costmap_subscription = self.node.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.costmap_callback,
            costmap_qos)
        self.path = []
        self.map_message = None
        self.costmap_message = None
        self.last_feedback_distance = None
        self.last_feedback_wall = 0.0
        self.wall_start_mono = None
        self.wall_end_mono = None
        self.wall_start_unix = None
        self.wall_end_unix = None
        self.sim_start = None
        self.sim_end = None
        self.status = None
        self.goal_handle = None

    def map_callback(self, message):
        self.map_message = message

    def costmap_callback(self, message):
        self.costmap_message = message

    def odom_callback(self, message):
        source_frame = message.header.frame_id or 'odom'
        stamp = Time.from_msg(message.header.stamp)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.args.frame, source_frame, stamp,
                timeout=Duration(seconds=0.05))
            x, y = transform_xy(
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                transform,
            )
            yaw = yaw_from_quaternion(message.pose.pose.orientation)
            yaw += yaw_from_quaternion(transform.transform.rotation)
        except Exception:
            # Early startup can precede map->odom. Do not label fallback odom
            # coordinates as map data; omit them until TF is available.
            return

        sample = {
            'wall_time': time.time(),
            'wall_elapsed_s': (
                time.monotonic() - self.wall_start_mono
                if self.wall_start_mono is not None else 0.0),
            'sim_time': stamp_seconds(message.header.stamp),
            'x': x,
            'y': y,
            'yaw': math.atan2(math.sin(yaw), math.cos(yaw)),
        }
        if not self.path or math.hypot(
                x - self.path[-1]['x'], y - self.path[-1]['y']) >= 0.01:
            self.path.append(sample)

    def feedback_callback(self, feedback):
        distance = float(feedback.feedback.distance_remaining)
        now = time.monotonic()
        if (self.last_feedback_distance is None or
                now - self.last_feedback_wall >= 1.0 or
                abs(distance - self.last_feedback_distance) >= 0.25):
            self.node.get_logger().info(f'distance remaining: {distance:.2f} m')
            self.last_feedback_distance = distance
            self.last_feedback_wall = now

    def send_goal(self):
        if not self.action_client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError('navigate_to_pose action server unavailable')

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.args.frame
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.args.x
        goal.pose.pose.position.y = self.args.y
        goal.pose.pose.orientation.x, goal.pose.pose.orientation.y, \
            goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = \
            quaternion_from_yaw(self.args.yaw)

        self.wall_start_mono = time.monotonic()
        self.wall_start_unix = time.time()
        self.sim_start = stamp_seconds(goal.pose.header.stamp)
        send_future = self.action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self.node, send_future)
        self.goal_handle = send_future.result()
        if self.goal_handle is None or not self.goal_handle.accepted:
            raise RuntimeError('Nav2 rejected the goal')
        self.node.get_logger().info('Goal accepted; recording trajectory.')

        result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result()
        self.wall_end_mono = time.monotonic()
        self.wall_end_unix = time.time()
        self.sim_end = stamp_seconds(self.node.get_clock().now().to_msg())
        self.status = int(result.status) if result is not None else -1

    def run(self):
        try:
            self.send_goal()
        finally:
            if self.wall_end_mono is None:
                self.wall_end_mono = time.monotonic()
            if self.wall_end_unix is None:
                self.wall_end_unix = time.time()
            if self.sim_end is None:
                self.sim_end = stamp_seconds(self.node.get_clock().now().to_msg())

    def metrics(self):
        final = self.path[-1] if self.path else None
        goal_error = (
            math.hypot(final['x'] - self.args.x, final['y'] - self.args.y)
            if final is not None else None)
        wall_duration = self.wall_end_mono - self.wall_start_mono
        sim_duration = self.sim_end - self.sim_start
        return {
            'label': self.args.label,
            'goal_frame': self.args.frame,
            'goal_x_m': self.args.x,
            'goal_y_m': self.args.y,
            'goal_yaw_rad': self.args.yaw,
            'nav2_status': self.status,
            'succeeded': self.status == 4,
            'wall_start_unix_s': self.wall_start_unix,
            'wall_end_unix_s': self.wall_end_unix,
            'wall_duration_s': wall_duration,
            'simulation_start_s': self.sim_start,
            'simulation_end_s': self.sim_end,
            'simulation_duration_s': sim_duration,
            'samples': len(self.path),
            'final_x_m': final['x'] if final else None,
            'final_y_m': final['y'] if final else None,
            'final_xy_error_m': goal_error,
            'map_frame': self.args.frame,
            'map_received': self.map_message is not None,
            'global_costmap_received': self.costmap_message is not None,
        }

    def write_csv(self, path):
        with open(path, 'w', newline='', encoding='utf-8') as stream:
            fields = ['wall_time', 'wall_elapsed_s', 'sim_time', 'x', 'y', 'yaw']
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.path)

    @staticmethod
    def draw_grid(axis, message, alpha, label):
        values = map_array(message)
        if values is None:
            return False
        display = np.full(values.shape, 210, dtype=np.uint8)
        known_free = values >= 0
        display[known_free] = np.clip(255 - values[known_free], 0, 255)
        origin = message.info.origin.position
        extent = [
            origin.x,
            origin.x + message.info.width * message.info.resolution,
            origin.y,
            origin.y + message.info.height * message.info.resolution,
        ]
        axis.imshow(
            display, origin='lower', extent=extent, cmap='gray',
            vmin=0, vmax=255, interpolation='nearest', alpha=alpha,
            label=label)
        return True

    def write_plot(self, path):
        figure, axis = plt.subplots(figsize=(11, 8), constrained_layout=True)
        drew_map = self.draw_grid(axis, self.map_message, 0.88, 'RTAB-Map occupancy')
        if not drew_map:
            self.draw_grid(axis, self.costmap_message, 0.45, 'global costmap')

        if self.path:
            x_values = [sample['x'] for sample in self.path]
            y_values = [sample['y'] for sample in self.path]
            axis.plot(x_values, y_values, color='#d62728', linewidth=2.2,
                      label='robot trajectory', zorder=5)
            axis.scatter(x_values[0], y_values[0], color='#2ca02c', s=75,
                         marker='o', label='recorded start', zorder=6)
            axis.scatter(x_values[-1], y_values[-1], color='#1f77b4', s=75,
                         marker='x', label='recorded end', zorder=6)

        axis.scatter(self.args.x, self.args.y, facecolors='none',
                     edgecolors='#ff7f0e', s=150, linewidths=2,
                     marker='*', label='goal', zorder=7)
        axis.set_title(
            f'{self.args.label}: status={self.status}, '
            f'wall={self.wall_end_mono - self.wall_start_mono:.1f}s')
        axis.set_xlabel('x [m]')
        axis.set_ylabel('y [m]')
        axis.set_aspect('equal', adjustable='box')
        axis.grid(True, alpha=0.25)
        axis.legend(loc='best')
        figure.savefig(path, dpi=160)
        plt.close(figure)

    def write_metrics(self, path):
        metrics = self.metrics()
        with open(path, 'w', encoding='utf-8') as stream:
            for key, value in metrics.items():
                if isinstance(value, bool):
                    value = 'true' if value else 'false'
                elif value is None:
                    value = 'null'
                stream.write(f'{key}: {value}\n')
        return metrics


def main():
    parser = argparse.ArgumentParser(
        description='Run Nav2 and save trajectory.png, trajectory.csv and metrics.yaml.')
    parser.add_argument('--x', type=float, required=True, help='Goal X in map frame.')
    parser.add_argument('--y', type=float, required=True, help='Goal Y in map frame.')
    parser.add_argument('--yaw', type=float, default=0.0, help='Goal yaw in radians.')
    parser.add_argument('--frame', default='map', help='Goal and output frame.')
    parser.add_argument('--label', default='navigation_trial', help='Output folder name.')
    parser.add_argument('--output-dir', default='/workspaces/rtabmap_tb3_nav/results',
                        help='Parent directory for trial artifacts.')
    args = parser.parse_args()

    output_dir = os.path.join(args.output_dir, args.label)
    os.makedirs(output_dir, exist_ok=True)
    rclpy.init()
    trial = NavigationTrial(args)
    exit_code = 0
    try:
        trial.run()
        metrics = trial.write_metrics(os.path.join(output_dir, 'metrics.yaml'))
        trial.write_csv(os.path.join(output_dir, 'trajectory.csv'))
        trial.write_plot(os.path.join(output_dir, 'trajectory.png'))
        trial.node.get_logger().info(
            f"Trial {args.label}: status={metrics['nav2_status']} "
            f"wall={metrics['wall_duration_s']:.2f}s "
            f"sim={metrics['simulation_duration_s']:.2f}s "
            f"samples={metrics['samples']} "
            f"xy_error={metrics['final_xy_error_m']}")
        exit_code = 0 if metrics['succeeded'] else 5
    except Exception as error:
        trial.node.get_logger().error(str(error))
        exit_code = 2
    finally:
        trial.node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
