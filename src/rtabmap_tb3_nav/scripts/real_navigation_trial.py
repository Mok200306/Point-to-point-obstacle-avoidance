#!/usr/bin/env python3
"""Run and record one physical-robot NavigateToPose trial.

The script is intentionally a recorder and an action client only.  It sends a
navigation goal, never creates a Twist publisher, and never calls the WATER
TCP API.  The physical chassis must already be connected to
``/cmd_vel_safe`` through its reviewed driver.
"""

import argparse
import csv
import math
import os
import shutil
import subprocess
import sys
import time

os.environ.setdefault('MPLCONFIGDIR', '/tmp/rtabmap_matplotlib')
os.makedirs(os.environ['MPLCONFIGDIR'], exist_ok=True)

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from tf2_ros import Buffer, TransformListener


def quaternion_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def transform_xy(x, y, transform):
    yaw = yaw_from_quaternion(transform.transform.rotation)
    translation = transform.transform.translation
    return (
        translation.x + math.cos(yaw) * x - math.sin(yaw) * y,
        translation.y + math.sin(yaw) * x + math.cos(yaw) * y,
        yaw,
    )


def path_length(samples):
    return sum(
        math.hypot(current['x'] - previous['x'], current['y'] - previous['y'])
        for previous, current in zip(samples, samples[1:]))


def map_array(message):
    if message is None or message.info.width == 0 or message.info.height == 0:
        return None
    values = np.asarray(message.data, dtype=np.int16)
    return values.reshape((message.info.height, message.info.width))


def git_commit():
    try:
        return subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], stderr=subprocess.DEVNULL,
            text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        return 'unknown'


class RealNavigationTrial:
    """Record a real navigation goal without commanding the base directly."""

    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node(
            'real_navigation_trial',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, False)])
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(
            self.tf_buffer, self.node, spin_thread=False)
        self.action_client = ActionClient(
            self.node, NavigateToPose, 'navigate_to_pose')

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        global_costmap_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        local_costmap_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        path_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        self.map_message = None
        self.global_costmap_message = None
        self.local_costmap_message = None
        self.planned_path = []
        self.plan_frame = None
        self.plan_transform_failures = 0
        self.path = []
        self.command_samples = []
        self.message_counts = {}
        self.tf_lookup_failures = 0
        self.wall_start_mono = None
        self.wall_end_mono = None
        self.wall_start_unix = None
        self.wall_end_unix = None
        self.status = None
        self.goal_handle = None
        self.timed_out = False
        self.failure_reason = None
        self.runtime_snapshot_files = []

        self.node.create_subscription(
            Odometry, args.odom_topic, self.odom_callback,
            qos_profile_sensor_data)
        self.node.create_subscription(
            OccupancyGrid, args.map_topic,
            lambda message: setattr(self, 'map_message', message), map_qos)
        self.node.create_subscription(
            OccupancyGrid, args.global_costmap_topic,
            lambda message: setattr(self, 'global_costmap_message', message),
            global_costmap_qos)
        self.node.create_subscription(
            OccupancyGrid, args.local_costmap_topic,
            lambda message: setattr(self, 'local_costmap_message', message),
            local_costmap_qos)
        self.node.create_subscription(
            Path, args.plan_topic, self.plan_callback, path_qos)
        self.node.create_subscription(
            Twist, args.cmd_vel_topic,
            lambda message: self.command_callback(args.cmd_vel_topic, message),
            20)
        self.node.create_subscription(
            Twist, args.cmd_vel_safe_topic,
            lambda message: self.command_callback(
                args.cmd_vel_safe_topic, message), 20)
        self.node.create_subscription(
            Image, args.color_image_topic,
            lambda _message: self.count_message(args.color_image_topic),
            qos_profile_sensor_data)
        self.node.create_subscription(
            CameraInfo, args.color_info_topic,
            lambda _message: self.count_message(args.color_info_topic),
            qos_profile_sensor_data)
        self.node.create_subscription(
            Image, args.depth_image_topic,
            lambda _message: self.count_message(args.depth_image_topic),
            qos_profile_sensor_data)
        self.node.create_subscription(
            CameraInfo, args.depth_info_topic,
            lambda _message: self.count_message(args.depth_info_topic),
            qos_profile_sensor_data)
        self.node.create_subscription(
            PointCloud2, args.cloud_topic,
            lambda _message: self.count_message(args.cloud_topic),
            qos_profile_sensor_data)
        self.node.create_subscription(
            PointCloud2, args.obstacles_topic,
            lambda _message: self.count_message(args.obstacles_topic),
            qos_profile_sensor_data)
        self.node.create_subscription(
            PointCloud2, args.ground_topic,
            lambda _message: self.count_message(args.ground_topic),
            qos_profile_sensor_data)
        self.node.create_subscription(
            Imu, args.imu_topic,
            lambda _message: self.count_message(args.imu_topic),
            qos_profile_sensor_data)
        goal_line_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # This is a visualization/evidence request only; it is not a velocity
        # or route command and Nav2 does not consume it as a goal.
        self.goal_line_publisher = self.node.create_publisher(
            PoseStamped, '/goal_line_request', goal_line_qos)

    def count_message(self, topic):
        self.message_counts[topic] = self.message_counts.get(topic, 0) + 1

    def plan_callback(self, message):
        """Keep the latest Nav2 global plan for the evidence plot and CSV."""
        self.plan_frame = message.header.frame_id or self.args.frame
        self.planned_path = []
        for pose in message.poses:
            x = float(pose.pose.position.x)
            y = float(pose.pose.position.y)
            yaw = yaw_from_quaternion(pose.pose.orientation)
            if self.plan_frame != self.args.frame:
                try:
                    stamp = Time.from_msg(
                        pose.header.stamp if pose.header.stamp.sec or
                        pose.header.stamp.nanosec else message.header.stamp)
                    transform = self.tf_buffer.lookup_transform(
                        self.args.frame, self.plan_frame, stamp,
                        timeout=Duration(seconds=0.05))
                    x, y, transform_yaw = transform_xy(x, y, transform)
                    yaw += transform_yaw
                except Exception:
                    self.plan_transform_failures += 1
                    continue
            self.planned_path.append({
                'x': x,
                'y': y,
                'yaw': math.atan2(math.sin(yaw), math.cos(yaw)),
            })

    def command_callback(self, topic, message):
        if self.wall_start_mono is None:
            return
        self.command_samples.append({
            'wall_time': time.time(),
            'wall_elapsed_s': time.monotonic() - self.wall_start_mono,
            'topic': topic,
            'linear_x_mps': float(message.linear.x),
            'linear_y_mps': float(message.linear.y),
            'angular_z_radps': float(message.angular.z),
        })

    def odom_callback(self, message):
        if self.wall_start_mono is None:
            return
        source_frame = message.header.frame_id or self.args.odom_frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.args.frame, source_frame,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=0.05))
            x, y, tf_yaw = transform_xy(
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                transform)
        except Exception:
            self.tf_lookup_failures += 1
            return
        yaw = yaw_from_quaternion(message.pose.pose.orientation) + tf_yaw
        sample = {
            'wall_time': time.time(),
            'wall_elapsed_s': time.monotonic() - self.wall_start_mono,
            'odom_time_s': float(message.header.stamp.sec) +
            float(message.header.stamp.nanosec) * 1.0e-9,
            'x': x,
            'y': y,
            'yaw': math.atan2(math.sin(yaw), math.cos(yaw)),
        }
        if not self.path or math.hypot(
                x - self.path[-1]['x'], y - self.path[-1]['y']) >= 0.01:
            self.path.append(sample)

    def settle_before_goal(self):
        duration = max(float(self.args.settle_seconds), 0.0)
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.10)

    def send_goal(self):
        if not self.action_client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError('navigate_to_pose action server unavailable')
        self.settle_before_goal()

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.args.frame
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.args.x
        goal.pose.pose.position.y = self.args.y
        (goal.pose.pose.orientation.x, goal.pose.pose.orientation.y,
         goal.pose.pose.orientation.z, goal.pose.pose.orientation.w) = \
            quaternion_from_yaw(self.args.yaw)
        for _ in range(3):
            self.goal_line_publisher.publish(goal.pose)
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.wall_start_mono = time.monotonic()
        self.wall_start_unix = time.time()
        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future)
        self.goal_handle = send_future.result()
        if self.goal_handle is None or not self.goal_handle.accepted:
            raise RuntimeError('Nav2 rejected the goal')
        result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self.node, result_future, timeout_sec=self.args.timeout_seconds)
        if not result_future.done():
            self.timed_out = True
            self.failure_reason = (
                f'navigation action exceeded {self.args.timeout_seconds:.1f}s; '
                'goal cancellation requested')
            cancel_future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(
                self.node, cancel_future, timeout_sec=5.0)
            rclpy.spin_until_future_complete(
                self.node, result_future, timeout_sec=5.0)
        result = result_future.result() if result_future.done() else None
        self.wall_end_mono = time.monotonic()
        self.wall_end_unix = time.time()
        self.status = int(result.status) if result is not None else 5

    def run(self):
        try:
            self.send_goal()
        except Exception as error:
            self.failure_reason = str(error)
            raise
        finally:
            if self.wall_start_mono is None:
                self.wall_start_mono = time.monotonic()
                self.wall_start_unix = time.time()
            if self.wall_end_mono is None:
                self.wall_end_mono = time.monotonic()
                self.wall_end_unix = time.time()

    def metrics(self):
        final = self.path[-1] if self.path else None
        return {
            'label': self.args.label,
            'platform': 'WATER II-S',
            'real_robot': True,
            'profile': self.args.profile,
            'goal_frame': self.args.frame,
            'goal_x_m': self.args.x,
            'goal_y_m': self.args.y,
            'goal_yaw_rad': self.args.yaw,
            'nav2_status': self.status,
            'succeeded': self.status == 4,
            'timed_out': self.timed_out,
            'failure_reason': self.failure_reason,
            'wall_start_unix_s': self.wall_start_unix,
            'wall_end_unix_s': self.wall_end_unix,
            'navigation_wall_duration_s': self.wall_end_mono - self.wall_start_mono,
            'samples': len(self.path),
            'trajectory_length_m': path_length(self.path),
            'plan_frame': self.plan_frame,
            'planned_path_samples': len(self.planned_path),
            'planned_path_length_m': path_length(self.planned_path),
            'plan_transform_failures': self.plan_transform_failures,
            'final_x_m': final['x'] if final else None,
            'final_y_m': final['y'] if final else None,
            'final_xy_error_m': (
                math.hypot(final['x'] - self.args.x, final['y'] - self.args.y)
                if final else None),
            'map_received': self.map_message is not None,
            'global_costmap_received': self.global_costmap_message is not None,
            'local_costmap_received': self.local_costmap_message is not None,
            'tf_lookup_failures': self.tf_lookup_failures,
            'message_counts': self.message_counts,
            'minimum_clearance_m': None,
            'collision': None,
            'collision_evidence': 'physical contact/bumper or video/rosbag required',
            'gazebo_contacts_available': False,
            'runtime_snapshot_dir': self.args.runtime_snapshot_dir or None,
            'runtime_snapshot_files': self.runtime_snapshot_files,
            'git_commit': git_commit(),
        }

    @staticmethod
    def write_csv(path, samples):
        fields = ['wall_time', 'wall_elapsed_s', 'odom_time_s', 'x', 'y', 'yaw']
        with open(path, 'w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(samples)

    def write_commands_csv(self, path):
        fields = [
            'wall_time', 'wall_elapsed_s', 'topic', 'linear_x_mps',
            'linear_y_mps', 'angular_z_radps']
        with open(path, 'w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.command_samples)

    @staticmethod
    def write_planned_path_csv(path, samples):
        fields = ['x', 'y', 'yaw']
        with open(path, 'w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(samples)

    def copy_runtime_snapshots(self, output_dir):
        """Copy effective launch snapshots into this trial's evidence folder."""
        source_dir = self.args.runtime_snapshot_dir.strip()
        candidates = (
            '导航参数.yaml',
            '碰撞监视参数.yaml',
            '相机参数.yaml',
            '运行时元数据.yaml',
        )
        copied = []
        for name in candidates:
            destination = os.path.join(output_dir, name)
            source = os.path.join(source_dir, name) if source_dir else destination
            if not os.path.isfile(source):
                continue
            if os.path.realpath(source) != os.path.realpath(destination):
                shutil.copy2(source, destination)
            copied.append(name)
        self.runtime_snapshot_files = copied

    @staticmethod
    def draw_grid(axis, message, alpha, label):
        values = map_array(message)
        if values is None:
            return False
        display = np.full(values.shape, 210, dtype=np.uint8)
        known = values >= 0
        display[known] = np.clip(
            255 - values[known].astype(np.float32) * 2.55, 0, 255)
        origin = message.info.origin.position
        extent = [
            origin.x,
            origin.x + message.info.width * message.info.resolution,
            origin.y,
            origin.y + message.info.height * message.info.resolution]
        axis.imshow(
            display, origin='lower', extent=extent, cmap='gray',
            vmin=0, vmax=255, interpolation='nearest', alpha=alpha,
            label=label)
        return True

    def draw_trajectory(self, axis):
        if self.map_message is not None:
            self.draw_grid(axis, self.map_message, 0.86, 'RTAB-Map /map')
        if self.global_costmap_message is not None:
            self.draw_grid(axis, self.global_costmap_message, 0.30,
                           'global costmap')
        if self.planned_path:
            axis.plot(
                [sample['x'] for sample in self.planned_path],
                [sample['y'] for sample in self.planned_path],
                color='#1f77b4', linestyle='--', linewidth=1.4,
                label='latest Nav2 global plan')
        if self.path:
            axis.plot(
                [sample['x'] for sample in self.path],
                [sample['y'] for sample in self.path],
                color='#d62728', linewidth=2.0, label='real odom trajectory')
            axis.scatter(
                self.path[0]['x'], self.path[0]['y'], color='#2ca02c',
                marker='o', s=55, label='recorded start')
            axis.scatter(
                self.path[-1]['x'], self.path[-1]['y'], color='#1f77b4',
                marker='x', s=65, label='recorded end')
        axis.scatter(
            self.args.x, self.args.y, facecolors='none', edgecolors='#ff7f0e',
            marker='*', s=150, linewidths=2, label='goal')
        axis.set_title('Physical robot /map + costmap trajectory')
        axis.set_aspect('equal', adjustable='datalim')
        axis.grid(True, alpha=0.25)
        axis.legend(loc='best', fontsize=8)

    def draw_commands(self, axis):
        if self.command_samples:
            topics = sorted({sample['topic'] for sample in self.command_samples})
            for topic in topics:
                samples = [
                    sample for sample in self.command_samples
                    if sample['topic'] == topic]
                axis.plot(
                    [sample['wall_elapsed_s'] for sample in samples],
                    [sample['linear_x_mps'] for sample in samples],
                    label=f'{topic} linear.x')
                axis.plot(
                    [sample['wall_elapsed_s'] for sample in samples],
                    [sample['angular_z_radps'] for sample in samples],
                    '--', label=f'{topic} angular.z')
        axis.axhline(0.0, color='black', linewidth=0.7)
        axis.set_xlabel('navigation time (s)')
        axis.set_ylabel('velocity (m/s or rad/s)')
        axis.set_title('Command chain evidence (no direct base command)')
        axis.grid(True, alpha=0.25)
        axis.legend(loc='best', fontsize=8)

    def write_artifacts(self, output_dir):
        os.makedirs(output_dir, exist_ok=True)
        self.copy_runtime_snapshots(output_dir)
        metrics = self.metrics()
        self.write_csv(os.path.join(output_dir, 'trajectory.csv'), self.path)
        self.write_planned_path_csv(
            os.path.join(output_dir, '规划路径.csv'), self.planned_path)
        self.write_commands_csv(os.path.join(output_dir, '速度命令.csv'))
        figure, axes = plt.subplots(1, 2, figsize=(16, 7), constrained_layout=True)
        self.draw_trajectory(axes[0])
        self.draw_commands(axes[1])
        figure.suptitle(
            f'{self.args.label} | real robot | status={self.status} | '
            f'wall={metrics["navigation_wall_duration_s"]:.1f}s', fontsize=13)
        figure.savefig(os.path.join(output_dir, 'trajectory_comparison.png'), dpi=160)
        plt.close(figure)
        figure, axis = plt.subplots(figsize=(9, 7), constrained_layout=True)
        self.draw_trajectory(axis)
        figure.savefig(os.path.join(output_dir, 'trajectory.png'), dpi=160)
        plt.close(figure)
        with open(os.path.join(output_dir, 'metrics.yaml'), 'w',
                  encoding='utf-8') as stream:
            for key, value in metrics.items():
                if isinstance(value, dict):
                    stream.write(f'{key}:\n')
                    for child_key, child_value in value.items():
                        stream.write(f'  {child_key}: {child_value}\n')
                elif isinstance(value, bool):
                    stream.write(f'{key}: {str(value).lower()}\n')
                elif value is None:
                    stream.write(f'{key}: null\n')
                elif isinstance(value, str) and ':' in value:
                    stream.write(f'{key}: "{value}"\n')
                else:
                    stream.write(f'{key}: {value}\n')
        with open(os.path.join(output_dir, 'contacts.yaml'), 'w',
                  encoding='utf-8') as stream:
            stream.write(
                'source: physical_robot\n'
                'gazebo_contacts_available: false\n'
                'non_ground_contact: null\n'
                'status: unavailable_until_bumper_or_rosbag_evidence\n'
                'note: "Physical robot has no Gazebo contacts topic; do not infer zero collision."\n')
        with open(os.path.join(output_dir, '世界快照.yaml'), 'w',
                  encoding='utf-8') as stream:
            stream.write(
                'world_type: physical_robot\n'
                'platform: WATER II-S\n'
                f'git_commit: {metrics["git_commit"]}\n'
                f'base_frame: {self.args.base_frame}\n'
                f'camera_frame: {self.args.camera_frame}\n'
                f'map_topic: {self.args.map_topic}\n'
                'gazebo_world_file: null\n'
                'note: "Record room, floor, load, camera serial and measured extrinsics here."\n')
        with open(os.path.join(output_dir, '实验参数.yaml'), 'w',
                  encoding='utf-8') as stream:
            stream.write(
                f'profile: {self.args.profile}\n'
                f'goal_frame: {self.args.frame}\n'
                f'goal_x_m: {self.args.x}\n'
                f'goal_y_m: {self.args.y}\n'
                f'goal_yaw_rad: {self.args.yaw}\n'
                f'plan_topic: {self.args.plan_topic}\n'
                f'odom_topic: {self.args.odom_topic}\n'
                f'cmd_vel_topic: {self.args.cmd_vel_topic}\n'
                f'cmd_vel_safe_topic: {self.args.cmd_vel_safe_topic}\n'
                f'ground_topic: {self.args.ground_topic}\n'
                f'imu_topic: {self.args.imu_topic}\n'
                f'runtime_snapshot_dir: {self.args.runtime_snapshot_dir}\n'
                'use_sim_time: false\n'
                'real_robot: true\n'
                'direct_velocity_publisher: false\n')


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Record one physical WATER II-S navigation goal without direct velocity control.')
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--y', type=float, required=True)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--frame', default='map')
    parser.add_argument('--label', default='real_navigation_trial')
    parser.add_argument(
        '--output-dir', default='/workspaces/rtabmap_tb3_nav/实车记录')
    parser.add_argument(
        '--profile', default='adaptive_goal_line_050_recovery_v13_line_tiebreaker')
    parser.add_argument('--settle-seconds', type=float, default=5.0)
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--camera-frame', default='camera_link')
    parser.add_argument('--odom-frame', default='odom')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--map-topic', default='/map')
    parser.add_argument('--global-costmap-topic', default='/global_costmap/costmap')
    parser.add_argument('--local-costmap-topic', default='/local_costmap/costmap')
    parser.add_argument('--plan-topic', default='/plan')
    parser.add_argument('--cmd-vel-topic', default='/cmd_vel')
    parser.add_argument('--cmd-vel-safe-topic', default='/cmd_vel_safe')
    parser.add_argument(
        '--timeout-seconds', type=float, default=180.0,
        help='Maximum NavigateToPose duration before a cancellation request.')
    parser.add_argument('--color-image-topic', default='/camera/color/image_raw')
    parser.add_argument('--color-info-topic', default='/camera/color/camera_info')
    parser.add_argument(
        '--depth-image-topic',
        default='/camera/aligned_depth_to_color/image_raw')
    parser.add_argument(
        '--depth-info-topic',
        default='/camera/aligned_depth_to_color/camera_info')
    parser.add_argument('--cloud-topic', default='/camera/cloud')
    parser.add_argument('--obstacles-topic', default='/camera/obstacles')
    parser.add_argument('--ground-topic', default='/camera/ground')
    parser.add_argument('--imu-topic', default='/camera/imu')
    parser.add_argument(
        '--runtime-snapshot-dir', default='',
        help=('Directory written by real_d435i_nav.launch.py containing the '
              'effective Nav2, collision-monitor, camera and metadata YAML.'))
    args = parser.parse_args(argv)
    args.settle_seconds = max(args.settle_seconds, 0.0)
    args.timeout_seconds = max(args.timeout_seconds, 1.0)
    return args


def main(argv=None):
    args = parse_args(argv)
    output_dir = os.path.join(args.output_dir, args.label)
    rclpy.init()
    trial = RealNavigationTrial(args)
    exit_code = 5
    try:
        trial.run()
    except Exception as error:
        trial.node.get_logger().error(str(error))
    finally:
        trial.write_artifacts(output_dir)
        trial.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    metrics = trial.metrics()
    print(
        f"Trial {args.label}: status={metrics['nav2_status']} "
        f"success={metrics['succeeded']} "
        f"wall={metrics['navigation_wall_duration_s']:.2f}s "
        f"path={metrics['trajectory_length_m']:.2f}m")
    if metrics['succeeded']:
        exit_code = 0
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
