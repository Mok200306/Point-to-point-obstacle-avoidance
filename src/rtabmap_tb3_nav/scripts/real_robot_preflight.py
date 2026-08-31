#!/usr/bin/env python3
"""Read-only preflight checks for the WATER II-S and Intel RealSense D435i.

Only ROS graph inspection, subscriptions, TF lookup and parameter queries are
performed.  This tool never publishes Twist messages, calls the vendor TCP
API, toggles E-stop, or commands the chassis.  Its result is an interface
check, not a substitute for a human E-stop confirmation or motion test.
"""

import argparse
import json
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener


def expected_topics(args):
    """Return the ROS topic contract, allowing launch remappings to be tested."""
    return {
        args.odom_topic: ('nav_msgs/msg/Odometry', Odometry),
        args.map_topic: ('nav_msgs/msg/OccupancyGrid', OccupancyGrid),
        args.cmd_vel_topic: ('geometry_msgs/msg/Twist', Twist),
        args.safe_cmd_vel_topic: ('geometry_msgs/msg/Twist', Twist),
        args.color_image_topic: ('sensor_msgs/msg/Image', Image),
        args.color_info_topic: ('sensor_msgs/msg/CameraInfo', CameraInfo),
        args.depth_image_topic: ('sensor_msgs/msg/Image', Image),
        args.depth_info_topic: ('sensor_msgs/msg/CameraInfo', CameraInfo),
        args.cloud_topic: ('sensor_msgs/msg/PointCloud2', PointCloud2),
        args.obstacles_topic: ('sensor_msgs/msg/PointCloud2', PointCloud2),
        args.ground_topic: ('sensor_msgs/msg/PointCloud2', PointCloud2),
        args.imu_topic: ('sensor_msgs/msg/Imu', Imu),
        args.gyro_topic: ('sensor_msgs/msg/Imu', Imu),
        args.accel_topic: ('sensor_msgs/msg/Imu', Imu),
    }


class RealRobotPreflight(Node):
    """Collect graph, message, TF and parameter evidence without motion."""

    def __init__(self, args):
        super().__init__('real_robot_preflight')
        self.args = args
        self.received_messages = set()
        self.expected_topics = expected_topics(args)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=False)
        camera_topics = {
            self.args.color_image_topic,
            self.args.color_info_topic,
            self.args.depth_image_topic,
            self.args.depth_info_topic,
            self.args.cloud_topic,
            self.args.obstacles_topic,
            self.args.ground_topic,
            self.args.imu_topic,
            self.args.gyro_topic,
            self.args.accel_topic,
        }
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for topic, (_, message_type) in self.expected_topics.items():
            if topic == self.args.map_topic:
                qos = map_qos
            elif topic == self.args.odom_topic:
                # A number of vendor drivers publish odometry best-effort.
                # A best-effort subscription remains compatible with a
                # reliable publisher and avoids a false negative here.
                qos = qos_profile_sensor_data
            else:
                qos = (qos_profile_sensor_data
                       if topic in camera_topics else 10)
            self.create_subscription(
                message_type, topic, self._message_callback(topic), qos)

    def _message_callback(self, topic):
        def callback(_message):
            self.received_messages.add(topic)
        return callback

    @staticmethod
    def _normalise_node_name(name, namespace):
        if namespace == '/':
            return f'/{name}'
        return f'{namespace.rstrip("/")}/{name}'

    def topic_report(self):
        discovered = {
            name: sorted(types)
            for name, types in self.get_topic_names_and_types()
        }
        report = {}
        for topic, (expected_type, _) in self.expected_topics.items():
            actual_types = discovered.get(topic, [])
            publishers = self.get_publishers_info_by_topic(topic)
            subscribers = self.get_subscriptions_info_by_topic(topic)
            report[topic] = {
                'expected_type': expected_type,
                'actual_types': actual_types,
                'type_ok': expected_type in actual_types,
                'publisher_count': len(publishers),
                'subscriber_count': len(subscribers),
                'message_received': topic in self.received_messages,
                'publisher_nodes': sorted({
                    self._normalise_node_name(
                        endpoint.node_name, endpoint.node_namespace)
                    for endpoint in publishers
                }),
                'subscriber_nodes': sorted({
                    self._normalise_node_name(
                        endpoint.node_name, endpoint.node_namespace)
                    for endpoint in subscribers
                }),
            }
        return report

    def tf_report(self):
        try:
            map_to_odom = self.tf_buffer.can_transform(
                self.args.map_frame, self.args.odom_frame, Time(),
                timeout=Duration(seconds=0.2))
            odom_to_base = self.tf_buffer.can_transform(
                self.args.odom_frame, self.args.base_frame, Time(),
                timeout=Duration(seconds=0.2))
            base_to_camera = self.tf_buffer.can_transform(
                self.args.base_frame, self.args.camera_frame, Time(),
                timeout=Duration(seconds=0.2))
            return {
                'map_to_odom': {
                    'target_frame': self.args.map_frame,
                    'source_frame': self.args.odom_frame,
                    'available': bool(map_to_odom),
                },
                'odom_to_base': {
                    'target_frame': self.args.odom_frame,
                    'source_frame': self.args.base_frame,
                    'available': bool(odom_to_base),
                },
                'base_to_camera': {
                    'target_frame': self.args.base_frame,
                    'source_frame': self.args.camera_frame,
                    'available': bool(base_to_camera),
                },
                'available': bool(map_to_odom and odom_to_base and base_to_camera),
            }
        except Exception as error:  # pragma: no cover - ROS implementation detail
            return {
                'map_to_odom': {
                    'target_frame': self.args.map_frame,
                    'source_frame': self.args.odom_frame,
                    'available': False,
                },
                'odom_to_base': {
                    'target_frame': self.args.odom_frame,
                    'source_frame': self.args.base_frame,
                    'available': False,
                },
                'base_to_camera': {
                    'target_frame': self.args.base_frame,
                    'source_frame': self.args.camera_frame,
                    'available': False,
                },
                'available': False,
                'error': str(error),
            }

    def parameter_report(self):
        node_names = {
            self._normalise_node_name(name, namespace)
            for name, namespace in self.get_node_names_and_namespaces()
        }
        report = {}
        for node_name in self.args.parameter_nodes:
            entry = {
                'present': node_name in node_names,
                'use_sim_time': None,
                'query_ok': False,
            }
            if entry['present']:
                service_name = f'{node_name}/get_parameters'
                client = self.create_client(GetParameters, service_name)
                if client.wait_for_service(timeout_sec=0.3):
                    request = GetParameters.Request()
                    request.names = ['use_sim_time']
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(
                        self, future, timeout_sec=0.8)
                    if future.done() and future.result():
                        values = future.result().values
                        if values and values[0].type == ParameterType.PARAMETER_BOOL:
                            entry['use_sim_time'] = bool(values[0].bool_value)
                            entry['query_ok'] = True
            report[node_name] = entry
        return report

    def build_report(self):
        topics = self.topic_report()
        tf = self.tf_report()
        parameters = self.parameter_report()
        required_topics = [
            self.args.odom_topic, self.args.map_topic,
            self.args.cmd_vel_topic, self.args.safe_cmd_vel_topic,
            self.args.color_image_topic, self.args.color_info_topic,
            self.args.depth_image_topic, self.args.depth_info_topic,
            self.args.cloud_topic, self.args.obstacles_topic,
            self.args.ground_topic,
        ]
        topic_checks = {
            topic: (
                topics[topic]['type_ok'] and
                topics[topic]['publisher_count'] > 0 and
                (topics[topic]['message_received'] or
                 topic in (self.args.cmd_vel_topic,
                           self.args.safe_cmd_vel_topic)))
            for topic in required_topics
        }
        topic_checks['imu'] = (
            (topics[self.args.imu_topic]['type_ok'] and
             topics[self.args.imu_topic]['publisher_count'] > 0 and
             topics[self.args.imu_topic]['message_received']) or
            (topics[self.args.gyro_topic]['type_ok'] and
             topics[self.args.gyro_topic]['publisher_count'] > 0 and
             topics[self.args.gyro_topic]['message_received'] and
             topics[self.args.accel_topic]['type_ok'] and
             topics[self.args.accel_topic]['publisher_count'] > 0 and
             topics[self.args.accel_topic]['message_received']))
        # A silent command topic is expected before a goal, so check the graph
        # endpoints rather than waiting for or sending a motion command.
        topic_checks[self.args.cmd_vel_topic] = (
            topics[self.args.cmd_vel_topic]['type_ok'] and
            topics[self.args.cmd_vel_topic]['publisher_count'] > 0 and
            topics[self.args.cmd_vel_topic]['subscriber_count'] > 0)
        safe_subscribers = [
            node for node in topics[self.args.safe_cmd_vel_topic][
                'subscriber_nodes']
            if node != '/real_robot_preflight'
        ]
        topic_checks[self.args.safe_cmd_vel_topic] = (
            topics[self.args.safe_cmd_vel_topic]['type_ok'] and
            topics[self.args.safe_cmd_vel_topic]['publisher_count'] > 0 and
            topics[self.args.safe_cmd_vel_topic]['subscriber_count'] > 0 and
            bool(safe_subscribers))
        unexpected_cmd_vel_subscribers = [
            node for node in topics[self.args.cmd_vel_topic]['subscriber_nodes']
            if node not in (
                    self.args.collision_monitor_node,
                    '/real_robot_preflight')
        ]
        safety_routing = {
            'cmd_vel_input_topic': self.args.cmd_vel_topic,
            'cmd_vel_safe_topic': self.args.safe_cmd_vel_topic,
            'collision_monitor_node': self.args.collision_monitor_node,
            'collision_monitor_subscribed_to_cmd_vel': (
                self.args.collision_monitor_node in
                topics[self.args.cmd_vel_topic]['subscriber_nodes']),
            'external_safe_topic_subscribers': safe_subscribers,
            'unexpected_cmd_vel_subscribers': unexpected_cmd_vel_subscribers,
            'passed': (
                self.args.collision_monitor_node in
                topics[self.args.cmd_vel_topic]['subscriber_nodes'] and
                bool(safe_subscribers) and
                not unexpected_cmd_vel_subscribers),
        }
        parameter_checks = {
            node: (
                details['present'] and details['query_ok'] and
                details['use_sim_time'] is False)
            for node, details in parameters.items()
        }
        node_names = {
            self._normalise_node_name(name, namespace)
            for name, namespace in self.get_node_names_and_namespaces()
        }
        topic_names = {name for name, _ in self.get_topic_names_and_types()}
        gazebo_graph_present = (
            any(name == '/gazebo' or name.startswith('/gazebo/')
                for name in node_names) or
            any(name.startswith('/gazebo/') for name in topic_names))
        report = {
            'tool': 'real_robot_preflight.py',
            'read_only': True,
            'command_publishers_created': False,
            'vendor_api_called': False,
            'base_frame': self.args.base_frame,
            'camera_frame': self.args.camera_frame,
            'map_frame': self.args.map_frame,
            'odom_frame': self.args.odom_frame,
            'topic_checks': topic_checks,
            'topics': topics,
            'tf': tf,
            'parameters': parameters,
            'parameter_checks': parameter_checks,
            'safety_routing': safety_routing,
            'gazebo_graph_present': gazebo_graph_present,
            'hardware_estop_human_confirmed': bool(
                self.args.estop_confirmed),
        }
        report['passed_without_hardware_estop'] = (
            all(topic_checks.values()) and bool(tf['available']) and
            all(parameter_checks.values()) and safety_routing['passed'] and
            not gazebo_graph_present)
        report['passed'] = (
            report['passed_without_hardware_estop'] and
            report['hardware_estop_human_confirmed'])
        return report


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            'Read-only ROS preflight for the physical D435i/WATER II-S. '
            'It never sends velocity or E-stop commands.'))
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--camera-frame', default='camera_link')
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--odom-frame', default='odom')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--map-topic', default='/map')
    parser.add_argument('--cmd-vel-topic', default='/cmd_vel')
    parser.add_argument('--safe-cmd-vel-topic', default='/cmd_vel_safe')
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
    parser.add_argument('--gyro-topic', default='/camera/gyro/sample')
    parser.add_argument('--accel-topic', default='/camera/accel/sample')
    parser.add_argument(
        '--collision-monitor-node', default='/collision_monitor',
        help='Fully qualified collision_monitor node name.')
    parser.add_argument(
        '--parameter-nodes',
        default=(
            '/controller_server,/smoother_server,/planner_server,'
            '/behavior_server,/bt_navigator,/waypoint_follower,'
            '/velocity_smoother,/collision_monitor,/rtabmap,/camera/camera,'
            '/local_costmap/local_costmap,/global_costmap/global_costmap'),
        help='Comma-separated node names whose use_sim_time must be false.')
    parser.add_argument(
        '--estop-confirmed', action='store_true',
        help='Human confirms that the physical E-stop is available.')
    parser.add_argument('--duration', type=float, default=8.0)
    parser.add_argument('--output', default='', help='Optional JSON report path.')
    args = parser.parse_args(argv)
    args.parameter_nodes = [
        name.strip() for name in args.parameter_nodes.split(',')
        if name.strip()
    ]
    args.duration = max(args.duration, 1.0)
    return args


def main(argv=None):
    args = parse_args(argv)
    rclpy.init()
    node = RealRobotPreflight(args)
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.10)
        report = node.build_report()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if args.output:
        output_path = os.path.abspath(args.output)
        parent = os.path.dirname(output_path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        with open(output_path, 'w', encoding='utf-8') as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
