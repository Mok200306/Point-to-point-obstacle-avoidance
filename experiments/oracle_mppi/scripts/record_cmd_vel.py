#!/usr/bin/env python3
"""Record commanded velocities with the ROS (simulation) clock.

The Twist message has no header timestamp, so the callback records both the
node clock and wall-clock arrival time.  Gate 1 uses the simulation timestamp
for control-period statistics and keeps the wall timestamp for diagnostics.
"""

import argparse
import csv
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter


class CmdVelRecorder(Node):
    def __init__(self, topic, output_path):
        # The recorder is part of the simulation evidence pipeline.  Without
        # this override its clock defaults to wall time even when /cmd_vel and
        # Gazebo ground truth are stamped in the ROS simulation-time domain.
        super().__init__(
            'oracle_cmd_vel_recorder',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self._stream = open(output_path, 'w', newline='', buffering=1)
        self._writer = csv.writer(self._stream)
        self._writer.writerow([
            'sim_stamp_s', 'wall_monotonic_s', 'linear_x_mps',
            'linear_y_mps', 'angular_z_rps',
        ])
        self._subscription = self.create_subscription(
            Twist, topic, self._callback, 100)
        self._count = 0

    def _callback(self, message):
        sim_stamp = self.get_clock().now().nanoseconds / 1e9
        self._writer.writerow([
            f'{sim_stamp:.9f}',
            f'{time.monotonic():.9f}',
            f'{message.linear.x:.9f}',
            f'{message.linear.y:.9f}',
            f'{message.angular.z:.9f}',
        ])
        self._count += 1

    def close(self):
        self.get_logger().info('Recorded %d %s messages', self._count, 'cmd_vel')
        self._stream.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/cmd_vel')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()

    rclpy.init()
    node = CmdVelRecorder(args.topic, args.output)

    def stop_handler(signum, frame):  # noqa: ARG001
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
