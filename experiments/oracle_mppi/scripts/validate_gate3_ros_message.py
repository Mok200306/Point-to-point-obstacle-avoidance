#!/usr/bin/env python3
"""Validate one live Gate 3 PredictedOccupancyGrid message.

This is intentionally a read-only smoke check.  It does not send a goal,
change Nav2 parameters, or modify a costmap.
"""

import argparse
import math

import rclpy
from oracle_dynamic_nav_msgs.msg import PredictedOccupancyGrid
from rclpy.node import Node


class MessageValidator(Node):
    def __init__(self, args):
        super().__init__('gate3_message_validator')
        self.args = args
        self.message = None
        self.subscription = self.create_subscription(
            PredictedOccupancyGrid, args.topic, self._callback, 10)

    def _callback(self, message):
        if self.message is None:
            self.message = message


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/oracle/predicted_occupancy')
    parser.add_argument('--timeout-s', type=float, default=10.0)
    parser.add_argument('--expected-frame', default='odom')
    parser.add_argument('--expected-source', default='oracle')
    parser.add_argument('--expected-resolution', type=float, default=0.05)
    parser.add_argument('--expected-width', type=int, default=120)
    parser.add_argument('--expected-height', type=int, default=100)
    parser.add_argument('--expected-dt', type=float, default=0.10)
    parser.add_argument('--expected-steps', type=int, default=31)
    parser.add_argument('--min-occupied-cells', type=int, default=0)
    args = parser.parse_args()

    rclpy.init()
    node = MessageValidator(args)
    deadline = node.get_clock().now().nanoseconds / 1e9 + args.timeout_s
    try:
        while node.message is None:
            rclpy.spin_once(node, timeout_sec=0.2)
            now = node.get_clock().now().nanoseconds / 1e9
            if now >= deadline:
                print('FAIL: timed out waiting for one message')
                return 1

        message = node.message
        errors = []
        if message.header.frame_id != args.expected_frame:
            errors.append(
                f'frame_id={message.header.frame_id!r}, '
                f'expected {args.expected_frame!r}')
        if message.source != args.expected_source:
            errors.append(
                f'source={message.source!r}, expected {args.expected_source!r}')
        if not math.isclose(
                message.resolution, args.expected_resolution, abs_tol=1e-6):
            errors.append(f'resolution={message.resolution}')
        if message.width != args.expected_width:
            errors.append(f'width={message.width}')
        if message.height != args.expected_height:
            errors.append(f'height={message.height}')
        if not math.isclose(message.dt, args.expected_dt, abs_tol=1e-6):
            errors.append(f'dt={message.dt}')
        if message.steps != args.expected_steps:
            errors.append(f'steps={message.steps}')
        expected_data = message.steps * message.width * message.height
        if len(message.data) != expected_data:
            errors.append(
                f'data_len={len(message.data)}, expected {expected_data}')
        if message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0:
            errors.append('header.stamp is zero')
        if not all(0.0 <= value <= 1.0 for value in message.data):
            errors.append('data contains a value outside [0, 1]')

        occupied = sum(1 for value in message.data if value >= 0.5)
        print(
            f'frame={message.header.frame_id} '
            f'stamp={message.header.stamp.sec}.'
            f'{message.header.stamp.nanosec:09d} '
            f'grid={message.width}x{message.height}@'
            f'{message.resolution:.3f}m dt={message.dt:.3f} '
            f'steps={message.steps} data_len={len(message.data)} '
            f'occupied_cells={occupied} source={message.source} '
            f'origin=({message.origin.position.x:.3f}, '
            f'{message.origin.position.y:.3f})')
        if errors:
            print('FAIL: ' + '; '.join(errors))
            return 1
        if occupied < args.min_occupied_cells:
            print(
                f'FAIL: occupied_cells={occupied}, '
                f'expected at least {args.min_occupied_cells}')
            return 1
        print('PASS: Gate 3 message interface fields are valid')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
