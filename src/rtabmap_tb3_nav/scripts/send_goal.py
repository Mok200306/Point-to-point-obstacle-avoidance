#!/usr/bin/env python3
"""Send one map-frame NavigateToPose goal to Nav2."""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


def quaternion_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def main():
    parser = argparse.ArgumentParser(description='Send an A-to-B goal to Nav2.')
    parser.add_argument('--x', type=float, default=8.5, help='Goal X in the map frame (large-course B point).')
    parser.add_argument('--y', type=float, default=0.0, help='Goal Y in the map frame (demo B point).')
    parser.add_argument('--yaw', type=float, default=0.0, help='Goal yaw in radians.')
    parser.add_argument('--frame', default='map', help='Goal frame, normally map.')
    parser.add_argument(
        '--settle-seconds', type=float, default=0.0,
        help='Wait for the live RGB-D map/costmap before sending the goal.')
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('rtabmap_tb3_send_goal')
    action_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    node.get_logger().info('Waiting for Nav2 navigate_to_pose...')
    if not action_client.wait_for_server(timeout_sec=30.0):
        node.get_logger().error('Nav2 action server was not available after 30 seconds.')
        node.destroy_node()
        rclpy.shutdown()
        return 2

    settle_seconds = max(args.settle_seconds, 0.0)
    if settle_seconds > 0.0:
        deadline = time.monotonic() + settle_seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.10)
        node.get_logger().info(
            f'Startup settle complete: {settle_seconds:.1f}s; sending goal.')

    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = args.frame
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = args.x
    goal.pose.pose.position.y = args.y
    goal.pose.pose.position.z = 0.0
    qx, qy, qz, qw = quaternion_from_yaw(args.yaw)
    goal.pose.pose.orientation.x = qx
    goal.pose.pose.orientation.y = qy
    goal.pose.pose.orientation.z = qz
    goal.pose.pose.orientation.w = qw

    last_feedback_time = 0.0
    last_distance = None

    def feedback_callback(feedback):
        nonlocal last_feedback_time, last_distance
        distance = feedback.feedback.distance_remaining
        now = time.monotonic()
        if (last_distance is None or now - last_feedback_time >= 0.5 or
                abs(distance - last_distance) >= 0.2):
            node.get_logger().info(f'distance remaining: {distance:.2f} m')
            last_feedback_time = now
            last_distance = distance

    send_future = action_client.send_goal_async(goal, feedback_callback=feedback_callback)
    rclpy.spin_until_future_complete(node, send_future)
    goal_handle = send_future.result()
    if goal_handle is None or not goal_handle.accepted:
        node.get_logger().error('Nav2 rejected the goal.')
        node.destroy_node()
        rclpy.shutdown()
        return 3

    node.get_logger().info('Goal accepted.')
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result()
    if result is None:
        node.get_logger().error('Nav2 returned no result.')
        status = 4
    else:
        status = result.status
        node.get_logger().info(f'NavigateToPose finished with status {status}.')

    node.destroy_node()
    rclpy.shutdown()
    return 0 if status == 4 else 5


if __name__ == '__main__':
    sys.exit(main())
