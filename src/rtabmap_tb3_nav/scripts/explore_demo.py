#!/usr/bin/env python3
"""Drive a safe perimeter route so RGB-D SLAM can map the demo room."""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


WAYPOINTS = [
    (-8.5, 4.8),
    (8.5, 4.8),
    (8.5, -4.8),
    (-8.5, -4.8),
    (-8.5, 0.0),
]


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_error(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class DemoExplorer:
    def __init__(self):
        self.node = rclpy.create_node('rtabmap_demo_explorer')
        # Feed Nav2's velocity smoother instead of competing with its output
        # publisher on /cmd_vel.
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.subscription = self.node.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.pose = None
        self.index = 0
        self.finished = False
        self.node.get_logger().info(
            f'RGB-D mapping route started: {len(WAYPOINTS)} waypoints')

    def odom_callback(self, message):
        self.pose = message.pose.pose

    def publish_stop(self):
        if not rclpy.ok():
            return
        try:
            for _ in range(3):
                self.publisher.publish(Twist())
                time.sleep(0.02)
        except rclpy.exceptions.RCLError:
            # The launch process may have already invalidated the ROS context.
            pass

    def step(self):
        if self.pose is None:
            return

        if self.index >= len(WAYPOINTS):
            self.publish_stop()
            if not self.finished:
                self.finished = True
                self.node.get_logger().info(
                    'RGB-D mapping route complete. Send the B goal now.')
            return

        target_x, target_y = WAYPOINTS[self.index]
        current_x = self.pose.position.x
        current_y = self.pose.position.y
        current_yaw = yaw_from_quaternion(self.pose.orientation)
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.hypot(dx, dy)

        if distance < 0.18:
            self.publish_stop()
            self.node.get_logger().info(
                f'Reached mapping waypoint {self.index + 1}/{len(WAYPOINTS)}: '
                f'({target_x:.1f}, {target_y:.1f})')
            self.index += 1
            return

        target_yaw = math.atan2(dy, dx)
        heading_error = angle_error(target_yaw, current_yaw)
        command = Twist()
        command.angular.z = clamp(1.5 * heading_error, -0.9, 0.9)
        if abs(heading_error) < 0.55:
            command.linear.x = clamp(0.16 * distance, 0.05, 0.18)
        self.publisher.publish(command)

    def run(self):
        while rclpy.ok() and not self.finished:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            self.step()
            time.sleep(0.04)
        self.publish_stop()


def main():
    rclpy.init()
    explorer = DemoExplorer()
    try:
        explorer.run()
    except KeyboardInterrupt:
        explorer.node.get_logger().info('Mapping route interrupted.')
    finally:
        explorer.publish_stop()
        explorer.node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
