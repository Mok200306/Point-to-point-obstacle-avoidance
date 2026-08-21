#!/usr/bin/env python3
"""Publish a black start-to-goal reference line for RViz.

The navigation action remains the source of the real plan. This node only
visualizes the straight segment requested for the current goal; it never sends
velocity commands or changes Nav2 planning.
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class GoalLineVisualizer:
    def __init__(self):
        self.node = rclpy.create_node('goal_line_visualizer')
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(
            self.tf_buffer, self.node, spin_thread=False)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.node.create_publisher(Path, '/goal_line', qos)
        self.subscriptions = [
            self.node.create_subscription(
                PoseStamped, '/goal_line_request', self.goal_callback, qos),
            # Nav2's RViz GoalTool publishes this topic in distributions that
            # expose the selected pose, so manual RViz goals get the same line.
            self.node.create_subscription(
                PoseStamped, '/goal_pose', self.goal_callback, qos),
        ]
        self.goal = None
        self.start = None
        self.timer = self.node.create_timer(0.20, self.retry_publish)

    def goal_callback(self, message):
        if not message.header.frame_id:
            self.node.get_logger().warning(
                'Ignoring a goal without a frame_id.')
            return
        self.goal = message
        self.start = None
        self.retry_publish()

    def lookup_start(self):
        for base_frame in ('base_footprint', 'base_link'):
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.goal.header.frame_id, base_frame, Time())
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                return (
                    translation.x, translation.y,
                    rotation.x, rotation.y, rotation.z, rotation.w)
            except Exception:
                continue
        return None

    def retry_publish(self):
        if self.goal is None:
            return
        if self.start is None:
            self.start = self.lookup_start()
        if self.start is None:
            return

        path = Path()
        path.header.frame_id = self.goal.header.frame_id
        path.header.stamp = self.node.get_clock().now().to_msg()

        start_pose = PoseStamped()
        start_pose.header = path.header
        start_pose.pose.position.x = self.start[0]
        start_pose.pose.position.y = self.start[1]
        start_pose.pose.orientation.x = self.start[2]
        start_pose.pose.orientation.y = self.start[3]
        start_pose.pose.orientation.z = self.start[4]
        start_pose.pose.orientation.w = self.start[5]

        goal_pose = PoseStamped()
        goal_pose.header = path.header
        goal_pose.pose = self.goal.pose
        path.poses = [start_pose, goal_pose]
        self.publisher.publish(path)


def main():
    rclpy.init()
    visualizer = GoalLineVisualizer()
    try:
        rclpy.spin(visualizer.node)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
