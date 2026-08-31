#!/usr/bin/env python3
"""Drive one Gazebo obstacle back and forth through a fixed corridor.

The obstacle itself is a non-static Gazebo model with the
``libgazebo_ros_planar_move.so`` plugin.  This node only publishes a bounded
velocity command; it does not publish a map, a route, or any navigation hint.
The model pose is read from ``/gazebo/model_states`` so the direction change is
repeatable even when Gazebo's real-time factor varies.
"""

import sys

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter


class DynamicObstacleDriver:
    def __init__(self):
        self.node = rclpy.create_node(
            'dynamic_obstacle_driver',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.node.declare_parameter('model_name', 'dynamic_obstacle')
        self.node.declare_parameter(
            'cmd_vel_topic', '/dynamic_obstacle/cmd_vel')
        self.node.declare_parameter('speed_mps', 0.18)
        self.node.declare_parameter('min_y_m', -2.8)
        self.node.declare_parameter('max_y_m', -1.0)
        self.node.declare_parameter('command_rate_hz', 20.0)
        self.model_name = self.node.get_parameter(
            'model_name').get_parameter_value().string_value
        topic = self.node.get_parameter(
            'cmd_vel_topic').get_parameter_value().string_value
        self.speed = abs(self.node.get_parameter(
            'speed_mps').get_parameter_value().double_value)
        self.min_y = self.node.get_parameter(
            'min_y_m').get_parameter_value().double_value
        self.max_y = self.node.get_parameter(
            'max_y_m').get_parameter_value().double_value
        rate = max(self.node.get_parameter(
            'command_rate_hz').get_parameter_value().double_value, 1.0)
        if self.min_y >= self.max_y:
            raise ValueError('min_y_m must be smaller than max_y_m')

        self.publisher = self.node.create_publisher(Twist, topic, 10)
        self.subscription = self.node.create_subscription(
            ModelStates, '/gazebo/model_states', self.model_states_callback,
            20)
        self.timer = self.node.create_timer(1.0 / rate, self.step)
        self.y = None
        self.direction = 1.0
        self.started = False
        self.last_boundary_log = None
        self.node.get_logger().info(
            f'Controlling {self.model_name!r}: y={self.min_y:.2f}..'
            f'{self.max_y:.2f} m at {self.speed:.2f} m/s')

    def model_states_callback(self, message):
        if self.model_name not in message.name:
            return
        self.y = message.pose[message.name.index(self.model_name)].position.y
        if self.y >= self.max_y:
            self.direction = -1.0
            boundary = 'max'
        elif self.y <= self.min_y:
            self.direction = 1.0
            boundary = 'min'
        else:
            boundary = None
        if not self.started:
            self.started = True
            self.node.get_logger().info(
                f'Model found at y={self.y:.3f} m; motion started.')
        if boundary and boundary != self.last_boundary_log:
            self.last_boundary_log = boundary
            self.node.get_logger().info(
                f'Reversed at {boundary} boundary: y={self.y:.3f} m')

    def publish_stop(self):
        if not rclpy.ok():
            return
        try:
            self.publisher.publish(Twist())
        except rclpy.exceptions.RCLError:
            pass

    def step(self):
        command = Twist()
        if self.y is not None:
            command.linear.y = self.direction * self.speed
        self.publisher.publish(command)


def main():
    rclpy.init()
    driver = None
    try:
        driver = DynamicObstacleDriver()
        rclpy.spin(driver.node)
    except KeyboardInterrupt:
        pass
    finally:
        if driver is not None:
            driver.publish_stop()
            driver.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
