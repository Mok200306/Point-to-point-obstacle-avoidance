#!/usr/bin/env python3
"""Drive a collidable Gazebo proxy from a deterministic waypoint schedule.

Gate 2 deliberately keeps this node separate from the Oracle publisher.  It
only executes the known scenario trajectory and records Gazebo state.  It does
not publish future information to Nav2 and it does not estimate a trajectory
from current velocity.
"""

import argparse
import csv
import math
import signal
from pathlib import Path

import rclpy
import yaml
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from rclpy.parameter import Parameter


def quaternion_from_yaw(yaw):
    return Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def rectangle_vertices(cx, cy, yaw, half_x, half_y):
    c = math.cos(yaw)
    s = math.sin(yaw)
    local = ((half_x, half_y), (half_x, -half_y),
             (-half_x, -half_y), (-half_x, half_y))
    return [
        (cx + c * x - s * y, cy + s * x + c * y)
        for x, y in local
    ]


def cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def subtract(a, b):
    return (a[0] - b[0], a[1] - b[1])


def point_segment_distance(point, start, end):
    segment = subtract(end, start)
    length_sq = segment[0] * segment[0] + segment[1] * segment[1]
    if length_sq <= 1e-15:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    t = ((point[0] - start[0]) * segment[0] +
         (point[1] - start[1]) * segment[1]) / length_sq
    t = max(0.0, min(1.0, t))
    projection = (start[0] + t * segment[0], start[1] + t * segment[1])
    return math.hypot(point[0] - projection[0], point[1] - projection[1])


def orientation(a, b, c):
    return cross(subtract(b, a), subtract(c, a))


def on_segment(a, b, point, epsilon=1e-9):
    return (
        min(a[0], b[0]) - epsilon <= point[0] <= max(a[0], b[0]) + epsilon
        and min(a[1], b[1]) - epsilon <= point[1] <= max(a[1], b[1]) + epsilon
    )


def segments_intersect(a, b, c, d):
    o1 = orientation(a, b, c)
    o2 = orientation(a, b, d)
    o3 = orientation(c, d, a)
    o4 = orientation(c, d, b)
    epsilon = 1e-9
    if ((o1 > epsilon and o2 < -epsilon) or
            (o1 < -epsilon and o2 > epsilon)) and \
            ((o3 > epsilon and o4 < -epsilon) or
             (o3 < -epsilon and o4 > epsilon)):
        return True
    return (
        abs(o1) <= epsilon and on_segment(a, b, c)
        or abs(o2) <= epsilon and on_segment(a, b, d)
        or abs(o3) <= epsilon and on_segment(c, d, a)
        or abs(o4) <= epsilon and on_segment(c, d, b)
    )


def polygons_intersect(first, second):
    for polygon_a, polygon_b in ((first, second), (second, first)):
        for index, point in enumerate(polygon_a):
            next_point = polygon_a[(index + 1) % len(polygon_a)]
            edge = subtract(next_point, point)
            axis = (-edge[1], edge[0])
            first_projection = [p[0] * axis[0] + p[1] * axis[1]
                               for p in first]
            second_projection = [p[0] * axis[0] + p[1] * axis[1]
                                 for p in second]
            if (max(first_projection) < min(second_projection) or
                    max(second_projection) < min(first_projection)):
                return False
    return True


def polygon_distance(first, second):
    if polygons_intersect(first, second):
        return 0.0
    distance = float('inf')
    for index, point in enumerate(first):
        next_point = first[(index + 1) % len(first)]
        for candidate in second:
            distance = min(distance,
                           point_segment_distance(candidate, point, next_point))
    for index, point in enumerate(second):
        next_point = second[(index + 1) % len(second)]
        for candidate in first:
            distance = min(distance,
                           point_segment_distance(candidate, point, next_point))
    return distance


class DynamicObstacleController(Node):
    def __init__(self, scenario, difficulty, output_path, summary_path,
                 robot_name='waffle'):
        # rclpy already declares the standard use_sim_time parameter for a
        # node.  Override it at construction instead of declaring it again;
        # declaring it a second time raises ParameterAlreadyDeclaredException
        # on ROS 2 Humble.
        super().__init__(
            'oracle_dynamic_obstacle_controller',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.scenario = scenario
        self.difficulty = difficulty
        self.obstacle_name = scenario['obstacle_name']
        self.robot_name = robot_name
        profile = scenario.get('difficulty_profiles', {}).get(difficulty, {})
        self.time_scale = float(profile.get('time_scale', 1.0))
        self.start_delay = float(profile.get('start_delay_s', 0.0))
        self.update_period = float(scenario.get('update_period_s', 0.05))
        self.obstacle_half_size = float(
            scenario.get('obstacle_half_size_m', 0.30))
        self.waypoints = [
            {
                't': float(point['t']),
                'x': float(point['x']),
                'y': float(point['y']),
                'yaw': float(point.get('yaw', 0.0)),
            }
            for point in scenario['waypoints']
        ]
        if len(self.waypoints) < 2:
            raise ValueError('scenario needs at least two waypoints')
        if self.time_scale <= 0.0:
            raise ValueError('difficulty time_scale must be positive')

        output = Path(output_path)
        output.parent.mkdir(parents=True, exist_ok=True)
        self._stream = output.open('w', newline='', buffering=1)
        self._writer = csv.writer(self._stream)
        self._writer.writerow([
            'sim_time_s', 'elapsed_s', 'scenario_time_s',
            'planned_x_m', 'planned_y_m', 'planned_yaw_rad',
            'planned_vx_mps', 'planned_vy_mps',
            'obstacle_x_m', 'obstacle_y_m', 'obstacle_yaw_rad',
            'robot_x_m', 'robot_y_m', 'robot_yaw_rad',
            'center_distance_m', 'robot_obstacle_clearance_m',
            'model_state_samples', 'service_updates', 'service_failures',
        ])
        self._summary_path = Path(summary_path)
        self._start_sim_ns = None
        self._last_obstacle = None
        self._last_robot = None
        self._model_state_samples = 0
        self._service_updates = 0
        self._service_failures = 0
        self._pending_request = None
        self._clearances = []
        self._trajectory_errors = []
        self._last_sim_s = None

        self._state_client = self.create_client(
            SetEntityState, '/gazebo/set_entity_state')
        self._states_subscription = self.create_subscription(
            ModelStates, '/gazebo/model_states', self._states_callback, 50)
        self._timer = self.create_timer(self.update_period, self._tick)

    def _states_callback(self, message):
        self._model_state_samples += 1
        names = list(message.name)
        if self.obstacle_name in names:
            index = names.index(self.obstacle_name)
            pose = message.pose[index]
            self._last_obstacle = (
                pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))
        if self.robot_name in names:
            index = names.index(self.robot_name)
            pose = message.pose[index]
            self._last_robot = (
                pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))

    def _trajectory_at(self, elapsed):
        if elapsed < self.start_delay:
            first = self.waypoints[0]
            return first['x'], first['y'], first['yaw'], 0.0, 0.0
        scenario_time = (elapsed - self.start_delay) / self.time_scale
        if scenario_time <= self.waypoints[0]['t']:
            first = self.waypoints[0]
            return first['x'], first['y'], first['yaw'], 0.0, 0.0
        for first, second in zip(self.waypoints, self.waypoints[1:]):
            if scenario_time <= second['t']:
                duration = second['t'] - first['t']
                ratio = ((scenario_time - first['t']) / duration
                         if duration > 0.0 else 1.0)
                ratio = max(0.0, min(1.0, ratio))
                x = first['x'] + ratio * (second['x'] - first['x'])
                y = first['y'] + ratio * (second['y'] - first['y'])
                vx = (second['x'] - first['x']) / duration / self.time_scale
                vy = (second['y'] - first['y']) / duration / self.time_scale
                return x, y, first['yaw'] + ratio * (
                    second['yaw'] - first['yaw']), vx, vy
        last = self.waypoints[-1]
        return last['x'], last['y'], last['yaw'], 0.0, 0.0

    def _service_done(self, future):
        self._pending_request = None
        try:
            response = future.result()
            if response is None or not response.success:
                self._service_failures += 1
        except Exception:  # DDS/service teardown can complete exceptionally.
            self._service_failures += 1

    def _tick(self):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns <= 0:
            return
        if self._start_sim_ns is None:
            self._start_sim_ns = now_ns
        sim_time = now_ns / 1e9
        elapsed = (now_ns - self._start_sim_ns) / 1e9
        x, y, yaw, vx, vy = self._trajectory_at(elapsed)

        if not self._state_client.service_is_ready():
            self._state_client.wait_for_service(timeout_sec=0.0)
        if self._state_client.service_is_ready() and self._pending_request is None:
            request = SetEntityState.Request()
            request.state.name = self.obstacle_name
            request.state.pose.position.x = x
            request.state.pose.position.y = y
            request.state.pose.position.z = 0.0
            request.state.pose.orientation = quaternion_from_yaw(yaw)
            request.state.twist.linear.x = vx
            request.state.twist.linear.y = vy
            request.state.reference_frame = 'world'
            self._pending_request = self._state_client.call_async(request)
            self._pending_request.add_done_callback(self._service_done)
            self._service_updates += 1

        center_distance = ''
        clearance = ''
        obstacle = self._last_obstacle
        robot = self._last_robot
        if obstacle is not None:
            self._trajectory_errors.append(math.hypot(obstacle[0] - x, obstacle[1] - y))
        if obstacle is not None and robot is not None:
            center_distance = math.hypot(obstacle[0] - robot[0], obstacle[1] - robot[1])
            robot_polygon = rectangle_vertices(
                robot[0], robot[1], robot[2], 0.33, 0.27)
            obstacle_polygon = rectangle_vertices(
                obstacle[0], obstacle[1], obstacle[2],
                self.obstacle_half_size, self.obstacle_half_size)
            clearance = polygon_distance(robot_polygon, obstacle_polygon)
            self._clearances.append(clearance)

        self._writer.writerow([
            f'{sim_time:.9f}', f'{elapsed:.9f}',
            f'{max(0.0, (elapsed - self.start_delay) / self.time_scale):.9f}',
            f'{x:.9f}', f'{y:.9f}', f'{yaw:.9f}',
            f'{vx:.9f}', f'{vy:.9f}',
            f'{obstacle[0]:.9f}' if obstacle is not None else '',
            f'{obstacle[1]:.9f}' if obstacle is not None else '',
            f'{obstacle[2]:.9f}' if obstacle is not None else '',
            f'{robot[0]:.9f}' if robot is not None else '',
            f'{robot[1]:.9f}' if robot is not None else '',
            f'{robot[2]:.9f}' if robot is not None else '',
            f'{center_distance:.9f}' if center_distance != '' else '',
            f'{clearance:.9f}' if clearance != '' else '',
            self._model_state_samples, self._service_updates,
            self._service_failures,
        ])
        self._last_sim_s = sim_time

    def close(self):
        if self._stream.closed:
            return
        finite_clearances = [value for value in self._clearances
                             if math.isfinite(value)]
        finite_errors = [value for value in self._trajectory_errors
                         if math.isfinite(value)]
        summary = {
            'scenario_id': self.scenario.get('scenario_id', 'unknown'),
            'difficulty': self.difficulty,
            'obstacle_name': self.obstacle_name,
            'update_period_s': self.update_period,
            'time_scale': self.time_scale,
            'start_delay_s': self.start_delay,
            'sim_end_s': self._last_sim_s,
            'model_state_samples': self._model_state_samples,
            'service_updates': self._service_updates,
            'service_failures': self._service_failures,
            'groundtruth_rows_with_clearance': len(finite_clearances),
            'minimum_robot_obstacle_clearance_m': (
                min(finite_clearances) if finite_clearances else None),
            'maximum_script_to_gazebo_position_error_m': (
                max(finite_errors) if finite_errors else None),
            'source': 'deterministic waypoint schedule; not velocity extrapolation',
        }
        self._stream.close()
        self._summary_path.parent.mkdir(parents=True, exist_ok=True)
        with self._summary_path.open('w', encoding='utf-8') as stream:
            yaml.safe_dump(summary, stream, sort_keys=False)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', required=True)
    parser.add_argument('--difficulty', default='medium')
    parser.add_argument('--output', required=True)
    parser.add_argument('--summary', required=True)
    parser.add_argument('--robot-name', default='waffle')
    args = parser.parse_args()

    with open(args.scenario, encoding='utf-8') as stream:
        scenario = yaml.safe_load(stream)
    rclpy.init()
    node = DynamicObstacleController(
        scenario, args.difficulty, args.output, args.summary, args.robot_name)

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
