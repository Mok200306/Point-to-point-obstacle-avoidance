"""ROS 2 publisher for deterministic Oracle future occupancy grids.

This node is intentionally only a future-information source.  It does not
change the Nav2 costmap, command velocity, or controller configuration.  The
dynamic obstacle pose at ``t0 + tau`` is queried directly from the scenario's
known waypoint schedule; current-velocity extrapolation is never used.
"""

import math
from pathlib import Path
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from oracle_dynamic_nav_msgs.msg import PredictedOccupancyGrid
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
import yaml

from .grid import GridSpec, load_costmap_grid_spec, rasterize_rotated_box
from .trajectory import WaypointSchedule


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def transform_xy_yaw(transform, x: float, y: float, yaw: float
                     ) -> Tuple[float, float, float]:
    """Apply a planar TransformStamped to a pose expressed in its source frame."""
    q = transform.transform.rotation
    tf_yaw = yaw_from_quaternion(q)
    c = math.cos(tf_yaw)
    s = math.sin(tf_yaw)
    return (
        transform.transform.translation.x + c * x - s * y,
        transform.transform.translation.y + s * x + c * y,
        math.atan2(math.sin(tf_yaw + yaw), math.cos(tf_yaw + yaw)),
    )


class OraclePredictionPublisher(Node):
    """Publish a time-indexed grid sourced from the known scenario schedule."""

    def __init__(self) -> None:
        super().__init__('oracle_prediction_publisher')
        self.declare_parameter('scenario_file', '')
        self.declare_parameter('difficulty', 'medium')
        self.declare_parameter('topic', '/oracle/predicted_occupancy')
        self.declare_parameter('nav2_params_file', '')
        self.declare_parameter('costmap_name', 'local_costmap')
        self.declare_parameter('frame_id', 'odom')
        # Gate 2 schedules are authored in the Gazebo world coordinate basis.
        # In this benchmark the reset odometry frame is coincident with that
        # basis, so the first interface version publishes directly in odom.
        # A later real-robot adapter must provide an explicit world/map to
        # local-costmap transform rather than silently assuming identity.
        self.declare_parameter('source_frame', 'odom')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('origin_mode', 'robot_center')
        self.declare_parameter('origin_x', 0.0)
        self.declare_parameter('origin_y', 0.0)
        self.declare_parameter('prediction_dt', 0.10)
        self.declare_parameter('prediction_horizon_s', 3.0)
        self.declare_parameter('publish_period_s', 0.10)
        self.declare_parameter('risk_padding_m', 0.0)
        self.declare_parameter('conservative_cell', True)
        self.declare_parameter('scenario_start_sim_time', -1.0)

        scenario_file = str(self.get_parameter('scenario_file').value)
        if not scenario_file:
            raise ValueError('scenario_file parameter is required')
        nav2_params_file = str(self.get_parameter('nav2_params_file').value)
        if not nav2_params_file:
            raise ValueError('nav2_params_file parameter is required')

        self.scenario_file = str(Path(scenario_file).resolve())
        self.nav2_params_file = str(Path(nav2_params_file).resolve())
        with open(self.scenario_file, encoding='utf-8') as stream:
            scenario = yaml.safe_load(stream)
        self.schedule = WaypointSchedule(
            scenario, str(self.get_parameter('difficulty').value))
        self.scenario_id = self.schedule.scenario_id

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.source_frame = str(self.get_parameter('source_frame').value)
        self.robot_base_frame = str(
            self.get_parameter('robot_base_frame').value)
        self.origin_mode = str(self.get_parameter('origin_mode').value)
        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)
        self.prediction_dt = float(self.get_parameter('prediction_dt').value)
        self.prediction_horizon_s = float(
            self.get_parameter('prediction_horizon_s').value)
        self.publish_period_s = float(self.get_parameter('publish_period_s').value)
        self.risk_padding_m = float(self.get_parameter('risk_padding_m').value)
        self.conservative_cell = bool(
            self.get_parameter('conservative_cell').value)
        if self.prediction_dt <= 0.0 or self.prediction_horizon_s < 0.0:
            raise ValueError('prediction_dt must be > 0 and horizon >= 0')
        if self.publish_period_s <= 0.0:
            raise ValueError('publish_period_s must be > 0')

        costmap_name = str(self.get_parameter('costmap_name').value)
        self.grid_spec: GridSpec = load_costmap_grid_spec(
            self.nav2_params_file, costmap_name)
        self.steps = int(round(self.prediction_horizon_s / self.prediction_dt)) + 1
        self.prediction_horizon_s = (self.steps - 1) * self.prediction_dt

        self.publisher = self.create_publisher(
            PredictedOccupancyGrid, str(self.get_parameter('topic').value), 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.odom_pose: Optional[Tuple[float, float, float]] = None
        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self.reference_sim_time_s: Optional[float] = None
        explicit_start = float(self.get_parameter('scenario_start_sim_time').value)
        if explicit_start >= 0.0:
            self.reference_sim_time_s = explicit_start
        self._logged_first_message = False
        self.timer = self.create_timer(self.publish_period_s, self._publish)

        self.get_logger().info(
            f'Gate 3 Oracle ready: scenario={self.scenario_id} '
            f'difficulty={self.get_parameter("difficulty").value} '
            f'frame={self.frame_id} source_frame={self.source_frame} '
            f'grid={self.grid_spec.width}x{self.grid_spec.height} '
            f'@ {self.grid_spec.resolution:.3f} m dt={self.prediction_dt:.3f} '
            f'steps={self.steps} horizon={self.prediction_horizon_s:.3f} s '
            f'topic={self.get_parameter("topic").value}')

    def _odom_callback(self, message: Odometry) -> None:
        pose = message.pose.pose
        self.odom_pose = (
            pose.position.x, pose.position.y,
            yaw_from_quaternion(pose.orientation),
        )

    def _lookup_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id, self.robot_base_frame, Time())
            return (
                transform.transform.translation.x,
                transform.transform.translation.y,
                yaw_from_quaternion(transform.transform.rotation),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            if self.frame_id == 'odom' and self.odom_pose is not None:
                return self.odom_pose
            return None

    def _source_to_target_transform(self):
        if self.source_frame == self.frame_id:
            return None
        try:
            return self.tf_buffer.lookup_transform(
                self.frame_id, self.source_frame, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    def _target_pose(self, x: float, y: float, yaw: float, transform):
        if transform is None:
            return x, y, yaw
        return transform_xy_yaw(transform, x, y, yaw)

    def _publish(self) -> None:
        now = self.get_clock().now()
        now_s = now.nanoseconds / 1e9
        if now_s <= 0.0:
            return
        if self.reference_sim_time_s is None:
            self.reference_sim_time_s = now_s
        elapsed_s = now_s - self.reference_sim_time_s
        if elapsed_s < 0.0:
            return

        robot = self._lookup_robot_pose()
        transform = self._source_to_target_transform()
        if robot is None or (self.source_frame != self.frame_id and transform is None):
            self.get_logger().warning(
                f'Waiting for pose/TF: frame={self.frame_id} '
                f'source_frame={self.source_frame} '
                f'robot_base={self.robot_base_frame}',
                throttle_duration_sec=5.0)
            return

        if self.origin_mode == 'robot_center':
            origin_x = robot[0] - self.grid_spec.size_x_m / 2.0
            origin_y = robot[1] - self.grid_spec.size_y_m / 2.0
        elif self.origin_mode == 'fixed':
            origin_x, origin_y = self.origin_x, self.origin_y
        else:
            raise ValueError(
                "origin_mode must be 'robot_center' or 'fixed'")

        data = []
        for index in range(self.steps):
            tau_s = index * self.prediction_dt
            x, y, yaw = self.schedule.pose_at_elapsed(elapsed_s + tau_s)
            x, y, yaw = self._target_pose(x, y, yaw, transform)
            data.extend(rasterize_rotated_box(
                self.grid_spec, origin_x, origin_y, x, y, yaw,
                self.schedule.obstacle_half_size_m,
                self.schedule.obstacle_half_size_m,
                padding_m=self.risk_padding_m,
                conservative_cell=self.conservative_cell))

        message = PredictedOccupancyGrid()
        # Each publication has its own reference t0.  The layer at index k is
        # pose_obstacle(t0 + k * dt), so consumers can use age = t_eval -
        # header.stamp when a message is delivered late.  The separate
        # reference_sim_time_s only anchors the scenario schedule's elapsed
        # time; it is not the message timestamp.
        message.header.stamp = now.to_msg()
        message.header.frame_id = self.frame_id
        message.resolution = float(self.grid_spec.resolution)
        message.width = self.grid_spec.width
        message.height = self.grid_spec.height
        message.origin = Pose()
        message.origin.position.x = origin_x
        message.origin.position.y = origin_y
        message.origin.orientation.w = 1.0
        message.dt = float(self.prediction_dt)
        message.steps = self.steps
        message.data = data
        message.source = 'oracle'
        message.footprint_half_size_m = float(
            self.schedule.obstacle_half_size_m)
        message.risk_padding_m = float(self.risk_padding_m)
        message.conservative_cell = self.conservative_cell
        self.publisher.publish(message)

        if not self._logged_first_message:
            self._logged_first_message = True
            self.get_logger().info(
                f'Published first Oracle grid at sim_t={now_s:.3f}, '
                f'reference_t0={self.reference_sim_time_s:.3f}, '
                f'origin=({origin_x:.3f}, {origin_y:.3f}), '
                f'cells={len(data)}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = OraclePredictionPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ValueError) as exc:
        if node is not None:
            node.get_logger().error(str(exc))
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
