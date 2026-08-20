#!/usr/bin/env python3
"""Publish RTAB-Map's growing occupancy grid in a fixed-size map envelope.

Nav2's StaticLayer is useful for an online global planner because it retains
obstacles that have already been observed. In Humble, however, feeding the
growing RTAB-Map grid directly to a costmap can repeatedly resize the costmap.
This node keeps the map frame and resolution, but copies each RTAB-Map update
into a fixed envelope so StaticLayer can update in place.
"""

import math

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class FixedMapPadder(Node):
    """Copy a changing OccupancyGrid into a stable, transient-local grid."""

    def __init__(self):
        super().__init__('online_map_padder')
        self.declare_parameter('source_topic', '/map')
        self.declare_parameter('output_topic', '/nav_map')
        self.declare_parameter('width', 800)
        self.declare_parameter('height', 600)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('origin_x', -20.0)
        self.declare_parameter('origin_y', -15.0)

        source_topic = self.get_parameter('source_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.resolution = float(self.get_parameter('resolution').value)
        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)

        if self.width <= 0 or self.height <= 0 or self.resolution <= 0.0:
            raise ValueError('fixed map dimensions and resolution must be positive')

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(OccupancyGrid, output_topic, map_qos)
        self.subscription = self.create_subscription(
            OccupancyGrid, source_topic, self.map_callback, map_qos)
        self.update_count = 0
        self.last_source_shape = None

        self.get_logger().info(
            f'Padding {source_topic} into {output_topic}: '
            f'{self.width}x{self.height} at {self.resolution:.3f} m '
            f'with origin ({self.origin_x:.2f}, {self.origin_y:.2f})')

    @staticmethod
    def origin_yaw(orientation):
        return math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))

    def map_callback(self, source):
        source_width = int(source.info.width)
        source_height = int(source.info.height)
        source_resolution = float(source.info.resolution)
        if source_width <= 0 or source_height <= 0 or source_resolution <= 0.0:
            return

        target = [-1] * (self.width * self.height)
        source_origin = source.info.origin.position
        source_yaw = self.origin_yaw(source.info.origin.orientation)
        cos_yaw = math.cos(source_yaw)
        sin_yaw = math.sin(source_yaw)

        # OccupancyGrid stores row-major cells. Mapping cell centers keeps the
        # copy correct when RTAB-Map expands its origin by whole cells.
        for source_y in range(source_height):
            local_y = (source_y + 0.5) * source_resolution
            for source_x in range(source_width):
                value = int(source.data[source_y * source_width + source_x])
                if value < 0 or value > 100:
                    continue
                local_x = (source_x + 0.5) * source_resolution
                world_x = source_origin.x + cos_yaw * local_x - sin_yaw * local_y
                world_y = source_origin.y + sin_yaw * local_x + cos_yaw * local_y
                target_x = math.floor((world_x - self.origin_x) / self.resolution)
                target_y = math.floor((world_y - self.origin_y) / self.resolution)
                if not (0 <= target_x < self.width and 0 <= target_y < self.height):
                    continue
                target_index = target_y * self.width + target_x
                # Preserve the most occupied value if resampling maps several
                # source cells onto one target cell.
                if value > target[target_index]:
                    target[target_index] = value

        output = OccupancyGrid()
        output.header = source.header
        output.info.map_load_time = source.info.map_load_time
        output.info.resolution = self.resolution
        output.info.width = self.width
        output.info.height = self.height
        output.info.origin.position.x = self.origin_x
        output.info.origin.position.y = self.origin_y
        output.info.origin.position.z = 0.0
        output.info.origin.orientation.w = 1.0
        output.data = target
        self.publisher.publish(output)

        self.update_count += 1
        source_shape = (source_width, source_height)
        if source_shape != self.last_source_shape or self.update_count == 1:
            known = sum(value >= 0 for value in target)
            self.get_logger().info(
                f'Published fixed map update {self.update_count}: '
                f'source={source_width}x{source_height}, known_cells={known}')
            self.last_source_shape = source_shape


def main(args=None):
    rclpy.init(args=args)
    node = FixedMapPadder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
