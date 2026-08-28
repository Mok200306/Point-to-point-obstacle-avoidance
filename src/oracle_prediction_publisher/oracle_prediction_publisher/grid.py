"""Grid specification, box rasterization and alignment helpers for Gate 3."""

from dataclasses import dataclass
import math
from typing import List, Sequence, Tuple

import yaml


@dataclass(frozen=True)
class GridSpec:
    width: int
    height: int
    resolution: float

    @property
    def size_x_m(self) -> float:
        return self.width * self.resolution

    @property
    def size_y_m(self) -> float:
        return self.height * self.resolution


def load_costmap_grid_spec(path: str, costmap: str = 'local_costmap') -> GridSpec:
    """Read physical dimensions from a Nav2 costmap YAML.

    Nav2 costmap ``width`` and ``height`` are metres, while an occupancy
    message stores cell counts.  Convert them using the configured
    resolution instead of treating the metre values as cell counts.
    """
    with open(path, encoding='utf-8') as stream:
        params = yaml.safe_load(stream)
    section = params[costmap][costmap]['ros__parameters']
    resolution = float(section['resolution'])
    width = int(round(float(section['width']) / resolution))
    height = int(round(float(section['height']) / resolution))
    if width <= 0 or height <= 0 or resolution <= 0.0:
        raise ValueError(f'invalid {costmap} grid specification')
    return GridSpec(width, height, resolution)


def load_costmap_origin(path: str, costmap: str = 'global_costmap') -> Tuple[float, float]:
    """Read an optional fixed costmap origin in the same frame as the map."""
    with open(path, encoding='utf-8') as stream:
        params = yaml.safe_load(stream)
    section = params[costmap][costmap]['ros__parameters']
    return float(section.get('origin_x', 0.0)), float(section.get('origin_y', 0.0))


def point_in_rotated_box(
        x: float, y: float, cx: float, cy: float, yaw: float,
        half_x: float, half_y: float, padding_m: float = 0.0) -> bool:
    dx = x - cx
    dy = y - cy
    c = math.cos(yaw)
    s = math.sin(yaw)
    local_x = c * dx + s * dy
    local_y = -s * dx + c * dy
    return (abs(local_x) <= half_x + padding_m and
            abs(local_y) <= half_y + padding_m)


def rasterize_rotated_box(
        spec: GridSpec, origin_x: float, origin_y: float,
        center_x: float, center_y: float, yaw: float,
        half_x: float, half_y: float, padding_m: float = 0.0,
        conservative_cell: bool = True) -> List[float]:
    """Rasterize one footprint layer in row-major OccupancyGrid order.

    The optional half-cell expansion is a conservative rasterization rule. It
    bounds the center-location error by one grid cell while preserving a
    binary true-footprint layer when padding_m is zero.
    """
    cell_padding = (spec.resolution * math.sqrt(2.0) / 2.0
                    if conservative_cell else 0.0)
    effective_padding = float(padding_m) + cell_padding
    data = [0.0] * (spec.width * spec.height)
    radius = math.hypot(half_x + effective_padding,
                        half_y + effective_padding)
    min_col = max(0, int(math.floor(
        (center_x - radius - origin_x) / spec.resolution)) - 1)
    max_col = min(spec.width - 1, int(math.ceil(
        (center_x + radius - origin_x) / spec.resolution)) + 1)
    min_row = max(0, int(math.floor(
        (center_y - radius - origin_y) / spec.resolution)) - 1)
    max_row = min(spec.height - 1, int(math.ceil(
        (center_y + radius - origin_y) / spec.resolution)) + 1)
    if min_col > max_col or min_row > max_row:
        return data
    for row in range(min_row, max_row + 1):
        y = origin_y + (row + 0.5) * spec.resolution
        for col in range(min_col, max_col + 1):
            x = origin_x + (col + 0.5) * spec.resolution
            if point_in_rotated_box(
                    x, y, center_x, center_y, yaw, half_x, half_y,
                    effective_padding):
                data[row * spec.width + col] = 1.0
    return data


def occupied_cell_centroid(
        spec: GridSpec, origin_x: float, origin_y: float,
        data: Sequence[float], threshold: float = 0.5
        ) -> Tuple[float, float, int]:
    points = []
    for row in range(spec.height):
        y = origin_y + (row + 0.5) * spec.resolution
        for col in range(spec.width):
            if data[row * spec.width + col] >= threshold:
                x = origin_x + (col + 0.5) * spec.resolution
                points.append((x, y))
    if not points:
        return math.nan, math.nan, 0
    return (
        sum(point[0] for point in points) / len(points),
        sum(point[1] for point in points) / len(points),
        len(points),
    )
