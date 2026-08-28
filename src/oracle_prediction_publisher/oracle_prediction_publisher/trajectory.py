"""Pure-Python deterministic waypoint schedule used by Gate 3.

The interpolation deliberately mirrors the Gate 2 dynamic obstacle controller.
Keeping this module independent of ROS makes the schedule and grid alignment
tests runnable before a ROS workspace has been built.
"""

from dataclasses import dataclass
from typing import Any, Dict, List, Tuple


@dataclass(frozen=True)
class Waypoint:
    t: float
    x: float
    y: float
    yaw: float


class WaypointSchedule:
    """Query pose_obstacle(t) from a scenario YAML dictionary."""

    def __init__(self, scenario: Dict[str, Any], difficulty: str = 'medium'):
        self.scenario = scenario
        self.scenario_id = str(scenario.get('scenario_id', 'unknown'))
        profile = scenario.get('difficulty_profiles', {}).get(difficulty, {})
        self.time_scale = float(profile.get('time_scale', 1.0))
        self.start_delay_s = float(profile.get('start_delay_s', 0.0))
        self.update_period_s = float(scenario.get('update_period_s', 0.05))
        self.obstacle_half_size_m = float(
            scenario.get('obstacle_half_size_m', 0.30))
        self.waypoints: List[Waypoint] = [
            Waypoint(
                t=float(point['t']),
                x=float(point['x']),
                y=float(point['y']),
                yaw=float(point.get('yaw', 0.0)),
            )
            for point in scenario['waypoints']
        ]
        if len(self.waypoints) < 2:
            raise ValueError('scenario needs at least two waypoints')
        if self.time_scale <= 0.0:
            raise ValueError('difficulty time_scale must be positive')

    def pose_at_elapsed(self, elapsed_s: float) -> Tuple[float, float, float]:
        """Return x, y, yaw at seconds after the controller reference t0."""
        x, y, yaw, _, _ = self.state_at_elapsed(elapsed_s)
        return x, y, yaw

    def state_at_elapsed(self, elapsed_s: float) -> Tuple[float, float, float, float, float]:
        """Return x, y, yaw, vx, vy using the Gate 2 schedule semantics."""
        elapsed_s = float(elapsed_s)
        first = self.waypoints[0]
        if elapsed_s < self.start_delay_s:
            return first.x, first.y, first.yaw, 0.0, 0.0

        scenario_time = (elapsed_s - self.start_delay_s) / self.time_scale
        if scenario_time <= first.t:
            return first.x, first.y, first.yaw, 0.0, 0.0

        for left, right in zip(self.waypoints, self.waypoints[1:]):
            if scenario_time <= right.t:
                duration = right.t - left.t
                ratio = ((scenario_time - left.t) / duration
                         if duration > 0.0 else 1.0)
                ratio = max(0.0, min(1.0, ratio))
                x = left.x + ratio * (right.x - left.x)
                y = left.y + ratio * (right.y - left.y)
                yaw = left.yaw + ratio * (right.yaw - left.yaw)
                vx = ((right.x - left.x) / duration / self.time_scale
                      if duration > 0.0 else 0.0)
                vy = ((right.y - left.y) / duration / self.time_scale
                      if duration > 0.0 else 0.0)
                return x, y, yaw, vx, vy

        last = self.waypoints[-1]
        return last.x, last.y, last.yaw, 0.0, 0.0
