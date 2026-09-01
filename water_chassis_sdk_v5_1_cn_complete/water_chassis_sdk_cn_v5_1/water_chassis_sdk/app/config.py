from __future__ import annotations

import os
from dataclasses import dataclass

from water_config import deep_get, load_config


def _env_float(name: str, default: float) -> float:
    value = os.getenv(name)
    return default if value is None else float(value)


def _env_int(name: str, default: int) -> int:
    value = os.getenv(name)
    return default if value is None else int(value)


def _env_str(name: str, default: str) -> str:
    value = os.getenv(name)
    return default if value is None else value


@dataclass(frozen=True)
class Settings:
    robot_host: str
    robot_port: int
    robot_connect_timeout_s: float
    robot_request_timeout_s: float
    robot_reconnect_delay_s: float

    status_frequency_hz: float
    velocity_frequency_hz: float
    power_poll_period_s: float
    pose_stale_after_s: float
    stream_frequency_hz: float

    direct_max_linear_mps: float
    direct_max_angular_rps: float
    direct_watchdog_s: float
    direct_command_rate_hz: float

    rotate_max_angular_rps: float
    rotate_tolerance_rad: float
    rotate_kp: float
    rotate_min_angular_rps: float
    rotate_timeout_factor: float
    rotate_timeout_extra_s: float

    distance_speed_mps: float
    distance_tolerance_m: float
    distance_heading_hold: bool
    distance_speed_kp: float
    distance_min_speed_mps: float
    distance_heading_kp: float
    distance_max_heading_correction_rps: float
    distance_timeout_factor: float
    distance_timeout_extra_s: float

    default_distance_tolerance_m: float
    default_yaw_tolerance_rad: float
    default_task_timeout_s: float
    default_max_continuous_retries: int

    gateway_host: str
    gateway_port: int


def load_settings() -> Settings:
    c = load_config()
    import math

    robot_host = _env_str("ROBOT_HOST", str(deep_get(c, "robot.host", "192.168.10.10")))
    robot_port = _env_int("ROBOT_PORT", int(deep_get(c, "robot.port", 31001)))

    return Settings(
        robot_host=robot_host,
        robot_port=robot_port,
        robot_connect_timeout_s=_env_float("ROBOT_CONNECT_TIMEOUT_S", float(deep_get(c, "robot.connect_timeout_s", 3.0))),
        robot_request_timeout_s=_env_float("ROBOT_REQUEST_TIMEOUT_S", float(deep_get(c, "robot.request_timeout_s", 3.0))),
        robot_reconnect_delay_s=_env_float("ROBOT_RECONNECT_DELAY_S", float(deep_get(c, "robot.reconnect_delay_s", 1.0))),
        status_frequency_hz=_env_float("ROBOT_STATUS_HZ", float(deep_get(c, "state.status_frequency_hz", 5.0))),
        velocity_frequency_hz=_env_float("ROBOT_VELOCITY_HZ", float(deep_get(c, "state.velocity_frequency_hz", 10.0))),
        power_poll_period_s=_env_float("ROBOT_POWER_POLL_S", float(deep_get(c, "state.power_poll_period_s", 5.0))),
        pose_stale_after_s=_env_float("POSE_STALE_AFTER_S", float(deep_get(c, "state.pose_stale_after_s", 1.0))),
        stream_frequency_hz=float(deep_get(c, "state.stream_frequency_hz", 5.0)),
        direct_max_linear_mps=_env_float("DIRECT_MAX_LINEAR_MPS", float(deep_get(c, "direct.max_linear_mps", 0.35))),
        direct_max_angular_rps=_env_float("DIRECT_MAX_ANGULAR_RPS", float(deep_get(c, "direct.max_angular_rps", 0.70))),
        direct_watchdog_s=_env_float("DIRECT_WATCHDOG_S", float(deep_get(c, "direct.watchdog_s", 0.35))),
        direct_command_rate_hz=float(deep_get(c, "direct.command_rate_hz", 10.0)),
        rotate_max_angular_rps=float(deep_get(c, "direct.rotation.max_angular_rps", 0.30)),
        rotate_tolerance_rad=math.radians(float(deep_get(c, "direct.rotation.tolerance_deg", 2.0))),
        rotate_kp=float(deep_get(c, "direct.rotation.kp", 1.8)),
        rotate_min_angular_rps=float(deep_get(c, "direct.rotation.min_angular_rps", 0.08)),
        rotate_timeout_factor=float(deep_get(c, "direct.rotation.timeout_factor", 2.5)),
        rotate_timeout_extra_s=float(deep_get(c, "direct.rotation.timeout_extra_s", 2.0)),
        distance_speed_mps=float(deep_get(c, "direct.distance.speed_mps", 0.10)),
        distance_tolerance_m=float(deep_get(c, "direct.distance.tolerance_m", 0.015)),
        distance_heading_hold=bool(deep_get(c, "direct.distance.heading_hold", True)),
        distance_speed_kp=float(deep_get(c, "direct.distance.speed_kp", 0.85)),
        distance_min_speed_mps=float(deep_get(c, "direct.distance.min_speed_mps", 0.035)),
        distance_heading_kp=float(deep_get(c, "direct.distance.heading_kp", 1.6)),
        distance_max_heading_correction_rps=float(deep_get(c, "direct.distance.max_heading_correction_rps", 0.20)),
        distance_timeout_factor=float(deep_get(c, "direct.distance.timeout_factor", 2.8)),
        distance_timeout_extra_s=float(deep_get(c, "direct.distance.timeout_extra_s", 3.0)),
        default_distance_tolerance_m=float(deep_get(c, "navigation.distance_tolerance_m", 0.05)),
        default_yaw_tolerance_rad=math.radians(float(deep_get(c, "navigation.yaw_tolerance_deg", 6.0))),
        default_task_timeout_s=float(deep_get(c, "navigation.task_timeout_s", 60.0)),
        default_max_continuous_retries=int(deep_get(c, "navigation.max_continuous_retries", 30)),
        gateway_host=_env_str("GATEWAY_HOST", str(deep_get(c, "gateway.host", "127.0.0.1"))),
        gateway_port=_env_int("GATEWAY_PORT", int(deep_get(c, "gateway.port", 8080))),
    )


settings = load_settings()
