#!/usr/bin/env python3
"""ROS 2 bridge for the WATER Python SDK V5.1.

The bridge deliberately exposes only the reviewed project control path:

    /cmd_vel_safe -> WaterChassis.set_velocity() -> WATER TCP API

It does not subscribe to ``/cmd_vel`` and it does not call WATER's autonomous
``/api/move`` endpoints.  The SDK currently exposes vendor map pose and actual
velocity feedback, but not a standard wheel-encoder ROS odometry message.  For
the first integration stage this node publishes a clearly-labelled provisional
``/odom`` by integrating the returned planar velocity.  This is sufficient to
exercise the ROS graph and RTAB-Map interface, but it must be replaced or fused
with encoder odometry before claiming final physical navigation accuracy.
"""

from __future__ import annotations

import math
import json
import os
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


def _load_water_chassis(sdk_root: str):
    """Import the SDK from the installed package or an explicit source path."""
    candidates: list[Path] = []
    import_errors: list[str] = []
    if sdk_root.strip():
        candidates.append(Path(sdk_root).expanduser().resolve())

    env_root = os.environ.get("WATER_SDK_ROOT", "").strip()
    if env_root:
        candidates.append(Path(env_root).expanduser().resolve())

    # Source-tree fallback, useful before a colcon install has been rebuilt.
    script_path = Path(__file__).resolve()
    candidates.extend([
        script_path.parents[3] / "water_chassis_sdk_v5_1_cn_complete" / "water_chassis_sdk_cn_v5_1",
        script_path.parents[2] / "water_chassis_sdk_v5_1_cn_complete" / "water_chassis_sdk_cn_v5_1",
    ])

    for candidate in candidates:
        if (candidate / "water_chassis_sdk").is_dir():
            sys.path.insert(0, str(candidate))
            try:
                from water_chassis_sdk import WaterChassis
                return WaterChassis
            except ImportError as error:
                import_errors.append(f"{candidate}: {error}")
                continue

    searched = ", ".join(str(path) for path in candidates)
    detail = "；导入错误：" + " | ".join(import_errors) if import_errors else ""
    raise ImportError(
        "无法导入 WATER V5.1 SDK；请确认 water_sdk_root 指向 "
        f"water_chassis_sdk_cn_v5_1。已搜索：{searched}{detail}"
    )


def _finite(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def _normalise_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class WaterChassisRosBridge(Node):
    """Own one SDK connection and bridge the safe ROS velocity/odom contract."""

    def __init__(self) -> None:
        super().__init__("water_chassis_ros_bridge")

        self.declare_parameter("sdk_root", "")
        self.declare_parameter("config_path", "")
        self.declare_parameter("robot_host", "192.168.10.10")
        self.declare_parameter("robot_port", 31001)
        self.declare_parameter("gateway_port", 8080)
        self.declare_parameter("connect_timeout_s", 12.0)
        self.declare_parameter("auto_start_gateway", True)
        self.declare_parameter("enable_motion", False)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_safe")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("command_rate_hz", 10.0)
        self.declare_parameter("state_rate_hz", 5.0)
        self.declare_parameter("odom_rate_hz", 20.0)
        self.declare_parameter("command_timeout_s", 0.25)
        self.declare_parameter("state_timeout_s", 1.2)
        self.declare_parameter("max_linear_velocity", 0.12)
        self.declare_parameter("max_angular_velocity", 0.35)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("allow_provisional_odom", False)
        self.declare_parameter("status_topic", "/water_chassis/status")

        self.sdk_root = str(self.get_parameter("sdk_root").value)
        config_path = str(self.get_parameter("config_path").value).strip()
        robot_host = str(self.get_parameter("robot_host").value)
        robot_port = int(self.get_parameter("robot_port").value)
        gateway_port = int(self.get_parameter("gateway_port").value)
        connect_timeout_s = float(self.get_parameter("connect_timeout_s").value)
        auto_start_gateway = bool(self.get_parameter("auto_start_gateway").value)
        self.enable_motion = bool(self.get_parameter("enable_motion").value)

        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.command_rate_hz = max(2.0, float(self.get_parameter("command_rate_hz").value))
        self.state_rate_hz = max(1.0, float(self.get_parameter("state_rate_hz").value))
        self.odom_rate_hz = max(5.0, float(self.get_parameter("odom_rate_hz").value))
        self.command_timeout_s = max(0.05, float(self.get_parameter("command_timeout_s").value))
        self.state_timeout_s = max(0.2, float(self.get_parameter("state_timeout_s").value))
        self.max_linear_velocity = abs(float(self.get_parameter("max_linear_velocity").value))
        self.max_angular_velocity = abs(float(self.get_parameter("max_angular_velocity").value))
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.allow_provisional_odom = bool(
            self.get_parameter("allow_provisional_odom").value)
        self.status_topic = str(self.get_parameter("status_topic").value)

        if not self.cmd_vel_topic.startswith("/") or not self.odom_topic.startswith("/"):
            raise ValueError("cmd_vel_topic 和 odom_topic 必须是绝对 ROS topic")
        if self.cmd_vel_topic != "/cmd_vel_safe":
            raise ValueError(
                "WATER bridge 只允许订阅 /cmd_vel_safe，"
                f"拒绝配置 {self.cmd_vel_topic!r}")
        if self.max_linear_velocity <= 0.0 or self.max_angular_velocity <= 0.0:
            raise ValueError("底盘速度上限必须为正数")

        WaterChassis = _load_water_chassis(self.sdk_root)
        self.get_logger().info(
            f"正在连接 WATER II-S SDK {robot_host}:{robot_port}；"
            f"安全速度入口={self.cmd_vel_topic}，里程计={self.odom_topic}"
        )
        kwargs: dict[str, Any] = {
            "robot_host": robot_host,
            "robot_port": robot_port,
            "gateway_port": gateway_port,
            "connect_timeout_s": connect_timeout_s,
            "auto_start_gateway": auto_start_gateway,
            "stop_on_close": self.enable_motion,
            "feedback": False,
        }
        if config_path:
            kwargs["config_path"] = config_path
        self.robot = WaterChassis(**kwargs)

        self._sdk_lock = threading.RLock()
        self._cmd_lock = threading.Lock()
        self._latest_linear = 0.0
        self._latest_angular = 0.0
        self._last_cmd_monotonic: float | None = None
        self._last_command_sent = (0.0, 0.0)
        self._last_command_error_log = 0.0

        self._actual_linear = 0.0
        self._actual_angular = 0.0
        self._state_ready = False
        self._state_online = False
        self._state_last_update: float | None = None
        self._last_state_error_log = 0.0
        self._last_blocked_command_log = 0.0
        self._vendor_pose: dict[str, Any] = {}
        self._closed = False

        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._odom_last_monotonic = time.monotonic()

        self.cmd_subscription = self.create_subscription(
            Twist, self.cmd_vel_topic, self._cmd_vel_callback, 10)
        self.odom_publisher = (
            self.create_publisher(Odometry, self.odom_topic, 20)
            if self.allow_provisional_odom else None
        )
        self.status_publisher = self.create_publisher(String, self.status_topic, 10)
        self.tf_broadcaster = (
            TransformBroadcaster(self)
            if self.publish_tf and self.allow_provisional_odom else None
        )

        self.command_timer = self.create_timer(
            1.0 / self.command_rate_hz, self._command_tick)
        self.state_timer = self.create_timer(
            1.0 / self.state_rate_hz, self._state_tick)
        self.odom_timer = self.create_timer(
            1.0 / self.odom_rate_hz, self._odom_tick)

        self.get_logger().info(
            "WATER bridge 已启动：运动放行=%s，暂定 /odom=%s；"
            "正式验收仍需真实编码器里程计或 robot_localization 融合。"
            % (self.enable_motion, self.allow_provisional_odom)
        )
        if not self.enable_motion:
            self.get_logger().warning(
                "enable_motion=false：桥接节点只做状态连接，"
                "不会向 WATER 发送速度命令。"
            )
        elif not self.allow_provisional_odom:
            self.get_logger().info(
                "enable_motion=true 且 allow_provisional_odom=false："
                "使用外部真实 /odom，bridge 不发布 /odom/TF。"
            )

    def _cmd_vel_callback(self, message: Twist) -> None:
        linear = _finite(message.linear.x)
        angular = _finite(message.angular.z)
        with self._cmd_lock:
            self._latest_linear = linear
            self._latest_angular = angular
            self._last_cmd_monotonic = time.monotonic()

    def _state_tick(self) -> None:
        try:
            with self._sdk_lock:
                state = self.robot.get_state(refresh=False)
            connection = state.get("connection") or {}
            safety = state.get("safety") or {}
            velocity = state.get("velocity") or {}
            pose = state.get("pose") or {}
            status_age_ms = connection.get("status_age_ms")
            velocity_age_ms = connection.get("velocity_age_ms")
            status_fresh = (
                status_age_ms is not None
                and _finite(status_age_ms, self.state_timeout_s * 1000.0)
                <= self.state_timeout_s * 1000.0
            )
            velocity_fresh = (
                velocity_age_ms is not None
                and _finite(velocity_age_ms, self.state_timeout_s * 1000.0)
                <= self.state_timeout_s * 1000.0
            )
            online = bool(connection.get("online"))
            ready = bool(state.get("ready_to_move"))
            if safety.get("estop") is True or safety.get("fault") is True:
                ready = False

            actual_linear = _finite(velocity.get("linear_mps"))
            actual_angular = _finite(velocity.get("angular_rps"))
            if not online or not status_fresh or not velocity_fresh:
                actual_linear = 0.0
                actual_angular = 0.0

            with self._cmd_lock:
                self._actual_linear = actual_linear
                self._actual_angular = actual_angular
                self._state_online = online
                self._state_ready = ready and status_fresh and velocity_fresh
                self._state_last_update = time.monotonic()
                self._vendor_pose = pose
            self._publish_status(state)
        except Exception as error:  # SDK/network failures must fail closed.
            with self._cmd_lock:
                self._actual_linear = 0.0
                self._actual_angular = 0.0
                self._state_online = False
                self._state_ready = False
            now = time.monotonic()
            if now - self._last_state_error_log >= 2.0:
                self._last_state_error_log = now
                self.get_logger().error(
                    f"WATER 状态读取失败，已将速度和里程计置零：{error}")
            self._publish_status({
                "bridge_error": str(error),
                "connection": {"online": False},
                "safety": {"estop": None, "fault": True},
            })

    def _publish_status(self, state: dict[str, Any]) -> None:
        with self._cmd_lock:
            command_age = (
                None if self._last_cmd_monotonic is None
                else max(0.0, time.monotonic() - self._last_cmd_monotonic)
            )
            bridge = {
                "enable_motion": self.enable_motion,
                "allow_provisional_odom": self.allow_provisional_odom,
                "state_online": self._state_online,
                "state_ready": self._state_ready,
                "command_age_s": command_age,
                "last_command_sent": {
                    "linear_mps": self._last_command_sent[0],
                    "angular_rps": self._last_command_sent[1],
                },
                "provisional_odom_source": (
                    "sdk_robot_velocity_integrated"
                    if self.allow_provisional_odom else None
                ),
            }
        message = String()
        message.data = json.dumps(
            {"bridge": bridge, "chassis": state},
            ensure_ascii=False,
            separators=(",", ":"),
            default=str,
        )
        self.status_publisher.publish(message)

    def _command_tick(self) -> None:
        # The default bridge mode is diagnostics-only.  Do not even send a
        # zero velocity frame in this mode: it must not take control away from
        # a vendor diagnostic session or an unreviewed external controller.
        if not self.enable_motion:
            return

        now = time.monotonic()
        with self._cmd_lock:
            requested_linear = self._latest_linear
            requested_angular = self._latest_angular
            last_cmd = self._last_cmd_monotonic
            state_ready = self._state_ready
            state_online = self._state_online
            state_update = self._state_last_update

        command_is_fresh = last_cmd is not None and now - last_cmd <= self.command_timeout_s
        state_is_fresh = state_update is not None and now - state_update <= self.state_timeout_s
        allowed = (
            self.enable_motion
            and command_is_fresh
            and state_ready
            and state_online
            and state_is_fresh
        )

        if allowed:
            if (
                abs(requested_linear) > self.max_linear_velocity + 1e-6
                or abs(requested_angular) > self.max_angular_velocity + 1e-6
            ):
                allowed = False
                now_log = time.monotonic()
                if now_log - self._last_command_error_log >= 2.0:
                    self._last_command_error_log = now_log
                    self.get_logger().error(
                        "收到超出 bridge 安全上限的 /cmd_vel_safe，已拒绝："
                        "v=%.3f (上限 %.3f), w=%.3f (上限 %.3f)"
                        % (requested_linear, self.max_linear_velocity,
                           requested_angular, self.max_angular_velocity)
                    )

        linear = requested_linear if allowed else 0.0
        angular = requested_angular if allowed else 0.0

        try:
            with self._sdk_lock:
                self.robot.set_velocity(linear, angular)
            with self._cmd_lock:
                self._last_command_sent = (linear, angular)
        except Exception as error:
            with self._cmd_lock:
                self._last_command_sent = (0.0, 0.0)
            now_log = time.monotonic()
            if now_log - self._last_command_error_log >= 2.0:
                self._last_command_error_log = now_log
                self.get_logger().error(
                    f"WATER 速度命令失败，底盘应保持停止：{error}")

    def _odom_tick(self) -> None:
        if not self.allow_provisional_odom:
            return

        now_monotonic = time.monotonic()
        dt = max(0.0, now_monotonic - self._odom_last_monotonic)
        self._odom_last_monotonic = now_monotonic
        # Avoid integrating an arbitrary jump after a paused/debugged process.
        dt = min(dt, 0.2)

        with self._cmd_lock:
            linear = self._actual_linear
            angular = self._actual_angular
            state_fresh = (
                self._state_last_update is not None
                and now_monotonic - self._state_last_update <= self.state_timeout_s
            )

        if not state_fresh:
            linear = 0.0
            angular = 0.0

        if abs(angular) > 1e-8:
            next_yaw = self._odom_yaw + angular * dt
            radius = linear / angular
            self._odom_x += radius * (math.sin(next_yaw) - math.sin(self._odom_yaw))
            self._odom_y += -radius * (math.cos(next_yaw) - math.cos(self._odom_yaw))
            self._odom_yaw = _normalise_angle(next_yaw)
        else:
            self._odom_x += linear * math.cos(self._odom_yaw) * dt
            self._odom_y += linear * math.sin(self._odom_yaw) * dt

        stamp = self.get_clock().now().to_msg()
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x = self._odom_x
        message.pose.pose.position.y = self._odom_y
        message.pose.pose.orientation.z = math.sin(self._odom_yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(self._odom_yaw / 2.0)
        message.twist.twist.linear.x = linear
        message.twist.twist.angular.z = angular

        # This is integrated SDK velocity, not encoder odometry.  Keep the
        # covariance conservative so downstream consumers do not mistake it
        # for a calibrated wheel/IMU estimate.
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = 0.16
        message.twist.covariance[0] = 0.04
        message.twist.covariance[35] = 0.04
        assert self.odom_publisher is not None
        self.odom_publisher.publish(message)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self._odom_x
            transform.transform.translation.y = self._odom_y
            transform.transform.rotation = message.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

    def close(self) -> None:
        """Stop the chassis and close only the SDK resources owned by this node."""
        if self._closed:
            return
        self._closed = True
        try:
            with self._sdk_lock:
                # WaterChassis.close() applies the stop_on_close policy that
                # was selected when this bridge connected.
                self.robot.close()
        except Exception as error:
            self.get_logger().error(f"WATER SDK 资源关闭失败：{error}")

    def destroy_node(self):  # type: ignore[override]
        self.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None

    def request_shutdown(_signum, _frame) -> None:
        """Turn supervisor termination into the normal cleanup path.

        ``ros2 launch`` normally forwards SIGINT, but Docker/systemd/supervisor
        shutdowns may deliver SIGTERM directly to this executable.  Letting
        Python's default SIGTERM handler terminate the process would bypass
        ``destroy_node()`` and could leave the SDK-owned Gateway alive.
        """
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGTERM, request_shutdown)
    signal.signal(signal.SIGINT, request_shutdown)
    try:
        node = WaterChassisRosBridge()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
