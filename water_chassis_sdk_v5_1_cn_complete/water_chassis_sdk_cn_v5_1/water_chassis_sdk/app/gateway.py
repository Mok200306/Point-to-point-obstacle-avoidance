from __future__ import annotations

import asyncio
import copy
import logging
import math
import time
import uuid
from dataclasses import dataclass
from typing import Any, Optional

from .config import Settings
from .models import (
    DirectDistanceRequest, DirectRelativeRequest, DirectRotateRequest, MapGoalRequest,
    NavParamsRequest, PlanDistanceRequest, RelativeMotionRequest, TaskRecordModel,
    VelocityCommandRequest,
)
from .vendor import VendorConnectionError, YunjiClient

logger = logging.getLogger(__name__)


def now_ms() -> int:
    return int(time.time() * 1000)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class GatewayError(RuntimeError):
    def __init__(self, code: str, message: str, http_status: int = 400, details: Optional[dict] = None):
        super().__init__(message)
        self.code = code
        self.message = message
        self.http_status = http_status
        self.details = details or {}


@dataclass
class _TaskRuntime:
    model: TaskRecordModel
    accepted_monotonic: Optional[float] = None


class RobotGateway:
    def __init__(self, settings: Settings):
        self.settings = settings
        self.vendor = YunjiClient(settings)
        self.vendor.set_packet_handler(self._handle_vendor_packet)
        self.vendor.set_connected_handler(self._on_vendor_connected)

        self.vendor_status: dict[str, Any] = {}
        self.velocity: dict[str, Any] = {"linear": 0.0, "angular": 0.0}
        self.power: dict[str, Any] = {}
        self.last_notification: Optional[dict[str, Any]] = None
        self.notifications: list[dict[str, Any]] = []

        self.last_status_monotonic: Optional[float] = None
        self.last_velocity_monotonic: Optional[float] = None
        self.last_power_monotonic: Optional[float] = None

        self.control_mode: str = "IDLE"  # IDLE / NAV / DIRECT / ESTOP
        self.active_task_id: Optional[str] = None
        self.tasks: dict[str, _TaskRuntime] = {}

        self.last_cmd_vel_monotonic: Optional[float] = None
        self.direct_source: Optional[str] = None
        self.direct_action: Optional[dict[str, Any]] = None
        self._direct_cancel_seq: int = 0
        self._watchdog_stop_sent = False

        self._housekeeping_task: Optional[asyncio.Task] = None
        self._command_lock = asyncio.Lock()

    async def start(self) -> None:
        await self.vendor.start()
        self._housekeeping_task = asyncio.create_task(self._housekeeping_loop(), name="gateway-housekeeping")

    async def stop(self) -> None:
        if self._housekeeping_task:
            self._housekeeping_task.cancel()
        await self.vendor.stop()

    async def _on_vendor_connected(self) -> None:
        # Subscribe after every reconnect. Failure is non-fatal because the fallback poller remains active.
        try:
            r1 = await self.vendor.subscribe_status(self.settings.status_frequency_hz)
            if r1.get("status") != "OK":
                logger.warning("robot_status subscription rejected: %s", r1)
        except Exception as exc:
            logger.warning("robot_status subscription failed: %r", exc)

        try:
            r2 = await self.vendor.subscribe_velocity(self.settings.velocity_frequency_hz)
            if r2.get("status") != "OK":
                logger.warning("robot_velocity subscription rejected: %s", r2)
        except Exception as exc:
            logger.warning("robot_velocity subscription failed: %r", exc)

        # Seed caches immediately.
        try:
            await self._poll_status_once()
        except Exception:
            pass
        try:
            await self._poll_power_once()
        except Exception:
            pass

    async def _handle_vendor_packet(self, packet: dict[str, Any]) -> None:
        pkt_type = packet.get("type")
        if pkt_type == "callback":
            topic = packet.get("topic")
            if topic == "robot_status":
                self._update_robot_status(packet.get("results") or {})
            elif topic == "robot_velocity":
                self._update_velocity(packet.get("results") or {})
        elif pkt_type == "notification":
            self.last_notification = copy.deepcopy(packet)
            self.notifications.append(copy.deepcopy(packet))
            if len(self.notifications) > 50:
                self.notifications = self.notifications[-50:]
        elif pkt_type == "response":
            command = packet.get("command")
            if packet.get("status") == "OK" and command == "/api/robot_status":
                self._update_robot_status(packet.get("results") or {})
            elif packet.get("status") == "OK" and command == "/api/get_power_status":
                self._update_power(packet.get("results") or {})

    def _update_robot_status(self, results: dict[str, Any]) -> None:
        if not isinstance(results, dict):
            return
        self.vendor_status = copy.deepcopy(results)
        self.last_status_monotonic = time.monotonic()

        if results.get("estop_state") is True:
            self.control_mode = "ESTOP"

        self._update_active_task_from_vendor_status(results)

    def _update_velocity(self, results: dict[str, Any]) -> None:
        if not isinstance(results, dict):
            return
        self.velocity = {
            "linear": float(results.get("linear", 0.0) or 0.0),
            "angular": float(results.get("angular", 0.0) or 0.0),
        }
        self.last_velocity_monotonic = time.monotonic()

    def _update_power(self, results: dict[str, Any]) -> None:
        if not isinstance(results, dict):
            return
        self.power = copy.deepcopy(results)
        self.last_power_monotonic = time.monotonic()

    def _update_active_task_from_vendor_status(self, results: dict[str, Any]) -> None:
        task_id = self.active_task_id
        if not task_id:
            return
        runtime = self.tasks.get(task_id)
        if not runtime:
            self.active_task_id = None
            return
        task = runtime.model
        if task.state not in ("SUBMITTING", "ACCEPTED", "EXECUTING"):
            return

        move_status = str(results.get("move_status") or "")
        ts = now_ms()
        if move_status == "running":
            if task.state != "EXECUTING":
                task.state = "EXECUTING"
                task.started_at_ms = task.started_at_ms or ts
            task.has_seen_running = True
            return

        if move_status == "failed":
            self._finish_task(task, "FAILED", {"code": "NAVIGATION_FAILED", "message": "Vendor move_status=failed"})
            return

        if move_status == "canceled":
            self._finish_task(task, "CANCELED", None)
            return

        if move_status == "succeeded":
            # Protect against a stale succeeded status from the previous task. Normally we require
            # observing running. For very short moves that finish between callbacks, accept success
            # only if enough time has elapsed and the robot is actually near the requested target.
            if task.has_seen_running:
                self._finish_task(task, "SUCCEEDED", None)
                return

            if runtime.accepted_monotonic is not None and time.monotonic() - runtime.accepted_monotonic >= 0.5:
                remaining = self._euclidean_distance_to_target(task.target)
                tolerance = float(task.target.get("distance_tolerance_m", self.settings.default_distance_tolerance_m))
                if remaining is not None and remaining <= max(tolerance + 0.15, 0.25):
                    self._finish_task(task, "SUCCEEDED", None)

    def _finish_task(self, task: TaskRecordModel, state: str, error: Optional[dict]) -> None:
        task.state = state  # type: ignore[assignment]
        task.finished_at_ms = now_ms()
        task.error = error
        if self.active_task_id == task.task_id:
            self.active_task_id = None
        if self.control_mode == "NAV":
            self.control_mode = "ESTOP" if self.vendor_status.get("estop_state") else "IDLE"

    async def _housekeeping_loop(self) -> None:
        last_power_poll = 0.0
        while True:
            try:
                await asyncio.sleep(0.05)
                now_mono = time.monotonic()

                # Direct-control watchdog: never replay the last command. If planner messages stop,
                # explicitly send a zero command and return to IDLE.
                if self.control_mode == "DIRECT" and self.last_cmd_vel_monotonic is not None:
                    if now_mono - self.last_cmd_vel_monotonic > self.settings.direct_watchdog_s and not self._watchdog_stop_sent:
                        self._watchdog_stop_sent = True
                        try:
                            await self._send_joy(0.0, 0.0)
                        except Exception:
                            pass
                        if self.control_mode == "DIRECT":
                            self.control_mode = "IDLE"
                        self.direct_source = None

                # Task timeout.
                if self.active_task_id:
                    runtime = self.tasks.get(self.active_task_id)
                    if runtime and runtime.accepted_monotonic is not None:
                        if now_mono - runtime.accepted_monotonic > runtime.model.timeout_s:
                            timed_out_task = runtime.model
                            try:
                                await self.vendor.send_command("/api/move/cancel")
                            except Exception:
                                pass
                            self._finish_task(
                                timed_out_task,
                                "FAILED",
                                {"code": "TASK_TIMEOUT", "message": "Navigation task exceeded timeout"},
                            )

                # Fallback status poll if callbacks are absent/stale.
                if self.vendor.connected:
                    stale = self.last_status_monotonic is None or (now_mono - self.last_status_monotonic > 1.2)
                    if stale:
                        try:
                            await self._poll_status_once()
                        except Exception:
                            pass

                    if now_mono - last_power_poll >= self.settings.power_poll_period_s:
                        last_power_poll = now_mono
                        try:
                            await self._poll_power_once()
                        except Exception:
                            pass
            except asyncio.CancelledError:
                break
            except Exception:
                logger.exception("Housekeeping loop error")

    async def _poll_status_once(self) -> None:
        resp = await self.vendor.request_status()
        if resp.get("status") == "OK":
            self._update_robot_status(resp.get("results") or {})

    async def _poll_power_once(self) -> None:
        resp = await self.vendor.request_power()
        if resp.get("status") == "OK":
            self._update_power(resp.get("results") or {})

    def _assert_command_fresh(self, generated_at_ms: Optional[int], max_age_ms: int) -> None:
        if generated_at_ms is None:
            return
        age = now_ms() - generated_at_ms
        if age > max_age_ms:
            raise GatewayError(
                "STALE_COMMAND",
                f"Command is {age} ms old, max allowed age is {max_age_ms} ms",
                409,
                {"age_ms": age, "max_age_ms": max_age_ms},
            )
        if age < -5000:
            raise GatewayError("INVALID_TIMESTAMP", "generated_at_ms 时间戳超前过多", 400)

    def _assert_online(self) -> None:
        if not self.vendor.connected:
            raise GatewayError("CHASSIS_OFFLINE", "WATER 底盘 TCP 连接已离线", 503)

    def _assert_safe_to_move(self) -> None:
        self._assert_online()
        if self.vendor_status.get("estop_state") is True:
            raise GatewayError("ESTOP_ACTIVE", "急停当前处于激活状态", 423)
        error_code = str(self.vendor_status.get("error_code") or "00000000")
        if error_code not in ("", "0", "00000000"):
            raise GatewayError("FAULT_ACTIVE", "底盘报告了非零 error_code", 423, {"error_code": error_code})

    def _assert_pose_fresh(self) -> dict[str, float]:
        pose = self.vendor_status.get("current_pose")
        if not isinstance(pose, dict) or not all(k in pose for k in ("x", "y", "theta")):
            raise GatewayError("POSE_UNAVAILABLE", "当前地图位姿不可用", 503)
        if self.last_status_monotonic is None or time.monotonic() - self.last_status_monotonic > self.settings.pose_stale_after_s:
            raise GatewayError("POSE_STALE", "当前地图位姿已过期", 503)
        return {"x": float(pose["x"]), "y": float(pose["y"]), "theta": float(pose["theta"])}

    def _busy_reason(self) -> Optional[str]:
        if self.control_mode in ("NAV", "DIRECT"):
            return self.control_mode
        vendor_move_status = str(self.vendor_status.get("move_status") or "")
        if vendor_move_status == "running" and self.active_task_id is None:
            return "EXTERNAL_NAV"
        return None

    async def _replace_or_reject(self, replace_current: bool) -> None:
        busy = self._busy_reason()
        if not busy:
            return
        if not replace_current:
            raise GatewayError("CHASSIS_BUSY", f"底盘当前忙碌（{busy}）", 409, {"mode": busy})
        await self.stop_current(reason="replaced by a newer command")
        await asyncio.sleep(0.05)

    async def move_relative(self, req: RelativeMotionRequest) -> TaskRecordModel:
        self._assert_command_fresh(req.generated_at_ms, req.max_age_ms)
        self._assert_safe_to_move()
        pose = self._assert_pose_fresh()

        if req.heading_frame == "base_link":
            travel_yaw = normalize_angle(pose["theta"] + req.heading_rad)
        else:
            travel_yaw = normalize_angle(req.heading_rad)

        x_goal = pose["x"] + req.distance_m * math.cos(travel_yaw)
        y_goal = pose["y"] + req.distance_m * math.sin(travel_yaw)

        if req.final_yaw_rad is None:
            yaw_goal = travel_yaw
        elif req.final_yaw_frame == "base_link":
            yaw_goal = normalize_angle(pose["theta"] + req.final_yaw_rad)
        else:
            yaw_goal = normalize_angle(req.final_yaw_rad)

        map_req = MapGoalRequest(
            request_id=req.request_id,
            x_m=x_goal,
            y_m=y_goal,
            yaw_rad=yaw_goal,
            distance_tolerance_m=req.distance_tolerance_m,
            yaw_tolerance_rad=req.yaw_tolerance_rad,
            timeout_s=req.timeout_s,
            max_continuous_retries=req.max_continuous_retries,
            replace_current=req.replace_current,
            generated_at_ms=req.generated_at_ms,
            max_age_ms=req.max_age_ms,
        )
        return await self._move_map_internal(map_req, kind="relative", extra_target={
            "source_pose": pose,
            "heading_rad": req.heading_rad,
            "heading_frame": req.heading_frame,
            "distance_m": req.distance_m,
            "travel_yaw_rad": travel_yaw,
        })

    async def move_map(self, req: MapGoalRequest) -> TaskRecordModel:
        self._assert_command_fresh(req.generated_at_ms, req.max_age_ms)
        self._assert_safe_to_move()
        return await self._move_map_internal(req, kind="map_goal", extra_target={})

    async def _move_map_internal(self, req: MapGoalRequest, kind: str, extra_target: dict[str, Any]) -> TaskRecordModel:
        # Replacement may need to call stop_current(), which takes the command lock itself.
        await self._replace_or_reject(req.replace_current)
        async with self._command_lock:
            # Re-check after acquiring the lock in case another caller won the race.
            busy = self._busy_reason()
            if busy:
                raise GatewayError("CHASSIS_BUSY", f"底盘当前忙碌（{busy}）", 409, {"mode": busy})
            self._assert_safe_to_move()

            request_id = req.request_id or uuid.uuid4().hex
            task_id = uuid.uuid4().hex
            timeout_s = req.timeout_s or self.settings.default_task_timeout_s
            distance_tolerance = req.distance_tolerance_m or self.settings.default_distance_tolerance_m
            yaw_tolerance = req.yaw_tolerance_rad or self.settings.default_yaw_tolerance_rad
            retries = self.settings.default_max_continuous_retries if req.max_continuous_retries is None else req.max_continuous_retries

            target = {
                "frame": "map",
                "x_m": req.x_m,
                "y_m": req.y_m,
                "yaw_rad": normalize_angle(req.yaw_rad),
                "distance_tolerance_m": distance_tolerance,
                "yaw_tolerance_rad": yaw_tolerance,
                **extra_target,
            }
            task = TaskRecordModel(
                task_id=task_id,
                request_id=request_id,
                state="SUBMITTING",
                kind=kind,  # type: ignore[arg-type]
                created_at_ms=now_ms(),
                timeout_s=timeout_s,
                target=target,
            )
            runtime = _TaskRuntime(model=task)
            self.tasks[task_id] = runtime

            command = (
                f"/api/move?location={req.x_m:.6f},{req.y_m:.6f},{normalize_angle(req.yaw_rad):.6f}"
                f"&distance_tolerance={distance_tolerance:.6f}"
                f"&theta_tolerance={yaw_tolerance:.6f}"
                f"&max_continuous_retries={retries}"
            )
            try:
                resp = await self.vendor.send_command(command)
            except VendorConnectionError as exc:
                self._finish_task(task, "FAILED", {"code": "CHASSIS_OFFLINE", "message": str(exc)})
                raise GatewayError("CHASSIS_OFFLINE", str(exc), 503) from exc
            except Exception as exc:
                self._finish_task(task, "FAILED", {"code": "VENDOR_ERROR", "message": repr(exc)})
                raise GatewayError("VENDOR_ERROR", f"厂家 move 请求失败：{exc}", 502) from exc

            if resp.get("status") != "OK":
                message = str(resp.get("error_message") or "Vendor rejected move command")
                self._finish_task(task, "REJECTED", {"code": "VENDOR_REJECTED", "message": message, "raw": resp})
                raise GatewayError("VENDOR_REJECTED", message, 409, {"raw": resp})

            task.vendor_task_id = resp.get("task_id")
            task.state = "ACCEPTED"
            runtime.accepted_monotonic = time.monotonic()
            self.active_task_id = task_id
            self.control_mode = "NAV"
            return copy.deepcopy(task)

    async def set_velocity(self, req: VelocityCommandRequest) -> dict[str, Any]:
        """Apply one direct velocity command.

        Repeated commands from the *same source* are intentionally allowed while the
        gateway is already in DIRECT mode. This is required for a normal planner loop
        (for example 10 Hz cmd_vel updates). A different source is rejected unless it
        explicitly asks to replace the current controller.
        """
        self._assert_command_fresh(req.generated_at_ms, req.max_age_ms)
        self._assert_safe_to_move()

        if abs(req.linear_mps) > self.settings.direct_max_linear_mps + 1e-9:
            raise GatewayError(
                "LINEAR_LIMIT_EXCEEDED",
                f"|linear_mps| must be <= {self.settings.direct_max_linear_mps}",
                400,
            )
        if abs(req.angular_rps) > self.settings.direct_max_angular_rps + 1e-9:
            raise GatewayError(
                "ANGULAR_LIMIT_EXCEEDED",
                f"|angular_rps| must be <= {self.settings.direct_max_angular_rps}",
                400,
            )

        # DIRECT mode is a stream of updates, not a one-shot task. Therefore the
        # current DIRECT owner is allowed to refresh its own command without first
        # stopping itself. This fixes the V2 behaviour where the second cmd_vel frame
        # was treated as CHASSIS_BUSY and the watchdog then stopped the robot.
        same_direct_owner = (
            self.control_mode == "DIRECT"
            and (self.direct_source is None or self.direct_source == req.source)
        )

        if not same_direct_owner:
            busy = self._busy_reason()
            if busy:
                if not req.replace_current:
                    raise GatewayError(
                        "CHASSIS_BUSY",
                        f"底盘当前忙碌（{busy}）",
                        409,
                        {"mode": busy, "direct_source": self.direct_source},
                    )
                await self.stop_current(reason=f"direct control replaced by {req.source}")
                await asyncio.sleep(0.05)

        async with self._command_lock:
            # Re-check after acquiring the lock in case another caller changed mode.
            busy = self._busy_reason()
            allowed_refresh = (
                busy == "DIRECT"
                and (self.direct_source is None or self.direct_source == req.source)
            )
            if busy and not allowed_refresh:
                raise GatewayError(
                    "CHASSIS_BUSY",
                    f"底盘当前忙碌（{busy}）",
                    409,
                    {"mode": busy, "direct_source": self.direct_source},
                )

            self._assert_safe_to_move()
            resp = await self._send_joy(req.linear_mps, req.angular_rps)
            if resp.get("status") != "OK":
                raise GatewayError(
                    "VENDOR_REJECTED",
                    str(resp.get("error_message") or "Vendor rejected joy_control"),
                    409,
                    {"raw": resp},
                )

            self.last_cmd_vel_monotonic = time.monotonic()
            self._watchdog_stop_sent = False

            if abs(req.linear_mps) < 1e-9 and abs(req.angular_rps) < 1e-9:
                self.control_mode = "IDLE"
                self.direct_source = None
                self.last_cmd_vel_monotonic = None
            else:
                self.control_mode = "DIRECT"
                self.direct_source = req.source

            # For an external real-time planner, expose the latest v/w command as a
            # direct-action snapshot. Primitive controllers maintain their own richer
            # action records and therefore are intentionally not overwritten here.
            if not req.source.startswith("primitive-"):
                self.direct_action = {
                    "action_id": req.request_id or req.source,
                    "action": "velocity",
                    "phase": "STREAMING" if self.control_mode == "DIRECT" else "STOPPED",
                    "active": self.control_mode == "DIRECT",
                    "source": req.source,
                    "command_linear_mps": req.linear_mps,
                    "command_angular_rps": req.angular_rps,
                    "progress": None,
                    "updated_at_ms": now_ms(),
                }

            return {
                "ok": True,
                "mode": self.control_mode,
                "source": self.direct_source,
                "watchdog_s": self.settings.direct_watchdog_s,
                "command": {
                    "linear_mps": req.linear_mps,
                    "angular_rps": req.angular_rps,
                },
            }

    async def rotate_direct(self, req: DirectRotateRequest) -> dict[str, Any]:
        """Closed-loop relative rotation over the verified joy_control path.

        Controller gains/defaults come from ``config.json``.  During execution a
        normalized progress record is exposed in ``state.motion.direct_action`` so
        Python/HTTP callers can display live feedback without touching vendor TCP.
        """
        self._assert_safe_to_move()
        try:
            await self._poll_status_once()
        except Exception:
            pass
        pose = self._assert_pose_fresh()
        yaw0 = pose["theta"]
        target = normalize_angle(yaw0 + req.angle_rad)

        max_w = float(req.max_angular_rps or self.settings.rotate_max_angular_rps)
        tolerance = float(req.tolerance_rad or self.settings.rotate_tolerance_rad)
        rate_hz = float(req.rate_hz or self.settings.direct_command_rate_hz)
        kp = float(self.settings.rotate_kp)
        min_w = min(float(self.settings.rotate_min_angular_rps), max_w)
        initial_err = normalize_angle(target - yaw0)

        action_id = uuid.uuid4().hex
        self.direct_action = {
            "action_id": action_id,
            "action": "rotate",
            "phase": "ROTATING",
            "active": True,
            "started_at_ms": now_ms(),
            "updated_at_ms": now_ms(),
            "requested_angle_rad": req.angle_rad,
            "start_yaw_rad": yaw0,
            "target_yaw_rad": target,
            "current_yaw_rad": yaw0,
            "error_rad": initial_err,
            "progress": 0.0,
        }
        cancel_seq = self._direct_cancel_seq

        if abs(initial_err) <= tolerance:
            self.direct_action.update({
                "phase": "SUCCEEDED", "active": False, "progress": 1.0,
                "current_yaw_rad": yaw0, "error_rad": initial_err,
                "finished_at_ms": now_ms(), "updated_at_ms": now_ms(),
            })
            return {
                "ok": True, "start_yaw_rad": yaw0, "target_yaw_rad": target,
                "final_yaw_rad": yaw0, "error_rad": initial_err,
            }

        source = f"primitive-rotate-{uuid.uuid4().hex[:8]}"
        timeout_s = req.timeout_s or max(
            4.0,
            abs(initial_err) / max(max_w, 0.05) * self.settings.rotate_timeout_factor
            + self.settings.rotate_timeout_extra_s,
        )
        deadline = time.monotonic() + float(timeout_s)
        period = 1.0 / rate_hz
        first = True
        final_yaw = yaw0
        try:
            while time.monotonic() < deadline:
                self._assert_safe_to_move()
                if cancel_seq != self._direct_cancel_seq:
                    raise GatewayError("DIRECT_CANCELED", "DIRECT 转向已被取消", 409)
                pose = self._assert_pose_fresh()
                final_yaw = pose["theta"]
                err = normalize_angle(target - final_yaw)
                if abs(err) <= tolerance:
                    break

                w = max(-max_w, min(max_w, kp * err))
                if 0.0 < abs(w) < min_w:
                    w = math.copysign(min_w, w)

                denom = max(abs(initial_err), 1e-9)
                progress = max(0.0, min(0.999, 1.0 - abs(err) / denom))
                self.direct_action.update({
                    "current_yaw_rad": final_yaw,
                    "error_rad": err,
                    "command_angular_rps": w,
                    "progress": progress,
                    "updated_at_ms": now_ms(),
                })

                await self.set_velocity(VelocityCommandRequest(
                    source=source, linear_mps=0.0, angular_rps=w,
                    replace_current=(req.replace_current if first else False),
                    generated_at_ms=now_ms(), max_age_ms=500,
                ))
                first = False
                await asyncio.sleep(period)
            else:
                raise GatewayError(
                    "DIRECT_TIMEOUT",
                    f"Direct rotation timed out after {float(timeout_s):.1f}s",
                    504,
                    {"target_yaw_rad": target, "last_yaw_rad": final_yaw},
                )
        except Exception as exc:
            phase = "CANCELED" if isinstance(exc, GatewayError) and exc.code == "DIRECT_CANCELED" else "FAILED"
            self.direct_action.update({
                "phase": phase, "active": False, "error": str(exc),
                "finished_at_ms": now_ms(), "updated_at_ms": now_ms(),
            })
            raise
        finally:
            try:
                await self.stop_current(reason="direct rotation finished")
            except Exception:
                pass

        await asyncio.sleep(0.12)
        try:
            await self._poll_status_once()
        except Exception:
            pass
        final_yaw = self._assert_pose_fresh()["theta"]
        final_err = normalize_angle(target - final_yaw)
        self.direct_action.update({
            "phase": "SUCCEEDED", "active": False, "progress": 1.0,
            "current_yaw_rad": final_yaw, "error_rad": final_err,
            "command_angular_rps": 0.0,
            "finished_at_ms": now_ms(), "updated_at_ms": now_ms(),
        })
        return {
            "ok": True,
            "start_yaw_rad": yaw0,
            "target_yaw_rad": target,
            "final_yaw_rad": final_yaw,
            "error_rad": final_err,
            "controller": {
                "max_angular_rps": max_w,
                "tolerance_rad": tolerance,
                "kp": kp,
                "min_angular_rps": min_w,
                "rate_hz": rate_hz,
            },
        }

    async def drive_distance_direct(self, req: DirectDistanceRequest) -> dict[str, Any]:
        """Closed-loop signed distance over joy_control using map-pose projection."""
        self._assert_safe_to_move()
        try:
            await self._poll_status_once()
        except Exception:
            pass
        pose = self._assert_pose_fresh()
        x0, y0, yaw0 = pose["x"], pose["y"], pose["theta"]
        distance = float(req.distance_m)

        speed_mps = float(req.speed_mps or self.settings.distance_speed_mps)
        tolerance = float(req.tolerance_m or self.settings.distance_tolerance_m)
        heading_hold = self.settings.distance_heading_hold if req.heading_hold is None else bool(req.heading_hold)
        max_heading_correction = float(
            self.settings.distance_max_heading_correction_rps
            if req.max_heading_correction_rps is None
            else req.max_heading_correction_rps
        )
        rate_hz = float(req.rate_hz or self.settings.direct_command_rate_hz)
        speed_kp = float(self.settings.distance_speed_kp)
        min_speed = min(float(self.settings.distance_min_speed_mps), speed_mps)
        heading_kp = float(self.settings.distance_heading_kp)

        action_id = uuid.uuid4().hex
        self.direct_action = {
            "action_id": action_id,
            "action": "distance",
            "phase": "DRIVING",
            "active": True,
            "started_at_ms": now_ms(),
            "updated_at_ms": now_ms(),
            "requested_distance_m": distance,
            "travelled_m": 0.0,
            "remaining_m": distance,
            "cross_track_m": 0.0,
            "progress": 0.0,
            "start_pose": {"x_m": x0, "y_m": y0, "yaw_rad": yaw0},
        }
        cancel_seq = self._direct_cancel_seq

        if abs(distance) < 1e-9:
            self.direct_action.update({
                "phase": "SUCCEEDED", "active": False, "progress": 1.0,
                "finished_at_ms": now_ms(), "updated_at_ms": now_ms(),
            })
            return {
                "ok": True, "requested_distance_m": 0.0, "travelled_m": 0.0,
                "remaining_m": 0.0, "cross_track_m": 0.0,
                "start_pose": {"x_m": x0, "y_m": y0, "yaw_rad": yaw0},
                "final_pose": {"x_m": x0, "y_m": y0, "yaw_rad": yaw0},
            }

        direction = 1.0 if distance > 0 else -1.0
        timeout_s = req.timeout_s or max(
            5.0,
            abs(distance) / max(speed_mps, 0.03) * self.settings.distance_timeout_factor
            + self.settings.distance_timeout_extra_s,
        )
        deadline = time.monotonic() + float(timeout_s)
        period = 1.0 / rate_hz
        source = f"primitive-distance-{uuid.uuid4().hex[:8]}"
        first = True
        x, y, yaw = x0, y0, yaw0
        along = 0.0
        cross = 0.0
        try:
            while time.monotonic() < deadline:
                self._assert_safe_to_move()
                if cancel_seq != self._direct_cancel_seq:
                    raise GatewayError("DIRECT_CANCELED", "DIRECT 距离控制已被取消", 409)
                pose = self._assert_pose_fresh()
                x, y, yaw = pose["x"], pose["y"], pose["theta"]
                dx, dy = x - x0, y - y0
                along = dx * math.cos(yaw0) + dy * math.sin(yaw0)
                cross = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
                remaining = distance - along
                if abs(remaining) <= tolerance or direction * remaining <= 0.0:
                    break

                v_mag = min(speed_mps, max(min_speed, speed_kp * abs(remaining)))
                v = direction * v_mag
                w = 0.0
                yaw_err = normalize_angle(yaw0 - yaw)
                if heading_hold:
                    w = max(
                        -max_heading_correction,
                        min(max_heading_correction, heading_kp * yaw_err),
                    )

                progress = max(0.0, min(0.999, abs(along) / max(abs(distance), 1e-9)))
                self.direct_action.update({
                    "travelled_m": along,
                    "remaining_m": remaining,
                    "cross_track_m": cross,
                    "current_yaw_rad": yaw,
                    "yaw_error_rad": yaw_err,
                    "command_linear_mps": v,
                    "command_angular_rps": w,
                    "progress": progress,
                    "updated_at_ms": now_ms(),
                })

                await self.set_velocity(VelocityCommandRequest(
                    source=source, linear_mps=v, angular_rps=w,
                    replace_current=(req.replace_current if first else False),
                    generated_at_ms=now_ms(), max_age_ms=500,
                ))
                first = False
                await asyncio.sleep(period)
            else:
                raise GatewayError(
                    "DIRECT_TIMEOUT",
                    f"Direct distance timed out after {float(timeout_s):.1f}s",
                    504,
                    {"requested_distance_m": distance, "travelled_m": along},
                )
        except Exception as exc:
            phase = "CANCELED" if isinstance(exc, GatewayError) and exc.code == "DIRECT_CANCELED" else "FAILED"
            self.direct_action.update({
                "phase": phase, "active": False, "error": str(exc),
                "finished_at_ms": now_ms(), "updated_at_ms": now_ms(),
            })
            raise
        finally:
            try:
                await self.stop_current(reason="direct distance finished")
            except Exception:
                pass

        await asyncio.sleep(0.12)
        try:
            await self._poll_status_once()
        except Exception:
            pass
        pose = self._assert_pose_fresh()
        x, y, yaw = pose["x"], pose["y"], pose["theta"]
        dx, dy = x - x0, y - y0
        along = dx * math.cos(yaw0) + dy * math.sin(yaw0)
        cross = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
        remaining = distance - along
        self.direct_action.update({
            "phase": "SUCCEEDED", "active": False, "progress": 1.0,
            "travelled_m": along, "remaining_m": remaining,
            "cross_track_m": cross, "current_yaw_rad": yaw,
            "command_linear_mps": 0.0, "command_angular_rps": 0.0,
            "finished_at_ms": now_ms(), "updated_at_ms": now_ms(),
        })
        return {
            "ok": True,
            "requested_distance_m": distance,
            "travelled_m": along,
            "remaining_m": remaining,
            "cross_track_m": cross,
            "start_pose": {"x_m": x0, "y_m": y0, "yaw_rad": yaw0},
            "final_pose": {"x_m": x, "y_m": y, "yaw_rad": yaw},
            "tolerance_m": tolerance,
            "controller": {
                "speed_mps": speed_mps,
                "speed_kp": speed_kp,
                "min_speed_mps": min_speed,
                "heading_hold": heading_hold,
                "heading_kp": heading_kp,
                "max_heading_correction_rps": max_heading_correction,
                "rate_hz": rate_hz,
            },
        }

    async def move_relative_direct(self, req: DirectRelativeRequest) -> dict[str, Any]:
        """Turn to the requested relative heading, then drive the signed distance."""
        self._assert_safe_to_move()
        try:
            await self._poll_status_once()
        except Exception:
            pass
        source_pose = self._assert_pose_fresh()

        angular_speed = req.angular_speed_rps or self.settings.rotate_max_angular_rps
        angle_tolerance = req.angle_tolerance_rad or self.settings.rotate_tolerance_rad
        linear_speed = req.linear_speed_mps or self.settings.distance_speed_mps
        distance_tolerance = req.distance_tolerance_m or self.settings.distance_tolerance_m
        rate_hz = req.rate_hz or self.settings.direct_command_rate_hz

        total_timeout = req.timeout_s
        rotate_timeout = None
        distance_timeout = None
        if total_timeout is not None:
            # Split a caller supplied overall budget conservatively according to
            # expected nominal motion time, while keeping each phase >= 1 s.
            tr = abs(req.heading_rad) / max(float(angular_speed), 0.05)
            td = abs(req.distance_m) / max(float(linear_speed), 0.03)
            total_nominal = max(tr + td, 1e-6)
            rotate_timeout = max(1.0, float(total_timeout) * tr / total_nominal) if tr > 0 else 1.0
            distance_timeout = max(1.0, float(total_timeout) - rotate_timeout)

        rotation = await self.rotate_direct(DirectRotateRequest(
            angle_rad=req.heading_rad,
            max_angular_rps=angular_speed,
            tolerance_rad=angle_tolerance,
            timeout_s=rotate_timeout,
            rate_hz=rate_hz,
            replace_current=req.replace_current,
        ))
        translation = await self.drive_distance_direct(DirectDistanceRequest(
            distance_m=req.distance_m,
            speed_mps=linear_speed,
            tolerance_m=distance_tolerance,
            heading_hold=True,
            max_heading_correction_rps=self.settings.distance_max_heading_correction_rps,
            timeout_s=distance_timeout,
            rate_hz=rate_hz,
            replace_current=True,
        ))
        return {
            "ok": True, "mode": "direct",
            "heading_rad": req.heading_rad, "distance_m": req.distance_m,
            "source_pose": {
                "x_m": source_pose["x"], "y_m": source_pose["y"],
                "yaw_rad": source_pose["theta"],
            },
            "rotation": rotation, "translation": translation,
        }

    async def _send_joy(self, linear_mps: float, angular_rps: float) -> dict[str, Any]:
        cmd = f"/api/joy_control?angular_velocity={angular_rps:.6f}&linear_velocity={linear_mps:.6f}"
        return await self.vendor.send_command(cmd)

    async def stop_current(self, reason: Optional[str] = None) -> dict[str, Any]:
        async with self._command_lock:
            self._assert_online()
            if not str(reason or "").startswith("direct "):
                self._direct_cancel_seq += 1
            if self.direct_action and self.direct_action.get("active") and not str(reason or "").startswith("direct "):
                self.direct_action.update({
                    "phase": "STOPPED",
                    "active": False,
                    "stopped_reason": reason,
                    "finished_at_ms": now_ms(),
                    "updated_at_ms": now_ms(),
                })
            result: dict[str, Any]
            if self.control_mode == "NAV" or self.active_task_id:
                resp = await self.vendor.send_command("/api/move/cancel")
                if resp.get("status") != "OK":
                    raise GatewayError("VENDOR_REJECTED", str(resp.get("error_message") or "move/cancel 被底盘拒绝"), 409)
                if self.active_task_id and self.active_task_id in self.tasks:
                    self._finish_task(self.tasks[self.active_task_id].model, "CANCELED", None)
                self.control_mode = "IDLE"
                result = {"ok": True, "stopped": "NAV", "reason": reason}
            else:
                resp = await self._send_joy(0.0, 0.0)
                if resp.get("status") != "OK":
                    raise GatewayError("VENDOR_REJECTED", str(resp.get("error_message") or "零速度 joy_control 被底盘拒绝"), 409)
                if self.control_mode != "ESTOP":
                    self.control_mode = "IDLE"
                self.direct_source = None
                self.last_cmd_vel_monotonic = None
                result = {"ok": True, "stopped": "DIRECT_OR_IDLE", "reason": reason}
            return result

    async def cancel_navigation(self) -> dict[str, Any]:
        async with self._command_lock:
            self._assert_online()
            if self.control_mode != "NAV" and not self.active_task_id:
                return {"ok": True, "canceled": False, "message": "No gateway navigation task is active"}
            resp = await self.vendor.send_command("/api/move/cancel")
            if resp.get("status") != "OK":
                raise GatewayError("VENDOR_REJECTED", str(resp.get("error_message") or "move/cancel 被底盘拒绝"), 409)
            if self.active_task_id and self.active_task_id in self.tasks:
                self._finish_task(self.tasks[self.active_task_id].model, "CANCELED", None)
            self.control_mode = "IDLE"
            return {"ok": True, "canceled": True}

    async def set_estop(self, engaged: bool) -> dict[str, Any]:
        async with self._command_lock:
            self._assert_online()
            resp = await self.vendor.send_command(f"/api/estop?flag={'true' if engaged else 'false'}")
            if resp.get("status") != "OK":
                raise GatewayError("VENDOR_REJECTED", str(resp.get("error_message") or "急停命令被底盘拒绝"), 409)

            if engaged:
                self._direct_cancel_seq += 1
                if self.active_task_id and self.active_task_id in self.tasks:
                    self._finish_task(
                        self.tasks[self.active_task_id].model,
                        "CANCELED",
                        {"code": "ESTOP", "message": "Task interrupted by software emergency stop"},
                    )
                self.control_mode = "ESTOP"
                self.direct_source = None
                self.last_cmd_vel_monotonic = None
            else:
                # This only releases the software estop. A physical estop can remain active.
                self.control_mode = "IDLE"
                try:
                    await self._poll_status_once()
                except Exception:
                    pass
                if self.vendor_status.get("estop_state") is True:
                    self.control_mode = "ESTOP"

            return {
                "ok": True,
                "soft_estop_requested": engaged,
                "estop_state": self.vendor_status.get("estop_state"),
                "hard_estop_state": self.vendor_status.get("hard_estop_state"),
            }

    @staticmethod
    def _require_vendor_ok(resp: dict[str, Any], operation: str) -> dict[str, Any]:
        if resp.get("status") != "OK":
            raise GatewayError(
                "VENDOR_REJECTED",
                str(resp.get("error_message") or f"Vendor rejected {operation}"),
                409,
                {"raw": resp},
            )
        return resp

    async def refresh_state(self) -> dict[str, Any]:
        """Force an immediate status + power poll, then return the normalized state."""
        self._assert_online()
        await self._poll_status_once()
        try:
            await self._poll_power_once()
        except Exception:
            # Power information is useful but should not make pose/status refresh fail.
            pass
        return self.state_snapshot()

    async def robot_info(self) -> dict[str, Any]:
        self._assert_online()
        resp = self._require_vendor_ok(await self.vendor.request_robot_info(), "robot_info")
        return {"ok": True, "results": resp.get("results") or {}}

    async def get_nav_params(self) -> dict[str, Any]:
        self._assert_online()
        resp = self._require_vendor_ok(await self.vendor.get_params(), "get_params")
        return {"ok": True, "results": resp.get("results") or {}}

    async def set_nav_params(self, req: NavParamsRequest) -> dict[str, Any]:
        self._assert_online()
        if req.max_speed_linear is None and req.max_speed_angular is None:
            raise GatewayError("INVALID_ARGUMENT", "至少需要提供一个导航参数", 400)
        resp = self._require_vendor_ok(
            await self.vendor.set_params(
                max_speed_linear=req.max_speed_linear,
                max_speed_angular=req.max_speed_angular,
            ),
            "set_params",
        )
        # Vendor manual says set_params may return status=OK even if a value did not
        # take effect, therefore read back the current values before returning.
        verify = self._require_vendor_ok(await self.vendor.get_params(), "get_params")
        return {
            "ok": True,
            "requested": req.model_dump(exclude_none=True),
            "results": verify.get("results") or {},
            "raw_set_response": resp,
        }

    async def diagnosis(self) -> dict[str, Any]:
        self._assert_online()
        resp = self._require_vendor_ok(await self.vendor.diagnosis(), "diagnosis")
        return {"ok": True, "results": resp.get("results") or {}}

    async def planned_path(self) -> dict[str, Any]:
        self._assert_online()
        resp = self._require_vendor_ok(await self.vendor.planned_path(), "get_planned_path")
        return {"ok": True, "results": resp.get("results") or {}}

    async def accessible_point(self, x_m: float, y_m: float) -> dict[str, Any]:
        self._assert_online()
        resp = self._require_vendor_ok(await self.vendor.accessible_point(x_m, y_m), "accessible_point_query")
        return {"ok": True, "results": resp.get("results") or {}}

    async def distance_probe(self, x_m: float, y_m: float) -> dict[str, Any]:
        self._assert_online()
        resp = self._require_vendor_ok(await self.vendor.distance_probe(x_m, y_m), "distance_probe")
        return {"ok": True, "results": resp.get("results") or {}}

    async def make_plan_distance(self, req: PlanDistanceRequest) -> dict[str, Any]:
        self._assert_online()
        resp = self._require_vendor_ok(
            await self.vendor.make_plan_distance(
                req.start_x, req.start_y, req.start_floor,
                req.goal_x, req.goal_y, req.goal_floor,
            ),
            "make_plan",
        )
        return {"ok": True, "results": resp.get("results") or {}}

    async def map_list(self) -> dict[str, Any]:
        self._assert_online()
        resp = self._require_vendor_ok(await self.vendor.map_list(), "map/list")
        return {"ok": True, "results": resp.get("results") or {}}

    async def current_map(self) -> dict[str, Any]:
        self._assert_online()
        resp = self._require_vendor_ok(await self.vendor.current_map(), "map/get_current_map")
        return {"ok": True, "results": resp.get("results") or {}}

    def get_task(self, task_id: str) -> TaskRecordModel:
        runtime = self.tasks.get(task_id)
        if not runtime:
            raise GatewayError("TASK_NOT_FOUND", f"未知 task_id：{task_id}", 404)
        return copy.deepcopy(runtime.model)

    def _euclidean_distance_to_target(self, target: dict[str, Any]) -> Optional[float]:
        pose = self.vendor_status.get("current_pose")
        if not isinstance(pose, dict):
            return None
        try:
            dx = float(target["x_m"]) - float(pose["x"])
            dy = float(target["y_m"]) - float(pose["y"])
            return math.hypot(dx, dy)
        except Exception:
            return None

    def state_snapshot(self) -> dict[str, Any]:
        mono = time.monotonic()
        pose = self.vendor_status.get("current_pose") or {}
        active_task = None
        distance_to_goal = None
        yaw_error = None
        if self.active_task_id and self.active_task_id in self.tasks:
            active_task = self.tasks[self.active_task_id].model.model_dump()
            distance_to_goal = self._euclidean_distance_to_target(self.tasks[self.active_task_id].model.target)
            try:
                yaw_error = normalize_angle(
                    float(self.tasks[self.active_task_id].model.target["yaw_rad"]) - float(pose.get("theta"))
                )
            except Exception:
                yaw_error = None

        status_age_ms = None if self.last_status_monotonic is None else int((mono - self.last_status_monotonic) * 1000)
        velocity_age_ms = None if self.last_velocity_monotonic is None else int((mono - self.last_velocity_monotonic) * 1000)
        power_age_ms = None if self.last_power_monotonic is None else int((mono - self.last_power_monotonic) * 1000)

        error_code = str(self.vendor_status.get("error_code") or "00000000")
        return {
            "timestamp_ms": now_ms(),
            "connection": {
                "online": self.vendor.connected,
                "robot_host": self.settings.robot_host,
                "robot_port": self.settings.robot_port,
                "generation": self.vendor.connection_generation,
                "last_error": self.vendor.last_error,
                "status_age_ms": status_age_ms,
                "velocity_age_ms": velocity_age_ms,
                "power_age_ms": power_age_ms,
            },
            "pose": {
                "frame": "map",
                "x_m": pose.get("x"),
                "y_m": pose.get("y"),
                "yaw_rad": pose.get("theta"),
                "floor": self.vendor_status.get("current_floor"),
            },
            "velocity": {
                "linear_mps": self.velocity.get("linear", 0.0),
                "angular_rps": self.velocity.get("angular", 0.0),
            },
            "motion": {
                "gateway_mode": self.control_mode,
                "vendor_move_status": self.vendor_status.get("move_status"),
                "vendor_running_status": self.vendor_status.get("running_status"),
                "vendor_move_target": self.vendor_status.get("move_target"),
                "vendor_move_retry_times": self.vendor_status.get("move_retry_times"),
                "active_task": active_task,
                "euclidean_distance_to_goal_m": distance_to_goal,
                "yaw_error_rad": yaw_error,
                "direct_source": self.direct_source,
                "direct_action": copy.deepcopy(self.direct_action),
            },
            "safety": {
                "estop": self.vendor_status.get("estop_state"),
                "soft_estop": self.vendor_status.get("soft_estop_state"),
                "hard_estop": self.vendor_status.get("hard_estop_state"),
                "error_code": error_code,
                "fault": error_code not in ("", "0", "00000000"),
            },
            "power": {
                "percent": self.power.get("battery_capacity", self.vendor_status.get("power_percent")),
                "battery_current_a": self.power.get("battery_current"),
                "battery_voltage_v": self.power.get("battery_voltage"),
                "charge_voltage_v": self.power.get("charge_voltage"),
                "charging": self.power.get("charger_connected_notice", self.vendor_status.get("charge_state")),
                "chargepile_id": self.vendor_status.get("chargepile_id"),
                "head_current_a": self.power.get("head_current"),
            },
            "ready_to_move": (
                self.vendor.connected
                and self.vendor_status.get("estop_state") is not True
                and error_code in ("", "0", "00000000")
            ),
            "last_notification": copy.deepcopy(self.last_notification),
        }

    def health_snapshot(self) -> dict[str, Any]:
        state = self.state_snapshot()
        return {
            "service": "ok",
            "chassis_connected": state["connection"]["online"],
            "control_available": (
                state["connection"]["online"]
                and state["safety"]["estop"] is not True
                and state["safety"]["fault"] is False
            ),
            "gateway_mode": self.control_mode,
            "status_age_ms": state["connection"]["status_age_ms"],
            "version": "1.3.0",
        }

    def capabilities(self) -> dict[str, Any]:
        return {
            "api_version": "v1",
            "robot_family": "Yunji WATER",
            "control_modes": ["direct_relative", "direct_distance", "direct_rotate", "cmd_vel", "relative_goal_vendor_nav", "map_goal_vendor_nav"],
            "coordinate_frames": {
                "base_link": {
                    "x": "forward",
                    "y": "left",
                    "yaw_positive": "counter-clockwise / left",
                },
                "map": "vendor map frame",
            },
            "units": {
                "distance": "m",
                "linear_velocity": "m/s",
                "angle": "rad",
                "angular_velocity": "rad/s",
            },
            "limits": {
                "vendor_joy_control": {
                    "linear_mps": [-0.5, 0.5],
                    "angular_rps": [-1.0, 1.0],
                },
                "gateway_direct": {
                    "linear_mps": [-self.settings.direct_max_linear_mps, self.settings.direct_max_linear_mps],
                    "angular_rps": [-self.settings.direct_max_angular_rps, self.settings.direct_max_angular_rps],
                    "watchdog_s": self.settings.direct_watchdog_s,
                },
            },
            "controller_defaults": {
                "rate_hz": self.settings.direct_command_rate_hz,
                "rotation": {
                    "max_angular_rps": self.settings.rotate_max_angular_rps,
                    "tolerance_rad": self.settings.rotate_tolerance_rad,
                    "kp": self.settings.rotate_kp,
                    "min_angular_rps": self.settings.rotate_min_angular_rps,
                },
                "distance": {
                    "speed_mps": self.settings.distance_speed_mps,
                    "tolerance_m": self.settings.distance_tolerance_m,
                    "heading_hold": self.settings.distance_heading_hold,
                    "speed_kp": self.settings.distance_speed_kp,
                    "min_speed_mps": self.settings.distance_min_speed_mps,
                    "heading_kp": self.settings.distance_heading_kp,
                    "max_heading_correction_rps": self.settings.distance_max_heading_correction_rps,
                },
            },
            "state_rates_hz": {
                "robot_status": self.settings.status_frequency_hz,
                "robot_velocity": self.settings.velocity_frequency_hz,
                "websocket_stream": self.settings.stream_frequency_hz,
            },
            "status_fields": ["pose", "velocity", "move_status", "running_status", "battery", "charging", "estop", "error_code"],
            "vendor_helpers": [
                "robot_info", "get/set navigation params", "diagnosis",
                "planned_path", "accessible_point", "distance_probe", "make_plan_distance",
                "map_list", "current_map",
            ],
        }
