"""Small WATER-compatible mock server for development without the physical chassis.

It implements only the subset used by this gateway:
/api/request_data, /api/robot_status, /api/get_power_status, /api/move,
/api/move/cancel, /api/joy_control and /api/estop.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import time
from urllib.parse import parse_qs, urlsplit


class MockRobot:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear = 0.0
        self.angular = 0.0
        self.move_status = "idle"
        self.running_status = "idle"
        self.estop = False
        self.soft_estop = False
        self.hard_estop = False
        self.goal = None
        self.goal_task = None
        self.last_joy = 0.0
        self.clients = set()
        self.subscriptions = {}
        self.lock = asyncio.Lock()
        self.nav_max_linear = 0.5
        self.nav_max_angular = 1.0

    def status_results(self):
        return {
            "move_target": "",
            "move_status": self.move_status,
            "running_status": self.running_status,
            "move_retry_times": 0,
            "charge_state": False,
            "soft_estop_state": self.soft_estop,
            "hard_estop_state": self.hard_estop,
            "estop_state": self.estop,
            "power_percent": 83,
            "current_pose": {"x": self.x, "y": self.y, "theta": self.theta},
            "current_floor": 1,
            "chargepile_id": "0",
            "error_code": "00000000",
        }

    async def physics_loop(self):
        last = time.monotonic()
        while True:
            await asyncio.sleep(0.02)
            now = time.monotonic()
            dt = now - last
            last = now
            async with self.lock:
                if self.estop:
                    self.linear = 0.0
                    self.angular = 0.0
                    continue

                # Vendor joy_control expires after 0.5 s.
                if now - self.last_joy > 0.5 and self.goal is None:
                    self.linear = 0.0
                    self.angular = 0.0

                if self.goal is not None:
                    gx, gy, gyaw, tol = self.goal
                    dx = gx - self.x
                    dy = gy - self.y
                    dist = math.hypot(dx, dy)
                    if dist <= tol:
                        yaw_err = math.atan2(math.sin(gyaw - self.theta), math.cos(gyaw - self.theta))
                        if abs(yaw_err) < 0.08:
                            self.x, self.y, self.theta = gx, gy, gyaw
                            self.linear = 0.0
                            self.angular = 0.0
                            self.goal = None
                            self.move_status = "succeeded"
                            self.running_status = "idle"
                        else:
                            self.linear = 0.0
                            self.angular = max(-0.6, min(0.6, 2.0 * yaw_err))
                    else:
                        desired = math.atan2(dy, dx)
                        yaw_err = math.atan2(math.sin(desired - self.theta), math.cos(desired - self.theta))
                        if abs(yaw_err) > 0.20:
                            self.linear = 0.0
                            self.angular = max(-0.6, min(0.6, 1.8 * yaw_err))
                        else:
                            self.angular = max(-0.4, min(0.4, 1.2 * yaw_err))
                            self.linear = min(0.25, max(0.05, dist))

                self.theta = math.atan2(math.sin(self.theta + self.angular * dt), math.cos(self.theta + self.angular * dt))
                self.x += self.linear * math.cos(self.theta) * dt
                self.y += self.linear * math.sin(self.theta) * dt

    async def callback_loop(self):
        last_sent = {}
        while True:
            await asyncio.sleep(0.01)
            now = time.monotonic()
            for writer in list(self.clients):
                subs = self.subscriptions.get(writer, {})
                for topic, hz in list(subs.items()):
                    key = (id(writer), topic)
                    if now - last_sent.get(key, 0) < 1.0 / hz:
                        continue
                    last_sent[key] = now
                    if topic == "robot_status":
                        pkt = {"type": "callback", "topic": "robot_status", "results": self.status_results()}
                    elif topic == "robot_velocity":
                        pkt = {"type": "callback", "topic": "robot_velocity", "results": {"angular": self.angular, "linear": self.linear}}
                    else:
                        continue
                    try:
                        writer.write((json.dumps(pkt) + "\n").encode())
                        await writer.drain()
                    except Exception:
                        pass

    async def handle(self, reader, writer):
        self.clients.add(writer)
        self.subscriptions[writer] = {}
        peer = writer.get_extra_info("peername")
        print("client connected:", peer)
        try:
            while True:
                raw = await reader.readline()
                if not raw:
                    break
                line = raw.decode().strip()
                if not line:
                    continue
                parts = urlsplit(line)
                path = parts.path
                q = {k: v[-1] for k, v in parse_qs(parts.query).items()}
                uid = q.get("uuid", "")
                response = {"type": "response", "command": path, "uuid": uid, "status": "OK", "error_message": ""}

                async with self.lock:
                    if path == "/api/request_data":
                        topic = q.get("topic")
                        hz = float(q.get("frequency", "1"))
                        self.subscriptions[writer][topic] = max(0.1, hz)
                    elif path == "/api/robot_status":
                        response["results"] = self.status_results()
                    elif path == "/api/get_power_status":
                        response["results"] = {
                            "battery_capacity": 83,
                            "battery_current": -1.2,
                            "battery_voltage": 25.8,
                            "charge_voltage": 0.0,
                            "charger_connected_notice": False,
                            "head_current": 0.4,
                        }
                    elif path == "/api/move":
                        if self.estop:
                            response["status"] = "BUSY"
                            response["error_message"] = "estop active"
                        elif self.move_status == "running":
                            response["status"] = "BUSY"
                            response["error_message"] = "move task already running"
                        else:
                            loc = q.get("location", "")
                            try:
                                gx, gy, gyaw = map(float, loc.split(","))
                                tol = float(q.get("distance_tolerance", "0.15"))
                                self.goal = (gx, gy, gyaw, tol)
                                self.move_status = "running"
                                self.running_status = "running"
                                self.goal_task = f"MOCK{int(time.time()*1000)}"
                                response["task_id"] = self.goal_task
                            except Exception:
                                response["status"] = "INVALID_ARGUMENT"
                                response["error_message"] = "bad location"
                    elif path == "/api/move/cancel":
                        self.goal = None
                        self.linear = 0.0
                        self.angular = 0.0
                        self.move_status = "canceled"
                        self.running_status = "idle"
                    elif path == "/api/joy_control":
                        if self.estop:
                            response["status"] = "BUSY"
                            response["error_message"] = "estop active"
                        else:
                            self.goal = None
                            self.move_status = "idle"
                            self.running_status = "idle"
                            self.angular = max(-1.0, min(1.0, float(q.get("angular_velocity", "0"))))
                            self.linear = max(-0.5, min(0.5, float(q.get("linear_velocity", "0"))))
                            self.last_joy = time.monotonic()
                    elif path == "/api/estop":
                        self.soft_estop = q.get("flag", "false").lower() == "true"
                        self.estop = self.soft_estop or self.hard_estop
                        if self.estop:
                            self.goal = None
                            self.linear = 0.0
                            self.angular = 0.0
                    elif path == "/api/robot_info":
                        response["results"] = {"robot_model": "MOCK-WATER", "software_version": "mock-1.0"}
                    elif path == "/api/get_params":
                        response["results"] = {
                            "max_speed_linear": self.nav_max_linear,
                            "max_speed_angular": self.nav_max_angular,
                        }
                    elif path == "/api/set_params":
                        if "max_speed_linear" in q:
                            self.nav_max_linear = float(q["max_speed_linear"])
                        if "max_speed_angular" in q:
                            self.nav_max_angular = float(q["max_speed_angular"])
                    elif path == "/api/diagnosis/get_result":
                        response["results"] = {
                            "motor_core_right": {"status": True},
                            "motor_core_left": {"status": True},
                            "laser": {"status": True},
                            "IMU": {"status": True},
                        }
                    elif path == "/api/get_planned_path":
                        response["results"] = {"path": [] if self.goal is None else [{"x": self.x, "y": self.y}, {"x": self.goal[0], "y": self.goal[1]}]}
                    elif path == "/api/map/accessible_point_query":
                        response["results"] = {"position": {"x": float(q.get("x", 0)), "y": float(q.get("y", 0))}}
                    elif path == "/api/map/distance_probe":
                        response["results"] = {"obstacle": 2.0, "static": 1.5}
                    elif path == "/api/make_plan":
                        sx, sy = float(q.get("start_x", 0)), float(q.get("start_y", 0))
                        gx, gy = float(q.get("goal_x", 0)), float(q.get("goal_y", 0))
                        response["results"] = {"distance": math.hypot(gx - sx, gy - sy)}
                    elif path == "/api/map/list":
                        response["results"] = {"maps": ["mock_map"]}
                    elif path == "/api/map/get_current_map":
                        response["results"] = {"hotel_id": "mock_map", "floor": 1}
                    else:
                        response["status"] = "UNKNOWN_ERROR"
                        response["error_message"] = f"mock endpoint not implemented: {path}"

                writer.write((json.dumps(response) + "\n").encode())
                await writer.drain()
        finally:
            self.clients.discard(writer)
            self.subscriptions.pop(writer, None)
            writer.close()
            await writer.wait_closed()
            print("client disconnected:", peer)


async def amain(host: str, port: int):
    robot = MockRobot()
    server = await asyncio.start_server(robot.handle, host, port)
    asyncio.create_task(robot.physics_loop())
    asyncio.create_task(robot.callback_loop())
    print(f"Mock WATER server listening on {host}:{port}")
    async with server:
        await server.serve_forever()


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=31001)
    a = p.parse_args()
    asyncio.run(amain(a.host, a.port))
