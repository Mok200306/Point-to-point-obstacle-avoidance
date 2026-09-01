from __future__ import annotations

import asyncio
import inspect
import json
import logging
import time
import uuid
from typing import Any, Awaitable, Callable, Optional

from .config import Settings

logger = logging.getLogger(__name__)

PacketHandler = Callable[[dict[str, Any]], Awaitable[None] | None]
ConnectedHandler = Callable[[], Awaitable[None] | None]


class VendorConnectionError(RuntimeError):
    pass


class VendorRequestError(RuntimeError):
    pass


class YunjiClient:
    """Async TCP client for Yunji WATER's newline-delimited command/JSON protocol.

    The vendor manual specifies URL-like command strings over TCP and JSON replies.
    The user's existing bench script has already validated line-oriented JSON reading on
    this chassis, so this adapter uses a trailing '\n' for commands and reader.readline().
    """

    def __init__(self, settings: Settings):
        self.settings = settings
        self._reader: Optional[asyncio.StreamReader] = None
        self._writer: Optional[asyncio.StreamWriter] = None
        self._manager_task: Optional[asyncio.Task] = None
        self._reader_task: Optional[asyncio.Task] = None
        self._stop_event = asyncio.Event()
        self._connected_event = asyncio.Event()
        self._write_lock = asyncio.Lock()
        self._pending: dict[str, asyncio.Future] = {}
        self._packet_handler: Optional[PacketHandler] = None
        self._connected_handler: Optional[ConnectedHandler] = None

        self.last_rx_monotonic: Optional[float] = None
        self.last_error: Optional[str] = None
        self.connection_generation: int = 0

    @property
    def connected(self) -> bool:
        return self._connected_event.is_set() and self._writer is not None

    def set_packet_handler(self, handler: PacketHandler) -> None:
        self._packet_handler = handler

    def set_connected_handler(self, handler: ConnectedHandler) -> None:
        self._connected_handler = handler

    async def start(self) -> None:
        if self._manager_task and not self._manager_task.done():
            return
        self._stop_event.clear()
        self._manager_task = asyncio.create_task(self._connection_manager(), name="yunji-connection-manager")

    async def stop(self) -> None:
        self._stop_event.set()
        if self._manager_task:
            self._manager_task.cancel()
        if self._reader_task:
            self._reader_task.cancel()
        await self._close_transport()
        for fut in list(self._pending.values()):
            if not fut.done():
                fut.set_exception(VendorConnectionError("client stopped"))
        self._pending.clear()

    async def _connection_manager(self) -> None:
        while not self._stop_event.is_set():
            try:
                logger.info("Connecting to WATER chassis %s:%s", self.settings.robot_host, self.settings.robot_port)
                reader, writer = await asyncio.wait_for(
                    asyncio.open_connection(self.settings.robot_host, self.settings.robot_port),
                    timeout=self.settings.robot_connect_timeout_s,
                )
                self._reader = reader
                self._writer = writer
                self._connected_event.set()
                self.last_error = None
                self.connection_generation += 1
                logger.info("Connected to WATER chassis (generation=%d)", self.connection_generation)

                self._reader_task = asyncio.create_task(self._reader_loop(), name="yunji-reader")
                if self._connected_handler:
                    result = self._connected_handler()
                    if inspect.isawaitable(result):
                        asyncio.create_task(result)

                await self._reader_task
            except asyncio.CancelledError:
                break
            except Exception as exc:
                self.last_error = repr(exc)
                logger.warning("WATER connection lost/failed: %r", exc)
            finally:
                self._connected_event.clear()
                await self._close_transport()
                self._fail_all_pending(VendorConnectionError("WATER connection lost"))

            if not self._stop_event.is_set():
                await asyncio.sleep(self.settings.robot_reconnect_delay_s)

    async def _reader_loop(self) -> None:
        assert self._reader is not None
        while not self._stop_event.is_set():
            raw = await self._reader.readline()
            if not raw:
                raise VendorConnectionError("peer closed TCP connection")

            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            self.last_rx_monotonic = time.monotonic()
            try:
                packet = json.loads(line)
            except json.JSONDecodeError:
                logger.warning("Ignoring non-JSON line from WATER: %r", line[:300])
                continue

            if not isinstance(packet, dict):
                logger.warning("Ignoring non-object JSON packet: %r", packet)
                continue

            if packet.get("type") == "response":
                request_uuid = str(packet.get("uuid") or "")
                fut = self._pending.pop(request_uuid, None) if request_uuid else None
                if fut is not None and not fut.done():
                    fut.set_result(packet)

            if self._packet_handler:
                try:
                    result = self._packet_handler(packet)
                    if inspect.isawaitable(result):
                        await result
                except Exception:
                    logger.exception("Packet handler failed")

    async def _close_transport(self) -> None:
        writer = self._writer
        self._reader = None
        self._writer = None
        if writer is not None:
            try:
                writer.close()
                await writer.wait_closed()
            except Exception:
                pass

    def _fail_all_pending(self, exc: Exception) -> None:
        for fut in list(self._pending.values()):
            if not fut.done():
                fut.set_exception(exc)
        self._pending.clear()

    async def wait_connected(self, timeout_s: Optional[float] = None) -> None:
        timeout_s = self.settings.robot_connect_timeout_s if timeout_s is None else timeout_s
        try:
            await asyncio.wait_for(self._connected_event.wait(), timeout=timeout_s)
        except asyncio.TimeoutError as exc:
            raise VendorConnectionError("WATER chassis is offline") from exc

    @staticmethod
    def _attach_uuid(command: str, request_uuid: str) -> str:
        sep = "&" if "?" in command else "?"
        return f"{command}{sep}uuid={request_uuid}"

    async def send_command(self, command: str, timeout_s: Optional[float] = None) -> dict[str, Any]:
        await self.wait_connected()
        timeout_s = self.settings.robot_request_timeout_s if timeout_s is None else timeout_s

        request_uuid = uuid.uuid4().hex
        loop = asyncio.get_running_loop()
        fut = loop.create_future()
        self._pending[request_uuid] = fut
        wire_command = self._attach_uuid(command, request_uuid) + "\n"

        try:
            async with self._write_lock:
                writer = self._writer
                if writer is None:
                    raise VendorConnectionError("WATER chassis is offline")
                writer.write(wire_command.encode("utf-8"))
                await writer.drain()
            response = await asyncio.wait_for(fut, timeout=timeout_s)
        except Exception:
            self._pending.pop(request_uuid, None)
            raise

        if not isinstance(response, dict):
            raise VendorRequestError("invalid response type")
        return response

    async def request_status(self) -> dict[str, Any]:
        return await self.send_command("/api/robot_status")

    async def subscribe_status(self, frequency_hz: float) -> dict[str, Any]:
        return await self.send_command(f"/api/request_data?topic=robot_status&frequency={frequency_hz:g}")

    async def subscribe_velocity(self, frequency_hz: float) -> dict[str, Any]:
        return await self.send_command(f"/api/request_data?topic=robot_velocity&frequency={frequency_hz:g}")

    async def request_power(self) -> dict[str, Any]:
        return await self.send_command("/api/get_power_status")

    async def request_robot_info(self) -> dict[str, Any]:
        return await self.send_command("/api/robot_info")

    async def get_params(self) -> dict[str, Any]:
        return await self.send_command("/api/get_params")

    async def set_params(
        self,
        *,
        max_speed_linear: Optional[float] = None,
        max_speed_angular: Optional[float] = None,
    ) -> dict[str, Any]:
        parts: list[str] = []
        if max_speed_linear is not None:
            parts.append(f"max_speed_linear={float(max_speed_linear):.6f}")
        if max_speed_angular is not None:
            parts.append(f"max_speed_angular={float(max_speed_angular):.6f}")
        if not parts:
            raise ValueError("At least one navigation parameter is required")
        return await self.send_command("/api/set_params?" + "&".join(parts))

    async def diagnosis(self) -> dict[str, Any]:
        return await self.send_command("/api/diagnosis/get_result")

    async def planned_path(self) -> dict[str, Any]:
        return await self.send_command("/api/get_planned_path")

    async def accessible_point(self, x: float, y: float) -> dict[str, Any]:
        return await self.send_command(f"/api/map/accessible_point_query?x={x:.6f}&y={y:.6f}")

    async def distance_probe(self, x: float, y: float) -> dict[str, Any]:
        return await self.send_command(f"/api/map/distance_probe?x={x:.6f}&y={y:.6f}")

    async def make_plan_distance(
        self,
        start_x: float,
        start_y: float,
        start_floor: int,
        goal_x: float,
        goal_y: float,
        goal_floor: int,
    ) -> dict[str, Any]:
        cmd = (
            "/api/make_plan?"
            f"start_x={start_x:.6f}&start_y={start_y:.6f}&start_floor={int(start_floor)}"
            f"&goal_x={goal_x:.6f}&goal_y={goal_y:.6f}&goal_floor={int(goal_floor)}"
        )
        return await self.send_command(cmd)

    async def map_list(self) -> dict[str, Any]:
        return await self.send_command("/api/map/list")

    async def current_map(self) -> dict[str, Any]:
        return await self.send_command("/api/map/get_current_map")
