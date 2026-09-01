from __future__ import annotations

import asyncio
import logging
from contextlib import asynccontextmanager

from fastapi import FastAPI, Query, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse

from .config import settings
from .gateway import GatewayError, RobotGateway
from .models import (
    DirectDistanceRequest,
    DirectRelativeRequest,
    DirectRotateRequest,
    EstopRequest,
    MapGoalRequest,
    NavParamsRequest,
    PlanDistanceRequest,
    RelativeMotionRequest,
    StopRequest,
    VelocityCommandRequest,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s: %(message)s",
)

gateway = RobotGateway(settings)


@asynccontextmanager
async def lifespan(app: FastAPI):
    await gateway.start()
    try:
        yield
    finally:
        await gateway.stop()


app = FastAPI(
    title="WATER Chassis Gateway API",
    version="1.3.0",
    description=(
        "Planner-friendly HTTP/WebSocket facade for Yunji WATER. "
        "Normal users should prefer water_chassis.py. V5 adds config-driven control and live progress feedback. HTTP endpoints remain available "
        "for C++/remote callers and diagnostics."
    ),
    lifespan=lifespan,
)


@app.exception_handler(GatewayError)
async def gateway_error_handler(_: Request, exc: GatewayError):
    return JSONResponse(
        status_code=exc.http_status,
        content={
            "ok": False,
            "error": {
                "code": exc.code,
                "message": exc.message,
                "details": exc.details or None,
            },
        },
    )


# ---------------------------------------------------------------------------
# State / capability
# ---------------------------------------------------------------------------
@app.get("/api/v1/health")
async def health():
    return gateway.health_snapshot()


@app.get("/api/v1/capabilities")
async def capabilities():
    return gateway.capabilities()


@app.get("/api/v1/chassis/state")
async def chassis_state():
    return gateway.state_snapshot()


@app.post("/api/v1/chassis/refresh")
async def chassis_refresh():
    return await gateway.refresh_state()


@app.get("/api/v1/chassis/info")
async def chassis_info():
    return await gateway.robot_info()


@app.get("/api/v1/chassis/diagnosis")
async def chassis_diagnosis():
    return await gateway.diagnosis()


# ---------------------------------------------------------------------------
# Motion
# ---------------------------------------------------------------------------
@app.post("/api/v1/motion/relative", status_code=202)
async def motion_relative(req: RelativeMotionRequest):
    """Vendor autonomous navigation to a relative target converted into map x/y/yaw."""
    task = await gateway.move_relative(req)
    return {"ok": True, "task": task.model_dump()}


@app.post("/api/v1/motion/goal", status_code=202)
async def motion_goal(req: MapGoalRequest):
    task = await gateway.move_map(req)
    return {"ok": True, "task": task.model_dump()}


@app.put("/api/v1/motion/velocity")
async def motion_velocity(req: VelocityCommandRequest):
    return await gateway.set_velocity(req)


@app.post("/api/v1/motion/direct/rotate")
async def motion_direct_rotate(req: DirectRotateRequest):
    return await gateway.rotate_direct(req)


@app.post("/api/v1/motion/direct/distance")
async def motion_direct_distance(req: DirectDistanceRequest):
    return await gateway.drive_distance_direct(req)


@app.post("/api/v1/motion/direct/relative")
async def motion_direct_relative(req: DirectRelativeRequest):
    return await gateway.move_relative_direct(req)


@app.post("/api/v1/motion/stop")
async def motion_stop(req: StopRequest = StopRequest()):
    return await gateway.stop_current(reason=req.reason)


@app.post("/api/v1/motion/cancel")
async def motion_cancel():
    return await gateway.cancel_navigation()


@app.get("/api/v1/tasks/{task_id}")
async def get_task(task_id: str):
    return {"ok": True, "task": gateway.get_task(task_id).model_dump()}


# ---------------------------------------------------------------------------
# Safety
# ---------------------------------------------------------------------------
@app.post("/api/v1/safety/estop")
async def safety_estop(req: EstopRequest):
    return await gateway.set_estop(req.engaged)


# ---------------------------------------------------------------------------
# Vendor NAV configuration / planning helpers
# ---------------------------------------------------------------------------
@app.get("/api/v1/navigation/params")
async def navigation_params():
    return await gateway.get_nav_params()


@app.put("/api/v1/navigation/params")
async def navigation_set_params(req: NavParamsRequest):
    return await gateway.set_nav_params(req)


@app.get("/api/v1/navigation/path")
async def navigation_path():
    return await gateway.planned_path()


@app.post("/api/v1/navigation/plan-distance")
async def navigation_plan_distance(req: PlanDistanceRequest):
    return await gateway.make_plan_distance(req)


@app.get("/api/v1/map/accessible-point")
async def map_accessible_point(
    x_m: float = Query(...),
    y_m: float = Query(...),
):
    return await gateway.accessible_point(x_m, y_m)


@app.get("/api/v1/map/distance-probe")
async def map_distance_probe(
    x_m: float = Query(...),
    y_m: float = Query(...),
):
    return await gateway.distance_probe(x_m, y_m)


@app.get("/api/v1/map/list")
async def map_list():
    return await gateway.map_list()


@app.get("/api/v1/map/current")
async def map_current():
    return await gateway.current_map()


@app.websocket("/api/v1/stream")
async def state_stream(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            await websocket.send_json(gateway.state_snapshot())
            await asyncio.sleep(1.0 / max(settings.stream_frequency_hz, 1.0))
    except WebSocketDisconnect:
        pass
    except Exception:
        try:
            await websocket.close()
        except Exception:
            pass
