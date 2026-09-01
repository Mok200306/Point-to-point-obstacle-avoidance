from __future__ import annotations

from typing import Literal, Optional

from pydantic import BaseModel, Field


HeadingFrame = Literal["base_link", "map"]
TaskState = Literal[
    "SUBMITTING",
    "ACCEPTED",
    "EXECUTING",
    "SUCCEEDED",
    "FAILED",
    "CANCELED",
    "REJECTED",
]


class RelativeMotionRequest(BaseModel):
    request_id: Optional[str] = None
    heading_rad: float = Field(..., ge=-3.141592653589793, le=3.141592653589793)
    heading_frame: HeadingFrame = "base_link"
    distance_m: float = Field(..., gt=0.0, le=100.0)
    final_yaw_rad: Optional[float] = Field(default=None, ge=-3.141592653589793, le=3.141592653589793)
    final_yaw_frame: HeadingFrame = "map"
    distance_tolerance_m: Optional[float] = Field(default=None, ge=0.01, le=2.0)
    yaw_tolerance_rad: Optional[float] = Field(default=None, ge=0.01, le=3.141592653589793)
    timeout_s: Optional[float] = Field(default=None, ge=1.0, le=600.0)
    max_continuous_retries: Optional[int] = Field(default=None, ge=0, le=100)
    replace_current: bool = False
    generated_at_ms: Optional[int] = None
    max_age_ms: int = Field(default=1000, ge=50, le=10000)


class MapGoalRequest(BaseModel):
    request_id: Optional[str] = None
    x_m: float
    y_m: float
    yaw_rad: float = Field(..., ge=-3.141592653589793, le=3.141592653589793)
    distance_tolerance_m: Optional[float] = Field(default=None, ge=0.01, le=2.0)
    yaw_tolerance_rad: Optional[float] = Field(default=None, ge=0.01, le=3.141592653589793)
    timeout_s: Optional[float] = Field(default=None, ge=1.0, le=600.0)
    max_continuous_retries: Optional[int] = Field(default=None, ge=0, le=100)
    replace_current: bool = False
    generated_at_ms: Optional[int] = None
    max_age_ms: int = Field(default=1000, ge=50, le=10000)


class VelocityCommandRequest(BaseModel):
    request_id: Optional[str] = None
    source: str = Field(default="planner", min_length=1, max_length=64)
    linear_mps: float
    angular_rps: float
    replace_current: bool = False
    generated_at_ms: Optional[int] = None
    max_age_ms: int = Field(default=300, ge=50, le=3000)


class DirectRotateRequest(BaseModel):
    angle_rad: float = Field(..., ge=-6.283185307179586, le=6.283185307179586)
    max_angular_rps: Optional[float] = Field(default=None, ge=0.05, le=0.70)
    tolerance_rad: Optional[float] = Field(default=None, ge=0.008726646, le=0.35)
    timeout_s: Optional[float] = Field(default=None, ge=1.0, le=120.0)
    rate_hz: Optional[float] = Field(default=None, ge=5.0, le=30.0)
    replace_current: bool = True


class DirectDistanceRequest(BaseModel):
    distance_m: float = Field(..., ge=-20.0, le=20.0)
    speed_mps: Optional[float] = Field(default=None, ge=0.03, le=0.35)
    tolerance_m: Optional[float] = Field(default=None, ge=0.005, le=0.10)
    heading_hold: Optional[bool] = None
    max_heading_correction_rps: Optional[float] = Field(default=None, ge=0.0, le=0.50)
    timeout_s: Optional[float] = Field(default=None, ge=1.0, le=300.0)
    rate_hz: Optional[float] = Field(default=None, ge=5.0, le=30.0)
    replace_current: bool = True


class DirectRelativeRequest(BaseModel):
    heading_rad: float = Field(..., ge=-3.141592653589793, le=3.141592653589793)
    distance_m: float = Field(..., ge=-20.0, le=20.0)
    linear_speed_mps: Optional[float] = Field(default=None, ge=0.03, le=0.35)
    angular_speed_rps: Optional[float] = Field(default=None, ge=0.05, le=0.70)
    distance_tolerance_m: Optional[float] = Field(default=None, ge=0.005, le=0.10)
    angle_tolerance_rad: Optional[float] = Field(default=None, ge=0.008726646, le=0.35)
    timeout_s: Optional[float] = Field(default=None, ge=1.0, le=300.0)
    rate_hz: Optional[float] = Field(default=None, ge=5.0, le=30.0)
    replace_current: bool = True


class EstopRequest(BaseModel):
    engaged: bool


class StopRequest(BaseModel):
    reason: Optional[str] = Field(default=None, max_length=200)


class NavParamsRequest(BaseModel):
    max_speed_linear: Optional[float] = Field(default=None, ge=0.1, le=1.0)
    max_speed_angular: Optional[float] = Field(default=None, ge=0.5, le=3.5)


class PlanDistanceRequest(BaseModel):
    start_x: float
    start_y: float
    start_floor: int
    goal_x: float
    goal_y: float
    goal_floor: int


class TaskRecordModel(BaseModel):
    task_id: str
    request_id: str
    vendor_task_id: Optional[str] = None
    state: TaskState
    kind: Literal["relative", "map_goal"]
    created_at_ms: int
    started_at_ms: Optional[int] = None
    finished_at_ms: Optional[int] = None
    timeout_s: float
    has_seen_running: bool = False
    target: dict
    error: Optional[dict] = None


class ApiError(BaseModel):
    code: str
    message: str
    details: Optional[dict] = None


class ApiErrorEnvelope(BaseModel):
    ok: bool = False
    error: ApiError
