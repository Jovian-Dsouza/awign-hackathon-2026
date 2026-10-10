"""Pydantic API models."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, Field

Priority = Literal["low", "normal", "high"]
OrderStatus = Literal["queued", "picking", "picked", "verification", "dispatched", "exception"]
TaskStatus = Literal["pending", "running", "completed", "failed", "skipped", "awaiting_review"]
TaskType = Literal["navigate", "identify", "pick", "transport", "place", "verify", "dispatch"]
QcStatus = Literal["PASS", "FAIL", "REVIEW", "PENDING"]
DataSource = Literal["simulated", "hardware", "system", "user", "none"]
RobotMode = Literal["idle", "executing", "paused", "error", "stopped"]


# ---------- catalog ----------
class CatalogItem(BaseModel):
    sku: str
    name: str
    location: str  # e.g. "A-L1-S0"
    shelf: str
    nominal_weight_g: int


# ---------- orders ----------
class OrderLineIn(BaseModel):
    sku: str
    quantity: int = Field(default=1, ge=1, le=10)


class OrderCreate(BaseModel):
    items: list[OrderLineIn] = Field(min_length=1, max_length=10)
    destination: str = "BAY-1"
    priority: Priority = "normal"
    assign_robot: bool = True
    note: str | None = Field(default=None, max_length=200)


class OrderLine(BaseModel):
    sku: str
    name: str
    quantity: int
    location: str


class Order(BaseModel):
    id: str
    items: list[OrderLine]
    destination: str
    priority: Priority
    status: OrderStatus
    robot_id: str | None
    created_at: str
    updated_at: str
    started_at: str | None
    completed_at: str | None
    note: str | None
    progress: float = 0.0  # 0..1 share of robot tasks finished
    flags: int = 0  # QC records needing attention


class AssignRequest(BaseModel):
    robot_id: str


# ---------- tasks ----------
class TaskEvent(BaseModel):
    ts: str
    status: str
    message: str


class Task(BaseModel):
    id: str
    order_id: str
    seq: int
    line_index: int | None
    type: TaskType
    title: str
    sku: str | None
    location: str | None
    status: TaskStatus
    attempts: int
    source: DataSource
    created_at: str
    started_at: str | None
    completed_at: str | None
    error: str | None
    events: list[TaskEvent] | None = None


# ---------- QC ----------
class QcRecord(BaseModel):
    id: str
    task_id: str
    order_id: str
    sku: str
    item_name: str
    expected_quantity: int
    expected_destination: str
    robot_outcome: str
    camera_evidence_url: str | None
    sensor_evidence: dict
    evidence_source: DataSource
    status: QcStatus
    reasons: list[str]
    review_decision: Literal["approved", "rejected"] | None
    reviewer: str | None
    review_notes: str | None
    reviewed_at: str | None
    created_at: str


class ReviewRequest(BaseModel):
    decision: Literal["approved", "rejected"]
    reviewer: str = Field(min_length=1, max_length=60)
    notes: str | None = Field(default=None, max_length=500)


class OrderDetail(Order):
    tasks: list[Task]
    qc: list[QcRecord]


# ---------- activity ----------
class ActivityEvent(BaseModel):
    id: int
    ts: str
    type: str
    severity: Literal["info", "success", "warning", "error"]
    message: str
    order_id: str | None
    task_id: str | None
    source: DataSource


# ---------- robot ----------
class RobotStatus(BaseModel):
    robot_id: str
    name: str
    model: str
    adapter: str
    simulated: bool
    connected: bool
    mode: RobotMode
    current_order_id: str | None
    current_task_id: str | None
    current_task_title: str | None
    zone: str | None
    battery_pct: float | None  # None unless real telemetry exists
    queue_length: int
    last_update: str
    stop_reason: str | None
    pending_anomaly: str | None


AnomalyKind = Literal["weight_mismatch", "nfc_missing", "sku_mismatch", "grasp_failure"]


class RobotCommand(BaseModel):
    command: Literal["start", "pause", "resume", "stop", "reset", "inject_anomaly"]
    order_id: str | None = None  # for "start": run this order next
    anomaly: AnomalyKind | None = None


class CommandResult(BaseModel):
    accepted: bool
    message: str
    simulated: bool
    status: RobotStatus


# ---------- camera ----------
class CameraStatus(BaseModel):
    state: Literal["streaming", "connecting", "error", "stopped"]
    device_index: int
    backend: str
    width: int | None
    height: int | None
    fps: float | None
    frames: int
    last_frame_at: str | None
    error: str | None
    clients: int


class CameraConfig(BaseModel):
    device_index: int = Field(ge=0, le=16)


# ---------- overview / analytics ----------
class Overview(BaseModel):
    active_orders: int
    orders_fulfilled: int
    tasks_in_progress: int
    tasks_awaiting_verification: int
    failed_or_blocked_tasks: int
    avg_fulfillment_seconds: float | None
    fulfilled_sample_size: int
