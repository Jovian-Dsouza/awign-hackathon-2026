"""Orders, tasks, QC, robot, activity, overview and analytics endpoints."""

from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException, Query
from fastapi.responses import FileResponse

from app.api.deps import Services, services
from app.config import settings
from app.models.schemas import (
    ActivityEvent, AssignRequest, CatalogItem, CommandResult, Order, OrderCreate, OrderDetail,
    Overview, QcRecord, ReviewRequest, RobotCommand, RobotStatus, Task,
)
from app.services import analytics
from app.services.workflow import DESTINATIONS, WorkflowError

router = APIRouter(prefix="/api")


def _qc_out(rec: dict) -> dict:
    rec = dict(rec)
    rec["camera_evidence_url"] = f"/api/qc/{rec['id']}/evidence" if rec.pop("camera_evidence", None) else None
    return rec


def _raise(err: WorkflowError):
    raise HTTPException(err.status_code, detail=str(err))


# ---------------- catalog ----------------
@router.get("/catalog", response_model=list[CatalogItem], tags=["catalog"])
def catalog(s: Services = Depends(services)):
    return sorted(s.catalog.values(), key=lambda c: c.location)


@router.get("/destinations", response_model=list[str], tags=["catalog"])
def destinations():
    return list(DESTINATIONS)


# ---------------- orders ----------------
@router.get("/orders", response_model=list[Order], tags=["orders"])
def list_orders(status: str | None = Query(None, description="order status, or 'active'"),
                limit: int = Query(200, le=500), s: Services = Depends(services)):
    return s.repo.list_orders(status, limit)


@router.post("/orders", response_model=Order, status_code=201, tags=["orders"])
def create_order(payload: OrderCreate, s: Services = Depends(services)):
    try:
        return s.engine.create_order(payload)
    except WorkflowError as e:
        _raise(e)


@router.post("/orders/demo", response_model=Order, status_code=201, tags=["orders"])
def create_demo_order(s: Services = Depends(services)):
    return s.engine.create_demo_order()


@router.get("/orders/{order_id}", response_model=OrderDetail, tags=["orders"])
def get_order(order_id: str, s: Services = Depends(services)):
    order = s.repo.get_order(order_id)
    if order is None:
        raise HTTPException(404, f"Order {order_id} not found")
    tasks = s.repo.list_tasks(order_id=order_id)
    for t in tasks:
        t["events"] = s.repo.task_events(t["id"])
    return {**order, "tasks": tasks, "qc": [_qc_out(q) for q in s.repo.list_qc(order_id=order_id)]}


@router.post("/orders/{order_id}/assign", response_model=Order, tags=["orders"])
def assign_order(order_id: str, body: AssignRequest, s: Services = Depends(services)):
    try:
        return s.engine.assign(order_id, body.robot_id)
    except WorkflowError as e:
        _raise(e)


# ---------------- tasks ----------------
@router.get("/tasks", response_model=list[Task], tags=["tasks"])
def list_tasks(order_id: str | None = None,
               status: str | None = Query(None, description="task status, or 'active'"),
               limit: int = Query(300, le=1000), s: Services = Depends(services)):
    return s.repo.list_tasks(order_id=order_id, status=status, limit=limit)


@router.get("/tasks/{task_id}", response_model=Task, tags=["tasks"])
def get_task(task_id: str, s: Services = Depends(services)):
    task = s.repo.get_task(task_id, with_events=True)
    if task is None:
        raise HTTPException(404, f"Task {task_id} not found")
    return task


# ---------------- QC ----------------
@router.get("/qc", response_model=list[QcRecord], tags=["qc"])
def list_qc(status: str | None = Query(None, description="PASS|FAIL|REVIEW|PENDING|flagged"),
            order_id: str | None = None, s: Services = Depends(services)):
    return [_qc_out(q) for q in s.repo.list_qc(status=status, order_id=order_id)]


@router.post("/qc/{task_id}/review", response_model=QcRecord, tags=["qc"])
def review(task_id: str, body: ReviewRequest, s: Services = Depends(services)):
    try:
        return _qc_out(s.engine.review(task_id, body.decision, body.reviewer.strip(), body.notes))
    except WorkflowError as e:
        _raise(e)


@router.get("/qc/{qc_id}/evidence", tags=["qc"])
def qc_evidence(qc_id: str, s: Services = Depends(services)):
    rec = s.repo.get_qc(qc_id)
    if rec is None or not rec["camera_evidence"]:
        raise HTTPException(404, "No camera evidence for this record")
    path = (settings.evidence_dir / rec["camera_evidence"]).resolve()
    if path.parent != settings.evidence_dir.resolve() or not path.is_file():
        raise HTTPException(404, "Evidence file missing")
    return FileResponse(path, media_type="image/jpeg")


# ---------------- robot ----------------
@router.get("/robot/status", response_model=RobotStatus, tags=["robot"])
def robot_status(s: Services = Depends(services)):
    return s.engine.robot_status()


@router.post("/robot/commands", response_model=CommandResult, tags=["robot"])
def robot_command(cmd: RobotCommand, s: Services = Depends(services)):
    try:
        accepted, message = s.engine.command(cmd.command, cmd.order_id, cmd.anomaly)
    except WorkflowError as e:
        _raise(e)
    return {"accepted": accepted, "message": message, "simulated": s.engine.robot.simulated,
            "status": s.engine.robot_status()}


# ---------------- activity / overview / analytics ----------------
@router.get("/activity", response_model=list[ActivityEvent], tags=["activity"])
def activity(limit: int = Query(50, le=500), order_id: str | None = None, s: Services = Depends(services)):
    return s.repo.list_activity(limit, order_id)


@router.get("/overview", response_model=Overview, tags=["overview"])
def overview(s: Services = Depends(services)):
    return s.repo.overview()


@router.get("/analytics", tags=["analytics"])
def get_analytics(s: Services = Depends(services)):
    return analytics.compute(s.repo)


@router.get("/settings", tags=["settings"])
def get_settings(s: Services = Depends(services)) -> dict[str, dict[str, str | int | float | bool | None]]:
    robot = s.engine.robot
    return {
        "camera": {"device_index": s.camera.index, "backend": s.camera.backend_name,
                   "requested_resolution": f"{s.camera.req_width}x{s.camera.req_height}",
                   "requested_fps": s.camera.req_fps, "jpeg_quality": s.camera.jpeg_quality,
                   "overlay": s.camera.overlay},
        "robot": {"adapter": robot.adapter_name, "robot_id": robot.robot_id, "model": robot.model,
                  "simulated": robot.simulated, "sim_speed": settings.sim_speed,
                  "sim_anomaly_rate": settings.sim_anomaly_rate},
        "sensors": {"source": s.engine.sensors.source,
                    "nfc": "simulated" if s.engine.sensors.source == "simulated" else "not connected",
                    "load_cell": "simulated" if s.engine.sensors.source == "simulated" else "not connected"},
        "storage": {"database": str(settings.db_path), "evidence_dir": str(settings.evidence_dir),
                    "inventory": str(settings.inventory_path), "catalog_items": len(s.catalog)},
    }

