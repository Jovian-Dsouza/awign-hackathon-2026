"""Order -> task workflow.

An order becomes, per line: navigate, identify, pick, transport, place, verify,
plus one order-level `dispatch` task. A background loop feeds the robot one
order at a time through a `RobotAdapter`; every state change is persisted, so a
backend restart resumes where it left off (a step that was mid-flight is re-run).

Order status: queued -> picking -> picked -> verification -> dispatched
                                                         \\-> exception (QC rejected)
"""

from __future__ import annotations

import asyncio
import logging
import random
import uuid
from pathlib import Path

from app.models.schemas import CatalogItem, OrderCreate
from app.repository import Repository, now_iso
from app.services import qc
from app.services.camera import CameraService
from app.services.robot import RobotAdapter, RobotStopped, StepCommand
from app.services.sensors import SensorHub

log = logging.getLogger(__name__)

MAX_PICK_ATTEMPTS = 2
DESTINATIONS = ("BAY-1", "BAY-2", "BAY-3")
LINE_STEPS = (
    ("navigate", "Navigate to shelf {loc}"),
    ("identify", "Identify {name}"),
    ("pick", "Pick {name}"),
    ("transport", "Transport to packing station"),
    ("place", "Place in {dest} tote"),
    ("verify", "Verify placement of {name}"),
)


class WorkflowError(Exception):
    def __init__(self, status_code: int, message: str):
        super().__init__(message)
        self.status_code = status_code


class WorkflowEngine:
    def __init__(self, repo: Repository, catalog: dict[str, CatalogItem], robot: RobotAdapter,
                 sensors: SensorHub, camera: CameraService, evidence_dir: Path):
        self.repo = repo
        self.catalog = catalog
        self.robot = robot
        self.sensors = sensors
        self.camera = camera
        self.evidence_dir = evidence_dir
        evidence_dir.mkdir(parents=True, exist_ok=True)
        self.current_order_id: str | None = None
        self.current_task: dict | None = None
        self.last_update = now_iso()
        self._loop_task: asyncio.Task | None = None
        self._wake = asyncio.Event()
        self._step_source = "simulated" if robot.simulated else "hardware"

    # ------------------------------------------------------------------ lifecycle
    async def start(self) -> None:
        for t in self.repo.requeue_interrupted_tasks():
            self.repo.log("system", f"Backend restarted during '{t['title']}'; step will be re-run",
                          severity="warning", order_id=t["order_id"], task_id=t["id"])
        if self.repo.kv_get("robot:stopped") == "1":
            self.robot.stop()
        if self.repo.kv_get("robot:paused") == "1":
            self.robot.pause()
        self._loop_task = asyncio.create_task(self._loop(), name="workflow-loop")

    async def shutdown(self) -> None:
        if self._loop_task:
            self._loop_task.cancel()
            try:
                await self._loop_task
            except (asyncio.CancelledError, Exception):
                pass

    # ------------------------------------------------------------------ orders
    def create_order(self, payload: OrderCreate) -> dict:
        lines = []
        for line in payload.items:
            item = self.catalog.get(line.sku)
            if item is None:
                raise WorkflowError(422, f"Unknown SKU '{line.sku}'")
            lines.append({"sku": item.sku, "name": item.name, "quantity": line.quantity, "location": item.location})
        if payload.destination not in DESTINATIONS:
            raise WorkflowError(422, f"Unknown destination '{payload.destination}'")

        order_id = self.repo.next_id("ORD", 2001)
        robot_id = self.robot.robot_id if payload.assign_robot else None
        self.repo.create_order(order_id, lines, payload.destination, payload.priority, robot_id, payload.note)
        self._create_tasks(order_id, lines, payload.destination)
        total_units = sum(l["quantity"] for l in lines)
        self.repo.log("order_created", f"New order {order_id}: {len(lines)} line(s), {total_units} unit(s) → {payload.destination}",
                      source="user", order_id=order_id)
        if robot_id:
            self._log_assigned(order_id, robot_id)
        self._wake.set()
        return self.repo.get_order(order_id)  # type: ignore[return-value]

    def create_demo_order(self) -> dict:
        skus = random.sample(list(self.catalog), k=min(len(self.catalog), random.randint(1, 3)))
        payload = OrderCreate(
            items=[{"sku": s, "quantity": random.choice((1, 1, 1, 2))} for s in skus],
            destination=random.choice(DESTINATIONS),
            priority=random.choices(("low", "normal", "high"), weights=(1, 4, 1))[0],
            note="Demo order",
        )
        return self.create_order(payload)

    def _create_tasks(self, order_id: str, lines: list[dict], dest: str) -> None:
        tasks, seq = [], 0
        for idx, line in enumerate(lines):
            name = line["name"] + (f" ×{line['quantity']}" if line["quantity"] > 1 else "")
            for type_, title in LINE_STEPS:
                seq += 1
                tasks.append({
                    "id": f"TSK-{uuid.uuid4().hex[:8].upper()}", "seq": seq, "line_index": idx, "type": type_,
                    "title": title.format(loc=line["location"], name=name, dest=dest),
                    "sku": line["sku"], "location": line["location"],
                    "source": "system" if type_ == "verify" else self._step_source,
                })
        seq += 1
        tasks.append({"id": f"TSK-{uuid.uuid4().hex[:8].upper()}", "seq": seq, "line_index": None,
                      "type": "dispatch", "title": "Mark order ready for dispatch", "source": "system"})
        self.repo.create_tasks(order_id, tasks)
        self.repo.log("mission_created", f"Picking mission created for {order_id} ({len(tasks)} tasks)",
                      order_id=order_id)

    def assign(self, order_id: str, robot_id: str) -> dict:
        order = self.repo.get_order(order_id)
        if order is None:
            raise WorkflowError(404, f"Order {order_id} not found")
        if robot_id != self.robot.robot_id:
            raise WorkflowError(404, f"Robot {robot_id} not found")
        if order["status"] != "queued":
            raise WorkflowError(409, f"Order {order_id} is {order['status']}; only queued orders can be assigned")
        self.repo.update_order(order_id, robot_id=robot_id)
        self._log_assigned(order_id, robot_id)
        self._wake.set()
        return self.repo.get_order(order_id)  # type: ignore[return-value]

    def _log_assigned(self, order_id: str, robot_id: str) -> None:
        self.repo.log("order_assigned", f"{order_id} assigned to {robot_id}", order_id=order_id)

    # ------------------------------------------------------------------ robot commands
    def command(self, command: str, order_id: str | None = None, anomaly: str | None = None) -> tuple[bool, str]:
        r = self.robot
        sim = " (simulated)" if r.simulated else ""
        if command == "start":
            if r.stopped:
                return False, "Robot is stopped. Reset the stop before starting."
            if order_id:
                order = self.repo.get_order(order_id)
                if order is None:
                    raise WorkflowError(404, f"Order {order_id} not found")
                if order["status"] == "queued" and order["robot_id"] is None:
                    self.assign(order_id, r.robot_id)
                elif order["robot_id"] != r.robot_id or order["status"] not in ("queued", "picking"):
                    return False, f"{order_id} is not waiting for this robot"
            r.resume()
            self.repo.kv_set("robot:paused", "0")
            self.repo.log("robot_command", f"Start mission{sim}" + (f" for {order_id}" if order_id else ""),
                          source="user", order_id=order_id)
            self._wake.set()
            return True, "Robot will run queued missions" + sim
        if command == "pause":
            r.pause()
            self.repo.kv_set("robot:paused", "1")
            self.repo.log("robot_command", f"Pause{sim}", severity="warning", source="user")
            return True, "Paused" + sim
        if command == "resume":
            if r.stopped:
                return False, "Robot is stopped. Reset the stop before resuming."
            r.resume()
            self.repo.kv_set("robot:paused", "0")
            self.repo.log("robot_command", f"Resume{sim}", source="user")
            self._wake.set()
            return True, "Resumed" + sim
        if command == "stop":
            r.stop()
            self.repo.kv_set("robot:stopped", "1")
            self.repo.log("robot_command", f"Software stop requested{sim}. This is not a physical E-stop.",
                          severity="error", source="user", order_id=self.current_order_id)
            return True, "Software stop engaged" + sim
        if command == "reset":
            r.reset()
            self.repo.kv_set("robot:stopped", "0")
            self.repo.kv_set("robot:paused", "0")
            self.repo.log("robot_command", f"Stop reset{sim}; robot may resume missions", source="user")
            self._wake.set()
            return True, "Stop reset" + sim
        if command == "inject_anomaly":
            scenario = getattr(r, "scenario", None)
            if scenario is None:
                return False, "Anomaly injection is only available in simulation"
            if anomaly is None:
                raise WorkflowError(422, "anomaly is required for inject_anomaly")
            scenario.inject(anomaly)
            self.repo.log("robot_command", f"Simulated anomaly '{anomaly}' armed for the next order line",
                          severity="warning", source="user")
            return True, f"'{anomaly}' will affect the next order line (simulation)"
        raise WorkflowError(422, f"Unknown command '{command}'")

    def robot_status(self) -> dict:
        r = self.robot
        tel = r.telemetry()
        if r.stopped:
            mode = "stopped"
        elif r.paused:
            mode = "paused"
        elif self.current_order_id:
            mode = "executing"
        else:
            mode = "idle"
        scenario = getattr(r, "scenario", None)
        return {
            "robot_id": r.robot_id, "name": r.name, "model": r.model, "adapter": r.adapter_name,
            "simulated": r.simulated, "connected": tel.connected, "mode": mode,
            "current_order_id": self.current_order_id,
            "current_task_id": self.current_task["id"] if self.current_task else None,
            "current_task_title": self.current_task["title"] if self.current_task else None,
            "zone": tel.zone, "battery_pct": tel.battery_pct,
            "queue_length": self.repo.robot_queue_length(r.robot_id),
            "last_update": self.last_update,
            "stop_reason": "Software stop engaged from dashboard" if r.stopped else None,
            "pending_anomaly": scenario.pending if scenario else None,
        }

    # ------------------------------------------------------------------ QC review
    def review(self, task_id: str, decision: str, reviewer: str, notes: str | None) -> dict:
        rec = self.repo.get_qc_by_task(task_id)
        if rec is None:
            raise WorkflowError(404, f"No QC record for task {task_id}")
        if rec["review_decision"]:
            raise WorkflowError(409, f"{rec['id']} was already {rec['review_decision']} by {rec['reviewer']}")
        if rec["status"] == "PENDING":
            raise WorkflowError(409, f"{rec['id']} is still being evaluated")
        self.repo.update_qc(rec["id"], review_decision=decision, reviewer=reviewer, review_notes=notes,
                            reviewed_at=now_iso())
        task_status = "completed" if decision == "approved" else "failed"
        self.repo.update_task(task_id, status=task_status, completed_at=now_iso(),
                              error=None if decision == "approved" else f"Rejected by {reviewer}")
        self.repo.add_task_event(task_id, task_status, f"Human review: {decision} by {reviewer}"
                                 + (f" — {notes}" if notes else ""))
        self.repo.log("review_completed",
                      f"{rec['id']} ({rec['item_name']}) {decision} by {reviewer}",
                      severity="success" if decision == "approved" else "error",
                      source="user", order_id=rec["order_id"], task_id=task_id)
        self._finalize_if_resolved(rec["order_id"])
        return self.repo.get_qc(rec["id"])  # type: ignore[return-value]

    def _finalize_if_resolved(self, order_id: str) -> None:
        order = self.repo.get_order(order_id)
        if order is None or order["status"] != "verification":
            return
        records = self.repo.list_qc(order_id=order_id)
        line_count = len(order["items"])
        if len(records) < line_count:
            return
        unresolved = [r for r in records if r["status"] in ("FAIL", "REVIEW", "PENDING") and not r["review_decision"]]
        if unresolved:
            return
        if any(r["review_decision"] == "rejected" for r in records):
            self.repo.update_order(order_id, status="exception")
            dispatch = self._dispatch_task(order_id)
            if dispatch:
                self.repo.update_task(dispatch["id"], status="skipped")
                self.repo.add_task_event(dispatch["id"], "skipped", "Not dispatched: QC rejected")
            self.repo.log("exception", f"{order_id} held: QC rejected; needs manual handling",
                          severity="error", order_id=order_id)
            return
        self._dispatch(order_id)

    def _dispatch_task(self, order_id: str) -> dict | None:
        return next((t for t in self.repo.list_tasks(order_id=order_id) if t["type"] == "dispatch"), None)

    def _dispatch(self, order_id: str) -> None:
        task = self._dispatch_task(order_id)
        ts = now_iso()
        if task:
            self.repo.update_task(task["id"], status="completed", started_at=ts, completed_at=ts, attempts=1)
            self.repo.add_task_event(task["id"], "completed", "All lines verified; order released to dispatch")
        self.repo.update_order(order_id, status="dispatched", completed_at=ts)
        self.repo.log("dispatched", f"{order_id} ready at dispatch", severity="success", order_id=order_id)

    # ------------------------------------------------------------------ execution loop
    async def _loop(self) -> None:
        while True:
            try:
                await self.robot.wait_until_ready()
                order = self.repo.next_order_for_robot(self.robot.robot_id)
                if order is None:
                    self.robot.park()
                    self._wake.clear()
                    try:
                        await asyncio.wait_for(self._wake.wait(), timeout=1.0)
                    except TimeoutError:
                        pass
                    continue
                await self._run_order(order)
            except RobotStopped:
                await asyncio.sleep(0.25)
            except asyncio.CancelledError:
                raise
            except Exception:  # keep the loop alive; surface the problem in the feed
                log.exception("workflow loop error")
                self.repo.log("exception", "Internal workflow error; see backend log", severity="error")
                await asyncio.sleep(2.0)
            finally:
                self.current_task = None
                self.current_order_id = None
                self.last_update = now_iso()

    async def _run_order(self, order: dict) -> None:
        order_id = order["id"]
        self.current_order_id = order_id
        if order["status"] == "queued":
            self.repo.update_order(order_id, status="picking", started_at=now_iso())
            self.repo.log("mission_started", f"{self.robot.robot_id} started mission {order_id}",
                          source=self._step_source, order_id=order_id)

        tasks = self.repo.list_tasks(order_id=order_id)
        failed_lines: set[int] = {t["line_index"] for t in tasks if t["type"] == "pick" and t["status"] == "failed"}
        outcomes: dict[int, tuple[str, str | None]] = {}

        for task in tasks:
            if task["status"] != "pending" or task["type"] == "dispatch":
                continue
            line = order["items"][task["line_index"]]
            idx = task["line_index"]
            if task["type"] == "verify":
                outcome = outcomes.get(idx) or (("pick_failed", None) if idx in failed_lines else ("placed", order["destination"]))
                self._verify(order, task, line, *outcome)
                continue
            if idx in failed_lines:
                self.repo.update_task(task["id"], status="skipped")
                self.repo.add_task_event(task["id"], "skipped", "Skipped: item was not picked")
                continue
            ok, result_outcome = await self._run_step(order, task, line)
            if not ok:
                failed_lines.add(idx)
                outcomes[idx] = (result_outcome, None)
            elif task["type"] == "place":
                outcomes[idx] = ("placed", order["destination"])

        self.current_task = None
        self.repo.update_order(order_id, status="picked")
        self.repo.log("order_picked", f"All lines of {order_id} processed; verifying", order_id=order_id)
        await asyncio.sleep(0.8)
        self.repo.update_order(order_id, status="verification")
        records = self.repo.list_qc(order_id=order_id)
        flagged = [r for r in records if r["status"] in ("FAIL", "REVIEW") and not r["review_decision"]]
        if flagged:
            self.repo.log("exception", f"{order_id} waiting on human review ({len(flagged)} flagged line(s))",
                          severity="warning", order_id=order_id)
        self._finalize_if_resolved(order_id)

    async def _run_step(self, order: dict, task: dict, line: dict) -> tuple[bool, str]:
        line_key = f"{order['id']}:{task['line_index']}"
        attempts = task["attempts"]
        while True:
            await self.robot.wait_until_ready()
            attempts += 1
            self.current_task = task
            self.last_update = now_iso()
            self.repo.update_task(task["id"], status="running", attempts=attempts,
                                  started_at=task["started_at"] or now_iso())
            self.repo.add_task_event(task["id"], "running", f"Attempt {attempts} started")
            if task["type"] in ("navigate", "pick") and attempts == 1:
                verb = "started" if task["type"] == "navigate" else "attempting"
                self.repo.log("task_started", f"{self.robot.robot_id} {verb}: {task['title']}",
                              source=self._step_source, order_id=order["id"], task_id=task["id"])
            try:
                result = await self.robot.execute(StepCommand(
                    type=task["type"], title=task["title"], line_key=line_key, sku=line["sku"],
                    location=line["location"], destination=order["destination"], attempt=attempts))
            except RobotStopped:
                self.repo.update_task(task["id"], status="pending")
                self.repo.add_task_event(task["id"], "pending", "Interrupted by software stop; will re-run")
                self.repo.log("exception", f"'{task['title']}' interrupted by stop", severity="error",
                              order_id=order["id"], task_id=task["id"])
                raise
            self.last_update = now_iso()
            if result.ok:
                self.repo.update_task(task["id"], status="completed", completed_at=now_iso(), error=None)
                self.repo.add_task_event(task["id"], "completed", result.detail)
                if task["type"] == "pick":
                    self.repo.log("item_picked", f"Picked {line['name']} from {line['location']}",
                                  severity="success", source=self._step_source, order_id=order["id"], task_id=task["id"])
                elif task["type"] == "place":
                    self.repo.log("item_delivered", f"Delivered {line['name']} to {order['destination']} tote",
                                  severity="success", source=self._step_source, order_id=order["id"], task_id=task["id"])
                return True, result.outcome
            self.repo.add_task_event(task["id"], "failed", f"Attempt {attempts}: {result.detail}")
            if task["type"] == "pick" and attempts < MAX_PICK_ATTEMPTS:
                self.repo.log("exception", f"Grasp failed for {line['name']}; retrying", severity="warning",
                              source=self._step_source, order_id=order["id"], task_id=task["id"])
                continue
            self.repo.update_task(task["id"], status="failed", completed_at=now_iso(), error=result.detail)
            self.repo.log("exception", f"{task['title']} failed after {attempts} attempt(s): {result.detail}",
                          severity="error", source=self._step_source, order_id=order["id"], task_id=task["id"])
            return False, result.outcome

    def _verify(self, order: dict, task: dict, line: dict, robot_outcome: str, robot_dest: str | None) -> None:
        ts = now_iso()
        self.current_task = task
        self.repo.update_task(task["id"], status="running", attempts=task["attempts"] + 1, started_at=ts)
        item = self.catalog.get(line["sku"])
        nominal = item.nominal_weight_g if item else 300
        line_key = f"{order['id']}:{task['line_index']}"

        sensors = self.sensors.read_placement(line_key, line["sku"], line["quantity"], nominal,
                                              {s: c.nominal_weight_g for s, c in self.catalog.items()})
        qc_id = f"QC-{uuid.uuid4().hex[:6].upper()}"
        evidence = None
        jpeg = self.camera.latest_jpeg()
        if jpeg:
            evidence = f"{qc_id}.jpg"
            (self.evidence_dir / evidence).write_bytes(jpeg)

        status, reasons = qc.evaluate(
            qc.Expected(sku=line["sku"], quantity=line["quantity"], destination=order["destination"],
                        nominal_weight_g=nominal),
            robot_outcome, robot_dest, sensors)
        self.repo.create_qc({
            "id": qc_id, "task_id": task["id"], "order_id": order["id"], "sku": line["sku"],
            "item_name": line["name"], "expected_quantity": line["quantity"],
            "expected_destination": order["destination"], "robot_outcome": robot_outcome,
            "camera_evidence": evidence, "sensor_evidence": sensors, "evidence_source": self.sensors.source,
            "status": status, "reasons": reasons,
        })
        sim_note = " (simulated sensors)" if self.sensors.source == "simulated" else ""
        if status == "PASS":
            self.repo.update_task(task["id"], status="completed", completed_at=now_iso())
            self.repo.add_task_event(task["id"], "completed", f"{qc_id}: PASS{sim_note}")
            self.repo.log("qc_pass", f"{qc_id} PASS: {line['name']} verified in {order['destination']}{sim_note}",
                          severity="success", order_id=order["id"], task_id=task["id"])
        else:
            self.repo.update_task(task["id"], status="awaiting_review", error="; ".join(reasons))
            self.repo.add_task_event(task["id"], "awaiting_review", f"{qc_id}: {status}: {'; '.join(reasons)}")
            self.repo.log("qc_flagged", f"{qc_id} {status}: {line['name']}: {reasons[0]}",
                          severity="error" if status == "FAIL" else "warning",
                          order_id=order["id"], task_id=task["id"])
