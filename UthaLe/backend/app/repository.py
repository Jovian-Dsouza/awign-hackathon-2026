"""All SQL lives here. Services talk to `Repository`, never to sqlite directly."""

from __future__ import annotations

import json
from datetime import UTC, datetime
from typing import Any

from app.database import Database

ACTIVE_ORDER_STATUSES = ("queued", "picking", "picked", "verification")
ROBOT_TASK_TYPES = ("navigate", "identify", "pick", "transport", "place", "verify")


def now_iso() -> str:
    return datetime.now(UTC).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def parse_iso(ts: str) -> datetime:
    return datetime.fromisoformat(ts.replace("Z", "+00:00"))


class Repository:
    def __init__(self, db: Database):
        self.db = db

    # ---------------- ids / kv ----------------
    def kv_get(self, key: str, default: str | None = None) -> str | None:
        row = self.db.one("SELECT value FROM kv WHERE key = ?", (key,))
        return row["value"] if row else default

    def kv_set(self, key: str, value: str) -> None:
        self.db.execute(
            "INSERT INTO kv(key, value) VALUES(?, ?) ON CONFLICT(key) DO UPDATE SET value = excluded.value",
            (key, value),
        )

    def next_id(self, prefix: str, start: int) -> str:
        with self.db.transaction() as conn:
            row = conn.execute("SELECT value FROM kv WHERE key = ?", (f"seq:{prefix}",)).fetchone()
            n = int(row["value"]) + 1 if row else start
            conn.execute(
                "INSERT INTO kv(key, value) VALUES(?, ?) ON CONFLICT(key) DO UPDATE SET value = excluded.value",
                (f"seq:{prefix}", str(n)),
            )
        return f"{prefix}-{n}"

    # ---------------- orders ----------------
    def create_order(self, order_id: str, items: list[dict], destination: str, priority: str,
                     robot_id: str | None, note: str | None) -> dict:
        ts = now_iso()
        self.db.execute(
            "INSERT INTO orders(id, items, destination, priority, status, robot_id, created_at, updated_at, note)"
            " VALUES(?, ?, ?, ?, 'queued', ?, ?, ?, ?)",
            (order_id, json.dumps(items), destination, priority, robot_id, ts, ts, note),
        )
        return self.get_order(order_id)  # type: ignore[return-value]

    def _hydrate_order(self, row: dict) -> dict:
        row["items"] = json.loads(row["items"])
        stats = self.db.one(
            "SELECT"
            "  SUM(CASE WHEN type IN ('navigate','identify','pick','transport','place','verify','dispatch') THEN 1 ELSE 0 END) AS total,"
            "  SUM(CASE WHEN status IN ('completed','skipped','failed','awaiting_review') THEN 1 ELSE 0 END) AS done"
            " FROM tasks WHERE order_id = ?",
            (row["id"],),
        ) or {}
        total, done = stats.get("total") or 0, stats.get("done") or 0
        row["progress"] = round(done / total, 3) if total else 0.0
        flags = self.db.one(
            "SELECT COUNT(*) AS n FROM qc_records WHERE order_id = ? AND status IN ('FAIL','REVIEW')"
            " AND review_decision IS NULL",
            (row["id"],),
        )
        row["flags"] = flags["n"] if flags else 0
        return row

    def get_order(self, order_id: str) -> dict | None:
        row = self.db.one("SELECT * FROM orders WHERE id = ?", (order_id,))
        return self._hydrate_order(row) if row else None

    def list_orders(self, status: str | None = None, limit: int = 200) -> list[dict]:
        if status == "active":
            marks = ",".join("?" * len(ACTIVE_ORDER_STATUSES))
            rows = self.db.all(
                f"SELECT * FROM orders WHERE status IN ({marks}) ORDER BY created_at DESC LIMIT ?",
                (*ACTIVE_ORDER_STATUSES, limit),
            )
        elif status:
            rows = self.db.all(
                "SELECT * FROM orders WHERE status = ? ORDER BY created_at DESC LIMIT ?", (status, limit)
            )
        else:
            rows = self.db.all("SELECT * FROM orders ORDER BY created_at DESC LIMIT ?", (limit,))
        return [self._hydrate_order(r) for r in rows]

    def update_order(self, order_id: str, **fields: Any) -> None:
        fields["updated_at"] = now_iso()
        cols = ", ".join(f"{k} = ?" for k in fields)
        self.db.execute(f"UPDATE orders SET {cols} WHERE id = ?", (*fields.values(), order_id))

    def next_order_for_robot(self, robot_id: str) -> dict | None:
        """In-progress orders first (resume after restart), then by priority, then FIFO."""
        row = self.db.one(
            "SELECT * FROM orders WHERE robot_id = ? AND status IN ('picking', 'queued')"
            " ORDER BY CASE status WHEN 'picking' THEN 0 ELSE 1 END,"
            "          CASE priority WHEN 'high' THEN 0 WHEN 'normal' THEN 1 ELSE 2 END,"
            "          created_at"
            " LIMIT 1",
            (robot_id,),
        )
        return self._hydrate_order(row) if row else None

    def robot_queue_length(self, robot_id: str) -> int:
        row = self.db.one(
            "SELECT COUNT(*) AS n FROM orders WHERE robot_id = ? AND status IN ('queued','picking')", (robot_id,)
        )
        return row["n"] if row else 0

    # ---------------- tasks ----------------
    def create_tasks(self, order_id: str, tasks: list[dict]) -> None:
        ts = now_iso()
        with self.db.transaction() as conn:
            for t in tasks:
                conn.execute(
                    "INSERT INTO tasks(id, order_id, seq, line_index, type, title, sku, location, status,"
                    " source, created_at) VALUES(?, ?, ?, ?, ?, ?, ?, ?, 'pending', ?, ?)",
                    (t["id"], order_id, t["seq"], t.get("line_index"), t["type"], t["title"],
                     t.get("sku"), t.get("location"), t["source"], ts),
                )

    def get_task(self, task_id: str, with_events: bool = False) -> dict | None:
        row = self.db.one("SELECT * FROM tasks WHERE id = ?", (task_id,))
        if row and with_events:
            row["events"] = self.task_events(task_id)
        return row

    def list_tasks(self, order_id: str | None = None, status: str | None = None,
                   limit: int = 300) -> list[dict]:
        where, params = [], []
        if order_id:
            where.append("order_id = ?")
            params.append(order_id)
        if status == "active":
            where.append("status IN ('running', 'awaiting_review', 'failed')")
        elif status:
            where.append("status = ?")
            params.append(status)
        clause = f"WHERE {' AND '.join(where)}" if where else ""
        order = "ORDER BY seq" if order_id else "ORDER BY COALESCE(started_at, created_at) DESC, seq DESC"
        return self.db.all(f"SELECT * FROM tasks {clause} {order} LIMIT ?", (*params, limit))

    def update_task(self, task_id: str, **fields: Any) -> None:
        cols = ", ".join(f"{k} = ?" for k in fields)
        self.db.execute(f"UPDATE tasks SET {cols} WHERE id = ?", (*fields.values(), task_id))

    def add_task_event(self, task_id: str, status: str, message: str) -> None:
        self.db.execute(
            "INSERT INTO task_events(task_id, ts, status, message) VALUES(?, ?, ?, ?)",
            (task_id, now_iso(), status, message),
        )

    def task_events(self, task_id: str) -> list[dict]:
        return self.db.all(
            "SELECT ts, status, message FROM task_events WHERE task_id = ? ORDER BY id", (task_id,)
        )

    def requeue_interrupted_tasks(self) -> list[dict]:
        rows = self.db.all("SELECT * FROM tasks WHERE status = 'running'")
        for r in rows:
            # The interrupted attempt never finished, so it shouldn't use up the retry budget.
            self.update_task(r["id"], status="pending", attempts=max(r["attempts"] - 1, 0))
            self.add_task_event(r["id"], "pending", "Backend restarted mid-task; task re-queued")
        return rows

    # ---------------- QC ----------------
    def create_qc(self, rec: dict) -> None:
        self.db.execute(
            "INSERT INTO qc_records(id, task_id, order_id, sku, item_name, expected_quantity,"
            " expected_destination, robot_outcome, camera_evidence, sensor_evidence, evidence_source,"
            " status, reasons, created_at)"
            " VALUES(:id, :task_id, :order_id, :sku, :item_name, :expected_quantity, :expected_destination,"
            " :robot_outcome, :camera_evidence, :sensor_evidence, :evidence_source, :status, :reasons, :created_at)",
            {**rec, "sensor_evidence": json.dumps(rec["sensor_evidence"]),
             "reasons": json.dumps(rec["reasons"]), "created_at": now_iso()},
        )

    @staticmethod
    def _hydrate_qc(row: dict) -> dict:
        row["sensor_evidence"] = json.loads(row["sensor_evidence"])
        row["reasons"] = json.loads(row["reasons"])
        return row

    def list_qc(self, status: str | None = None, order_id: str | None = None, limit: int = 300) -> list[dict]:
        where, params = [], []
        if status == "flagged":
            where.append("status IN ('FAIL','REVIEW') AND review_decision IS NULL")
        elif status:
            where.append("status = ?")
            params.append(status)
        if order_id:
            where.append("order_id = ?")
            params.append(order_id)
        clause = f"WHERE {' AND '.join(where)}" if where else ""
        rows = self.db.all(f"SELECT * FROM qc_records {clause} ORDER BY created_at DESC LIMIT ?", (*params, limit))
        return [self._hydrate_qc(r) for r in rows]

    def get_qc_by_task(self, task_id: str) -> dict | None:
        row = self.db.one("SELECT * FROM qc_records WHERE task_id = ?", (task_id,))
        return self._hydrate_qc(row) if row else None

    def get_qc(self, qc_id: str) -> dict | None:
        row = self.db.one("SELECT * FROM qc_records WHERE id = ?", (qc_id,))
        return self._hydrate_qc(row) if row else None

    def update_qc(self, qc_id: str, **fields: Any) -> None:
        cols = ", ".join(f"{k} = ?" for k in fields)
        self.db.execute(f"UPDATE qc_records SET {cols} WHERE id = ?", (*fields.values(), qc_id))

    # ---------------- activity ----------------
    def log(self, type_: str, message: str, *, severity: str = "info", source: str = "system",
            order_id: str | None = None, task_id: str | None = None) -> None:
        self.db.execute(
            "INSERT INTO activity(ts, type, severity, message, order_id, task_id, source)"
            " VALUES(?, ?, ?, ?, ?, ?, ?)",
            (now_iso(), type_, severity, message, order_id, task_id, source),
        )

    def list_activity(self, limit: int = 50, order_id: str | None = None) -> list[dict]:
        if order_id:
            return self.db.all(
                "SELECT * FROM activity WHERE order_id = ? ORDER BY id DESC LIMIT ?", (order_id, limit)
            )
        return self.db.all("SELECT * FROM activity ORDER BY id DESC LIMIT ?", (limit,))

    # ---------------- overview / analytics ----------------
    def overview(self) -> dict:
        marks = ",".join("?" * len(ACTIVE_ORDER_STATUSES))
        active = self.db.one(f"SELECT COUNT(*) n FROM orders WHERE status IN ({marks})", ACTIVE_ORDER_STATUSES)
        fulfilled = self.db.all(
            "SELECT created_at, completed_at FROM orders WHERE status = 'dispatched' AND completed_at IS NOT NULL"
        )
        running = self.db.one("SELECT COUNT(*) n FROM tasks WHERE status = 'running'")
        awaiting = self.db.one("SELECT COUNT(*) n FROM tasks WHERE status = 'awaiting_review'")
        failed = self.db.one(
            "SELECT COUNT(*) n FROM tasks t JOIN orders o ON o.id = t.order_id"
            " WHERE t.status = 'failed' AND o.status != 'dispatched'"
        )
        durations = [
            (parse_iso(r["completed_at"]) - parse_iso(r["created_at"])).total_seconds() for r in fulfilled
        ]
        return {
            "active_orders": active["n"],
            "orders_fulfilled": len(fulfilled),
            "tasks_in_progress": running["n"],
            "tasks_awaiting_verification": awaiting["n"],
            "failed_or_blocked_tasks": failed["n"],
            "avg_fulfillment_seconds": round(sum(durations) / len(durations), 1) if durations else None,
            "fulfilled_sample_size": len(durations),
        }

    def analytics_rows(self) -> dict[str, list[dict]]:
        return {
            "orders": self.db.all("SELECT id, status, created_at, started_at, completed_at FROM orders"),
            "tasks": self.db.all("SELECT type, status, error, source FROM tasks"),
            "qc": self.db.all("SELECT status, review_decision, reasons, evidence_source FROM qc_records"),
        }
