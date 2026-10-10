"""SQLite persistence.

A single shared connection guarded by a lock: the app is one process and every
query is tiny, so this keeps things simple. All SQL lives in `repository.py`;
swapping to PostgreSQL/Supabase means reimplementing that module, not the services.
"""

from __future__ import annotations

import sqlite3
import threading
from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path
from typing import Any

SCHEMA = """
CREATE TABLE IF NOT EXISTS orders (
    id            TEXT PRIMARY KEY,
    items         TEXT NOT NULL,          -- JSON [{sku, name, quantity, location}]
    destination   TEXT NOT NULL,
    priority      TEXT NOT NULL,
    status        TEXT NOT NULL,
    robot_id      TEXT,
    created_at    TEXT NOT NULL,
    updated_at    TEXT NOT NULL,
    started_at    TEXT,
    completed_at  TEXT,
    note          TEXT
);

CREATE TABLE IF NOT EXISTS tasks (
    id            TEXT PRIMARY KEY,
    order_id      TEXT NOT NULL REFERENCES orders(id),
    seq           INTEGER NOT NULL,
    line_index    INTEGER,                -- which order line (NULL for order-level tasks)
    type          TEXT NOT NULL,
    title         TEXT NOT NULL,
    sku           TEXT,
    location      TEXT,
    status        TEXT NOT NULL,
    attempts      INTEGER NOT NULL DEFAULT 0,
    source        TEXT NOT NULL,          -- 'simulated' | 'hardware' | 'system'
    created_at    TEXT NOT NULL,
    started_at    TEXT,
    completed_at  TEXT,
    error         TEXT
);
CREATE INDEX IF NOT EXISTS idx_tasks_order ON tasks(order_id, seq);

CREATE TABLE IF NOT EXISTS task_events (
    id        INTEGER PRIMARY KEY AUTOINCREMENT,
    task_id   TEXT NOT NULL REFERENCES tasks(id),
    ts        TEXT NOT NULL,
    status    TEXT NOT NULL,
    message   TEXT NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_task_events_task ON task_events(task_id);

CREATE TABLE IF NOT EXISTS qc_records (
    id                   TEXT PRIMARY KEY,
    task_id              TEXT NOT NULL UNIQUE REFERENCES tasks(id),
    order_id             TEXT NOT NULL REFERENCES orders(id),
    sku                  TEXT NOT NULL,
    item_name            TEXT NOT NULL,
    expected_quantity    INTEGER NOT NULL,
    expected_destination TEXT NOT NULL,
    robot_outcome        TEXT NOT NULL,
    camera_evidence      TEXT,           -- file name under EVIDENCE_DIR
    sensor_evidence      TEXT NOT NULL,  -- JSON
    evidence_source      TEXT NOT NULL,  -- 'simulated' | 'hardware' | 'none'
    status               TEXT NOT NULL,  -- PASS | FAIL | REVIEW | PENDING
    reasons              TEXT NOT NULL,  -- JSON [str]
    review_decision      TEXT,           -- approved | rejected
    reviewer             TEXT,
    review_notes         TEXT,
    reviewed_at          TEXT,
    created_at           TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS activity (
    id        INTEGER PRIMARY KEY AUTOINCREMENT,
    ts        TEXT NOT NULL,
    type      TEXT NOT NULL,
    severity  TEXT NOT NULL,     -- info | success | warning | error
    message   TEXT NOT NULL,
    order_id  TEXT,
    task_id   TEXT,
    source    TEXT NOT NULL      -- 'simulated' | 'hardware' | 'system' | 'user'
);

CREATE TABLE IF NOT EXISTS kv (
    key    TEXT PRIMARY KEY,
    value  TEXT NOT NULL
);
"""


class Database:
    def __init__(self, path: Path | str):
        self.path = str(path)
        if self.path != ":memory:":
            Path(self.path).parent.mkdir(parents=True, exist_ok=True)
        self._conn = sqlite3.connect(self.path, check_same_thread=False, isolation_level=None)
        self._conn.row_factory = sqlite3.Row
        self._conn.execute("PRAGMA journal_mode=WAL")
        self._conn.execute("PRAGMA foreign_keys=ON")
        self._conn.executescript(SCHEMA)
        self._lock = threading.RLock()

    @contextmanager
    def transaction(self) -> Iterator[sqlite3.Connection]:
        with self._lock:
            self._conn.execute("BEGIN")
            try:
                yield self._conn
            except BaseException:
                self._conn.execute("ROLLBACK")
                raise
            else:
                self._conn.execute("COMMIT")

    def execute(self, sql: str, params: tuple | dict = ()) -> sqlite3.Cursor:
        with self._lock:
            return self._conn.execute(sql, params)

    def all(self, sql: str, params: tuple | dict = ()) -> list[dict[str, Any]]:
        with self._lock:
            return [dict(r) for r in self._conn.execute(sql, params).fetchall()]

    def one(self, sql: str, params: tuple | dict = ()) -> dict[str, Any] | None:
        with self._lock:
            row = self._conn.execute(sql, params).fetchone()
            return dict(row) if row else None

    def close(self) -> None:
        with self._lock:
            self._conn.close()
