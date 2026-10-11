"""SQLite store for incidents and the feedback loop.

Every incident keeps what the camera saw, what the rules said, what the model said (if
any), what was shown to the admin, and what the admin finally decided. That last part is
the point: admin verdicts are the labelled data that Laya is fine-tuned on, and the
agreement report tells us when a model has earned more authority.
"""

from __future__ import annotations

import json
import sqlite3
import threading
import time
from pathlib import Path
from typing import Any

from overwatch.decision.describe import describe
from overwatch.decision.questions import QUESTIONS, VERDICT_ACTION, expected_from_verdict

SCHEMA = """
CREATE TABLE IF NOT EXISTS incidents (
    id            INTEGER PRIMARY KEY AUTOINCREMENT,
    ts            REAL NOT NULL,
    kind          TEXT NOT NULL,
    zone          TEXT NOT NULL,
    track_id      INTEGER,
    status        TEXT NOT NULL,          -- open | actioned | escalated | dismissed | filtered
    observation   TEXT NOT NULL,          -- JSON
    final         TEXT NOT NULL,          -- JSON Decision shown to the admin
    rules         TEXT NOT NULL,          -- JSON Decision
    laya          TEXT,                   -- JSON Decision or NULL
    snapshot      TEXT,                   -- file name under data/snapshots
    verdict       TEXT,
    verdict_note  TEXT,
    verdict_sev   INTEGER,
    verdict_ts    REAL,
    responder     TEXT,                   -- JSON result of the last responder call
    cleared_ts    REAL                    -- the object was no longer visible at this time
);
CREATE INDEX IF NOT EXISTS idx_incidents_status ON incidents(status);
CREATE INDEX IF NOT EXISTS idx_incidents_track ON incidents(track_id);
"""

_JSON_COLS = ("observation", "final", "rules", "laya", "responder")


def _row(r: sqlite3.Row | None) -> dict[str, Any] | None:
    if r is None:
        return None
    d = dict(r)
    for c in _JSON_COLS:
        d[c] = json.loads(d[c]) if d.get(c) else None
    d["id"] = int(d["id"])
    d["code"] = f"INC-{d['id']:04d}"
    return d


class IncidentStore:
    def __init__(self, path: Path | str):
        if str(path) != ":memory:":
            Path(path).parent.mkdir(parents=True, exist_ok=True)
        self._conn = sqlite3.connect(str(path), check_same_thread=False, isolation_level=None)
        self._conn.row_factory = sqlite3.Row
        self._lock = threading.Lock()
        with self._lock:
            self._conn.executescript(SCHEMA)

    def add(self, *, kind: str, zone: str, track_id: int | None, observation: dict, final: dict,
            rules: dict, laya: dict | None, snapshot: str | None, status: str = "open") -> dict:
        with self._lock:
            cur = self._conn.execute(
                "INSERT INTO incidents(ts,kind,zone,track_id,status,observation,final,rules,laya,snapshot) "
                "VALUES(?,?,?,?,?,?,?,?,?,?)",
                (time.time(), kind, zone, track_id, status, json.dumps(observation), json.dumps(final),
                 json.dumps(rules), json.dumps(laya) if laya else None, snapshot),
            )
            return self.get(int(cur.lastrowid), _locked=True)  # type: ignore[return-value]

    def get(self, incident_id: int, _locked: bool = False) -> dict | None:
        def q():
            return _row(self._conn.execute("SELECT * FROM incidents WHERE id=?", (incident_id,)).fetchone())
        if _locked:
            return q()
        with self._lock:
            return q()

    def list(self, status: str | None = None, limit: int = 100) -> list[dict]:
        """status "open" | "filtered"; None = everything that was alerted (filtered false alarms excluded)."""
        with self._lock:
            if status in ("open", "filtered"):
                rows = self._conn.execute(
                    "SELECT * FROM incidents WHERE status=? ORDER BY id DESC LIMIT ?", (status, limit))
            else:
                rows = self._conn.execute(
                    "SELECT * FROM incidents WHERE status!='filtered' ORDER BY id DESC LIMIT ?", (limit,))
            return [_row(r) for r in rows.fetchall()]  # type: ignore[misc]

    def set_verdict(self, incident_id: int, verdict: str, note: str, severity: int | None,
                    status: str, responder: dict | None) -> dict | None:
        with self._lock:
            self._conn.execute(
                "UPDATE incidents SET verdict=?, verdict_note=?, verdict_sev=?, verdict_ts=?, status=?, responder=? "
                "WHERE id=?",
                (verdict, note, severity, time.time(), status, json.dumps(responder) if responder else None,
                 incident_id),
            )
            return self.get(incident_id, _locked=True)

    def mark_cleared(self, track_id: int) -> list[int]:
        """The object behind this track is gone. Returns the incident ids it applied to."""
        with self._lock:
            ids = [r["id"] for r in self._conn.execute(
                "SELECT id FROM incidents WHERE track_id=? AND cleared_ts IS NULL", (track_id,)).fetchall()]
            self._conn.execute("UPDATE incidents SET cleared_ts=? WHERE track_id=? AND cleared_ts IS NULL",
                               (time.time(), track_id))
            return ids

    # -- feedback loop ---------------------------------------------------------
    def training_rows(self) -> list[dict]:
        """Laya fine-tuning rows `{state, questions, expected}` from admin verdicts."""
        rows = []
        with self._lock:
            found = self._conn.execute(
                "SELECT observation, verdict, verdict_sev FROM incidents WHERE verdict IS NOT NULL ORDER BY id"
            ).fetchall()
        for r in found:
            obs = json.loads(r["observation"])
            state = {"report": describe(obs["kind"], obs["features"], obs["summary"])}
            expected = expected_from_verdict(r["verdict"], r["verdict_sev"])
            if expected:
                rows.append({"state": state, "questions": QUESTIONS, "expected": expected})
        return rows

    def agreement(self) -> dict:
        """How often each decider's action matched what the admin finally chose."""
        with self._lock:
            found = self._conn.execute(
                "SELECT rules, laya, verdict FROM incidents WHERE verdict IS NOT NULL").fetchall()
        n = len(found)
        out: dict[str, Any] = {"verdicts": n, "rules_match": 0, "laya_scored": 0, "laya_match": 0}
        for r in found:
            want = VERDICT_ACTION.get(r["verdict"])
            if json.loads(r["rules"])["action"] == want:
                out["rules_match"] += 1
            if r["laya"]:
                out["laya_scored"] += 1
                if json.loads(r["laya"])["action"] == want:
                    out["laya_match"] += 1
        out["rules_accuracy"] = round(out["rules_match"] / n, 3) if n else None
        out["laya_accuracy"] = round(out["laya_match"] / out["laya_scored"], 3) if out["laya_scored"] else None
        return out
