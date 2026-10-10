"""Monitor (Overwatch) integration. Overwatch is replaced by a tiny stub server on a real
localhost socket, so the proxy, the stream passthrough and the WebSocket relay are exercised
for real."""

import asyncio
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest
from fastapi.testclient import TestClient
from starlette.websockets import WebSocketDisconnect

from app.api.deps import Services
from app.config import settings
from app.database import Database
from app.main import create_app
from app.repository import Repository
from app.services.camera import CameraService
from app.services.catalog import load_catalog
from app.services.robot import SimulatedRobotAdapter, SimulationScenario
from app.services.sensors import SimulatedSensorHub
from app.services.workflow import WorkflowEngine

TOKEN = "t0ken"


class _Overwatch(BaseHTTPRequestHandler):
    posts: list = []

    def log_message(self, *a):  # silence
        pass

    def _send(self, code, body, ctype="application/json"):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/api/status":
            self._send(200, json.dumps({"camera": {"state": "streaming"}, "reference_set": True}).encode())
        elif self.path == "/api/incidents?limit=5":
            self._send(200, b'[{"id": 1}]')
        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\nJPEGDATA\r\n")
        else:
            self._send(404, b'{"detail": "nope"}')

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        _Overwatch.posts.append((self.path, self.rfile.read(n)))
        self._send(200, b'{"ok": true}')


@pytest.fixture
def overwatch_stub(monkeypatch):
    srv = ThreadingHTTPServer(("127.0.0.1", 0), _Overwatch)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    monkeypatch.setattr(settings.__class__, "monitor_url", f"http://127.0.0.1:{srv.server_port}", raising=False)
    object.__setattr__(settings, "monitor_url", f"http://127.0.0.1:{srv.server_port}")
    object.__setattr__(settings, "monitor_ingest_token", TOKEN)
    _Overwatch.posts = []
    yield srv
    srv.shutdown()


@pytest.fixture
def client(tmp_path):
    repo = Repository(Database(tmp_path / "t.db"))
    catalog = load_catalog(settings.inventory_path)
    camera = CameraService(index=99)
    scenario = SimulationScenario(0.0, seed=1)
    robot = SimulatedRobotAdapter(scenario, speed=200)
    engine = WorkflowEngine(repo, catalog, robot, SimulatedSensorHub(scenario), camera, tmp_path / "ev")
    svc = Services(repo=repo, camera=camera, engine=engine, catalog=catalog)
    return TestClient(create_app(svc)), svc


def test_status_reports_unreachable_instead_of_faking(client, monkeypatch):
    c, _ = client
    object.__setattr__(settings, "monitor_url", "http://127.0.0.1:9")   # nothing listens here
    body = c.get("/api/monitor/status").json()
    assert body["reachable"] is False and "uv run overwatch" in body["error"]
    assert c.get("/api/monitor/stream").status_code == 503


def test_status_and_proxy_pass_through(client, overwatch_stub):
    c, _ = client
    body = c.get("/api/monitor/status").json()
    assert body["reachable"] is True and body["camera"]["state"] == "streaming"
    assert c.get("/api/monitor/incidents?limit=5").json() == [{"id": 1}]
    assert c.get("/api/monitor/incidents/7/snapshot").status_code == 404      # upstream's 404 passes through


def test_proxy_is_allow_listed(client, overwatch_stub):
    c, _ = client
    for bad in ("ingest", "../etc/passwd", "incidents/abc", "feedback.jsonl", "anything/else"):
        assert c.get(f"/api/monitor/{bad}").status_code == 404, bad


def test_stream_is_passed_through(client, overwatch_stub):
    c, _ = client
    with c.stream("GET", "/api/monitor/stream") as r:
        assert r.status_code == 200 and r.headers["content-type"].startswith("multipart/x-mixed-replace")
        assert b"JPEGDATA" in r.read()


def test_webhook_needs_token_and_lands_in_activity_feed(client):
    c, svc = client
    object.__setattr__(settings, "monitor_ingest_token", TOKEN)
    inc = {"incident": {"code": "INC-0007", "observation": {"kind": "foreign_object", "zone": "floor",
           "summary": "New object"}, "final": {"action": "propose_recovery", "severity": 1}}}
    assert c.post("/api/monitor/webhook", json=inc).status_code == 403
    assert c.post("/api/monitor/webhook", json=inc, headers={"Authorization": "Bearer wrong"}).status_code == 403
    assert c.post("/api/monitor/webhook", json={"nope": 1}, headers={"Authorization": f"Bearer {TOKEN}"}).status_code == 422
    assert c.post("/api/monitor/webhook", json=inc, headers={"Authorization": f"Bearer {TOKEN}"}).json() == {"ok": True}
    row = svc.repo.list_activity(1)[0]
    assert row["type"] == "monitor" and "INC-0007" in row["message"] and row["severity"] == "warning"
    assert row["source"] == "hardware"


def test_webhook_refused_when_no_token_configured(client):
    c, _ = client
    object.__setattr__(settings, "monitor_ingest_token", "")
    r = c.post("/api/monitor/webhook", json={"incident": {}}, headers={"Authorization": "Bearer "})
    assert r.status_code == 403


def test_relay_refuses_bad_token(client, overwatch_stub):
    c, _ = client
    for headers in ({}, {"Authorization": "Bearer nope"}):
        with pytest.raises(WebSocketDisconnect):
            with c.websocket_connect("/api/monitor/ingest", headers=headers):
                pass


def test_relay_closes_cleanly_when_monitor_is_down(client):
    c, _ = client
    object.__setattr__(settings, "monitor_ingest_token", TOKEN)
    object.__setattr__(settings, "monitor_url", "http://127.0.0.1:9")
    with c.websocket_connect("/api/monitor/ingest", headers={"Authorization": f"Bearer {TOKEN}"}) as ws:
        with pytest.raises(WebSocketDisconnect) as e:
            ws.receive_bytes()
    assert e.value.code == 1013
