"""HTTP API + admin page. `create_app` takes injectable parts so tests need no camera."""

from __future__ import annotations

import asyncio
import json
import logging
import secrets
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Literal

from fastapi import FastAPI, HTTPException, Query, Request, WebSocket
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import FileResponse, HTMLResponse, Response, StreamingResponse
from pydantic import BaseModel, Field

from overwatch.camera import CvCameraSource, FrameSource, RemoteFrameSource
from overwatch.config import Settings
from overwatch.decision.laya_decider import LayaDecider
from overwatch.pipeline import Pipeline
from overwatch.responders import Responder, build_responder
from overwatch.store import IncidentStore
from overwatch.zones import Zone, ZoneSet

log = logging.getLogger(__name__)
STATIC = Path(__file__).parent / "static"


class WatchBody(BaseModel):
    seconds: float | None = Field(None, gt=0, le=3600)


class VerdictBody(BaseModel):
    verdict: Literal["approve_recovery", "escalate", "dismiss"]
    note: str = Field("", max_length=500)
    severity: int | None = Field(None, ge=0, le=3)


def create_app(settings: Settings | None = None, source: FrameSource | None = None,
               responder: Responder | None = None, laya: LayaDecider | None = None) -> FastAPI:
    s = settings or Settings.from_env()
    if source is not None:
        src = source
    elif s.source.strip().lower() == "remote":
        src = RemoteFrameSource()
    elif not s.source.strip():
        raise SystemExit(
            "No camera configured. Set OVERWATCH_SOURCE (or --source) to the camera's exact name, e.g. "
            "'name:HD camera'. List them with: uv run overwatch --list-cameras")
    else:
        if s.rotate not in (0, 90, 180, 270):
            raise SystemExit(f"OVERWATCH_ROTATE must be 0, 90, 180 or 270 (got {s.rotate})")
        src = CvCameraSource(s.source, s.width, s.height, s.loop_file, s.rotate)
    remote = src if isinstance(src, RemoteFrameSource) else None
    resp = responder or build_responder(s.responder, s.pi_url, s.pi_recover_traj)
    if laya is None and s.laya_mode != "off":
        laya = LayaDecider(s.laya_model, s.laya_path)
    store = IncidentStore(Path(s.data_dir) / "overwatch.db")
    zones = ZoneSet.load(Path(s.data_dir) / "zones.json")
    pipe = Pipeline(s, src, store, zones, resp, laya)

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        pipe.start()
        if laya is not None:  # load off the event loop; first run downloads the checkpoint
            asyncio.get_running_loop().run_in_executor(None, laya.load)
        yield
        pipe.stop()

    app = FastAPI(title="Overwatch", lifespan=lifespan)
    app.state.pipeline, app.state.store = pipe, store

    # -- remote camera agent ----------------------------------------------------
    @app.websocket("/api/ingest")
    async def ingest(ws: WebSocket):
        """camera-agent/ pushes one JPEG per binary message. Needs SOURCE=remote and
        `Authorization: Bearer <INGEST_TOKEN>`; unauthorised handshakes are refused before accept."""
        supplied = ws.headers.get("authorization", "").removeprefix("Bearer ").strip()
        if remote is None or not s.ingest_token:
            log.warning("Rejected camera ingest: set OVERWATCH_SOURCE=remote and OVERWATCH_INGEST_TOKEN")
            await ws.close(code=1008)
            return
        if not secrets.compare_digest(supplied.encode(), s.ingest_token.encode()):
            log.warning("Rejected camera ingest: wrong or missing token")
            await ws.close(code=1008)
            return
        await ws.accept()
        refusal = remote.attach(ws.client.host if ws.client else "unknown")
        if refusal:
            await ws.close(code=1013, reason=refusal)
            return
        log.info("Camera agent connected from %s", ws.client.host if ws.client else "unknown")
        reason = "agent closed the connection"
        try:
            while True:
                msg = await ws.receive()
                if msg["type"] == "websocket.disconnect":
                    break
                data = msg.get("bytes")
                if data and not await run_in_threadpool(remote.push, data):
                    reason = "monitor stopped"
                    await ws.close(code=1013, reason="Monitor is stopped")
                    break
        finally:
            remote.detach(reason)
            log.warning("Camera agent disconnected: %s", reason)

    # -- page / video ---------------------------------------------------------
    @app.get("/", response_class=HTMLResponse)
    def index():
        return (STATIC / "index.html").read_text()

    @app.get("/stream")
    async def stream(request: Request):
        async def gen():
            last = -1
            while not await request.is_disconnected():
                fid, jpeg = pipe.latest_frame_id(), pipe.latest_jpeg()
                if jpeg is not None and fid != last:
                    last = fid
                    yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                           + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                await asyncio.sleep(0.05)
        return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame",
                                 headers={"Cache-Control": "no-store"})

    @app.get("/api/frame.jpg")
    def frame():
        jpeg = pipe.latest_jpeg()
        if jpeg is None:
            raise HTTPException(503, "No frame yet")
        return Response(jpeg, media_type="image/jpeg", headers={"Cache-Control": "no-store"})

    # -- status / setup ---------------------------------------------------------
    @app.get("/api/status")
    def status():
        return pipe.status()

    @app.post("/api/watch")
    def start_watch(body: WatchBody | None = None):
        """Say "the robot is picking now" (for tests and demos; real robots signal it themselves).
        Starts a watch window; a fresh baseline picture is taken at once."""
        pipe.window.arm_manual(body.seconds if body else None)
        log.info("Watch window started from the dashboard")
        return pipe.window.state()

    @app.delete("/api/watch")
    def stop_watch():
        pipe.window.disarm()
        return pipe.window.state()

    @app.get("/api/zones")
    def get_zones():
        return [z.to_dict() for z in zones.zones]

    @app.put("/api/zones")
    def put_zones(body: list[dict]):
        try:
            new = [Zone.from_dict(d) for d in body]
        except (KeyError, ValueError, TypeError) as e:
            raise HTTPException(422, str(e)) from e
        zones.replace(new)
        pipe.save_zones()
        return [z.to_dict() for z in zones.zones]

    # -- incidents --------------------------------------------------------------
    @app.get("/api/incidents")
    def incidents(status: str | None = Query(None, pattern="^(open|filtered)$"), limit: int = Query(100, le=500)):
        return store.list(status, limit)

    @app.get("/api/incidents/{incident_id}")
    def incident(incident_id: int):
        inc = store.get(incident_id)
        if inc is None:
            raise HTTPException(404, "Unknown incident")
        return inc

    @app.get("/api/incidents/{incident_id}/snapshot")
    def snapshot(incident_id: int):
        inc = store.get(incident_id)
        if inc is None or not inc.get("snapshot"):
            raise HTTPException(404, "No snapshot")
        path = pipe.snap_dir / inc["snapshot"]
        if not path.is_file():
            raise HTTPException(404, "Snapshot file missing")
        return FileResponse(path, media_type="image/jpeg")

    @app.post("/api/incidents/{incident_id}/verdict")
    def verdict(incident_id: int, body: VerdictBody):
        inc = store.get(incident_id)
        if inc is None:
            raise HTTPException(404, "Unknown incident")
        if inc["status"] != "open":
            raise HTTPException(409, f"Incident is already {inc['status']}")
        result, status_after = None, {"approve_recovery": "actioned", "escalate": "escalated",
                                      "dismiss": "dismissed"}[body.verdict]
        if body.verdict == "approve_recovery":
            obs = inc["observation"]
            result = resp.recover({"incident": inc["code"], "bbox": obs.get("bbox"),
                                   "centroid": obs.get("centroid"), "table_xy": None})
            if not result.get("ok"):
                status_after = "open"   # robot could not act: stays open so the admin can retry or escalate
        out = store.set_verdict(incident_id, body.verdict, body.note, body.severity, status_after, result)
        log.info("%s verdict=%s -> %s", inc["code"], body.verdict, status_after)
        return out

    @app.post("/api/robot/hold")
    def hold():
        result = resp.hold()
        log.warning("Admin pressed HOLD: %s", result.get("detail"))
        return result

    # -- feedback loop -------------------------------------------------------------
    @app.get("/api/feedback/summary")
    def feedback_summary():
        return store.agreement()

    @app.get("/api/feedback.jsonl")
    def feedback_jsonl():
        body = "\n".join(json.dumps(r) for r in store.training_rows())
        return Response(body + ("\n" if body else ""), media_type="application/x-ndjson")

    return app
