"""External monitor camera (the Overwatch service): the dashboard's single door to it.

Overwatch runs as its own service (awign-hackathon-2026/overwatch). This module does
three things so the browser and a remote camera agent only ever need ONE address (the
same ngrok tunnel the dashboard already uses):

  * proxy   GET  /api/monitor/stream        live annotated MJPEG
            *    /api/monitor/<allow-listed> status, incidents, snapshots, verdicts, reference...
  * relay   WS   /api/monitor/ingest        a camera-agent pushes frames here; they are
                                            forwarded to Overwatch's /api/ingest
  * inbox   POST /api/monitor/webhook       Overwatch reports each new incident; it lands in
                                            the activity feed

Nothing is faked: if Overwatch is not running the status says so and the stream is a 503.
The robot is never commanded from here except through Overwatch's explicit admin actions.
"""

from __future__ import annotations

import json
import logging
import re
import secrets
import urllib.error
import urllib.request

from fastapi import APIRouter, Depends, HTTPException, Request, Response, WebSocket
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import StreamingResponse

from app.api.deps import Services, services
from app.config import settings

router = APIRouter(prefix="/api/monitor", tags=["monitor"])
log = logging.getLogger(__name__)

# Only these Overwatch paths can be reached through the proxy.
ALLOWED = re.compile(
    r"^(status|reference|zones|robot/hold|feedback/summary|incidents(/\d+(/snapshot|/verdict)?)?)$"
)
UPSTREAM_ERRORS = (urllib.error.URLError, OSError, TimeoutError)


def _base() -> str:
    return settings.monitor_url.rstrip("/")


def _fetch(method: str, url: str, body: bytes | None, timeout: float = 6.0) -> tuple[int, bytes, str]:
    headers = {"Content-Type": "application/json"} if body is not None else {}
    req = urllib.request.Request(url, data=body, method=method, headers=headers)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, r.read(), r.headers.get("Content-Type", "application/json")
    except urllib.error.HTTPError as e:      # upstream answered with an error: pass it through
        return e.code, e.read(), e.headers.get("Content-Type", "application/json")


def _unreachable(e: Exception) -> str:
    return (f"Monitor service not reachable at {_base()} ({e}). "
            "Start it with `uv run overwatch` in awign-hackathon-2026/overwatch.")


def _authorised(supplied: str) -> bool:
    token = settings.monitor_ingest_token
    return bool(token) and secrets.compare_digest(supplied.encode(), token.encode())


def _bearer(headers) -> str:
    return headers.get("authorization", "").removeprefix("Bearer ").strip()


# ---------------------------------------------------------------- status / stream
@router.get("/status")
def monitor_status():
    """Always 200. `reachable: false` is a normal state the dashboard renders, not an error."""
    try:
        code, data, _ = _fetch("GET", f"{_base()}/api/status", None, timeout=3.0)
    except UPSTREAM_ERRORS as e:
        return {"reachable": False, "error": _unreachable(e)}
    if code != 200:
        return {"reachable": False, "error": f"Monitor service answered HTTP {code}"}
    return {"reachable": True, **json.loads(data)}


@router.get("/stream")
async def monitor_stream():
    """Annotated MJPEG from Overwatch. The upstream connection is opened before answering so a
    dead monitor is a clean 503 instead of an empty 200 that leaves the <img> hanging."""
    try:
        upstream = await run_in_threadpool(urllib.request.urlopen, f"{_base()}/stream", None, 8.0)
    except UPSTREAM_ERRORS as e:
        raise HTTPException(503, _unreachable(e)) from e

    def chunks():
        try:
            while True:
                data = upstream.read(16384)
                if not data:
                    break
                yield data
        except (OSError, TimeoutError):
            pass
        finally:
            upstream.close()

    return StreamingResponse(
        chunks(), media_type="multipart/x-mixed-replace; boundary=frame",
        headers={"Cache-Control": "no-store, no-cache, must-revalidate", "X-Accel-Buffering": "no"},
    )


# ---------------------------------------------------------------- camera agent relay
@router.websocket("/ingest")
async def monitor_ingest(ws: WebSocket):
    """camera-agent --via-uthale connects here. Frames are forwarded unchanged to Overwatch.

    Same contract as Overwatch's own /api/ingest: Bearer MONITOR_INGEST_TOKEN, refused
    before accept otherwise. The one shared token is also what Overwatch must have as
    OVERWATCH_INGEST_TOKEN."""
    if not _authorised(_bearer(ws.headers)):
        log.warning("Rejected monitor ingest: wrong or missing token (or MONITOR_INGEST_TOKEN unset)")
        await ws.close(code=1008)
        return
    from websockets.asyncio.client import connect
    from websockets.exceptions import WebSocketException

    await ws.accept()
    url = re.sub(r"^http", "ws", _base(), count=1) + "/api/ingest"
    reason = "agent closed the connection"
    try:
        async with connect(url, additional_headers={"Authorization": f"Bearer {settings.monitor_ingest_token}"},
                           open_timeout=5, compression=None, max_size=2**20) as upstream:
            log.info("Monitor relay open: agent -> %s", url)
            while True:
                msg = await ws.receive()
                if msg["type"] == "websocket.disconnect":
                    break
                data = msg.get("bytes")
                if data:
                    await upstream.send(data)
    except (WebSocketException, OSError, TimeoutError) as e:
        reason = f"monitor service unavailable: {e}"
        log.warning("Monitor relay ended: %s", reason)
        try:
            await ws.close(code=1013, reason=reason[:120])
        except RuntimeError:
            pass


# ---------------------------------------------------------------- incident inbox
@router.post("/webhook")
async def monitor_webhook(request: Request, s: Services = Depends(services)):
    """Overwatch -> UthaLe: one call per new incident. Becomes an activity-feed entry."""
    if not _authorised(_bearer(request.headers)):
        raise HTTPException(403, "Bad or missing token")
    try:
        inc = (await request.json()).get("incident") or {}
        obs, final = inc["observation"], inc["final"]
    except (ValueError, KeyError, AttributeError) as e:
        raise HTTPException(422, "Expected {'incident': {observation, final, ...}}") from e
    severity = "error" if final.get("severity", 0) >= 3 else "warning" if final.get("severity", 0) >= 1 else "info"
    s.repo.log("monitor", f"{inc.get('code', 'INCIDENT')} · {obs.get('kind', '?').replace('_', ' ')} "
                          f"({obs.get('zone', '?')}): {obs.get('summary', '')} → {final.get('action', '?')}",
               severity=severity, source="hardware")
    return {"ok": True}


# ---------------------------------------------------------------- allow-listed proxy (keep last)
@router.api_route("/{path:path}", methods=["GET", "POST", "PUT", "DELETE"])
async def monitor_proxy(path: str, request: Request):
    if not ALLOWED.match(path):
        raise HTTPException(404, "Not a monitor endpoint")
    body = await request.body() or None
    url = f"{_base()}/api/{path}" + (f"?{request.url.query}" if request.url.query else "")
    try:
        code, data, ctype = await run_in_threadpool(_fetch, request.method, url, body)
    except UPSTREAM_ERRORS as e:
        raise HTTPException(502, _unreachable(e)) from e
    return Response(data, status_code=code, media_type=ctype.split(";")[0])
