from __future__ import annotations

import asyncio
import logging
import secrets

from fastapi import APIRouter, Depends, HTTPException, Request, WebSocket
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import StreamingResponse

from app.api.deps import Services, services
from app.models.schemas import CameraConfig, CameraStatus

router = APIRouter(prefix="/api/camera", tags=["camera"])
log = logging.getLogger(__name__)


@router.get("/status", response_model=CameraStatus)
def camera_status(s: Services = Depends(services)):
    return s.camera.status()


@router.get("/stream")
async def camera_stream(request: Request, s: Services = Depends(services)):
    # Give a freshly (re)started camera a moment before refusing the client.
    for _ in range(20):
        if s.camera.is_streaming:
            break
        await asyncio.sleep(0.1)
    if not s.camera.is_streaming:
        status = s.camera.status()
        raise HTTPException(503, detail=status["error"] or f"Camera is {status['state']}")
    return StreamingResponse(
        s.camera.mjpeg(request.is_disconnected),
        media_type="multipart/x-mixed-replace; boundary=frame",
        headers={"Cache-Control": "no-store, no-cache, must-revalidate", "Pragma": "no-cache",
                 "X-Accel-Buffering": "no"},
    )


@router.post("/start", response_model=CameraStatus)
def camera_start(s: Services = Depends(services)):
    s.camera.start()
    what = "Remote camera ingest" if s.camera.is_remote else f"Camera {s.camera.index} capture"
    s.repo.log("camera", f"{what} started", source="user")
    return s.camera.status()


@router.post("/stop", response_model=CameraStatus)
def camera_stop(s: Services = Depends(services)):
    s.camera.stop()
    what = "Remote camera ingest stopped" if s.camera.is_remote else f"Camera {s.camera.index} capture stopped and device released"
    s.repo.log("camera", what, source="user")
    return s.camera.status()


@router.put("/config", response_model=CameraStatus)
def camera_config(cfg: CameraConfig, s: Services = Depends(services)):
    s.camera.set_index(cfg.device_index)
    s.repo.log("camera", f"Camera device index set to {cfg.device_index}", source="user")
    return s.camera.status()


@router.websocket("/ingest")
async def camera_ingest(ws: WebSocket, s: Services = Depends(services)):
    """Remote camera sender (pi-streamer/stream_camera.py). Each binary message is one JPEG.

    Requires CAMERA_SOURCE=remote and `Authorization: Bearer <CAMERA_INGEST_TOKEN>`.
    Unauthorised handshakes are refused before the WebSocket is accepted (HTTP 403).
    """
    cam = s.camera
    supplied = ws.headers.get("authorization", "").removeprefix("Bearer ").strip()
    if not cam.is_remote or not cam.ingest_token:
        log.warning("Rejected camera ingest: set CAMERA_SOURCE=remote and CAMERA_INGEST_TOKEN to enable it")
        await ws.close(code=1008)
        return
    if not secrets.compare_digest(supplied.encode(), cam.ingest_token.encode()):
        log.warning("Rejected camera ingest: wrong or missing token")
        await ws.close(code=1008)
        return

    await ws.accept()
    refusal = cam.attach_remote()
    if refusal:
        await ws.close(code=1013, reason=refusal)
        return
    s.repo.log("camera", "Remote camera connected")
    reason = "sender closed the connection"
    try:
        while True:
            msg = await ws.receive()
            if msg["type"] == "websocket.disconnect":
                break
            data = msg.get("bytes")
            if not data:
                continue  # text messages are ignored
            if not await run_in_threadpool(cam.push_frame, data):
                reason = "capture stopped on the dashboard"
                await ws.close(code=1013, reason="Camera capture is stopped on the dashboard")
                break
    finally:
        cam.detach_remote(reason)
        s.repo.log("camera", f"Remote camera disconnected: {reason}", severity="warning")
