from __future__ import annotations

import asyncio

from fastapi import APIRouter, Depends, HTTPException, Request
from fastapi.responses import StreamingResponse

from app.api.deps import Services, services
from app.models.schemas import CameraConfig, CameraStatus

router = APIRouter(prefix="/api/camera", tags=["camera"])


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
    s.repo.log("camera", f"Camera {s.camera.index} capture started", source="user")
    return s.camera.status()


@router.post("/stop", response_model=CameraStatus)
def camera_stop(s: Services = Depends(services)):
    s.camera.stop()
    s.repo.log("camera", f"Camera {s.camera.index} capture stopped and device released", source="user")
    return s.camera.status()


@router.put("/config", response_model=CameraStatus)
def camera_config(cfg: CameraConfig, s: Services = Depends(services)):
    s.camera.set_index(cfg.device_index)
    s.repo.log("camera", f"Camera device index set to {cfg.device_index}", source="user")
    return s.camera.status()
