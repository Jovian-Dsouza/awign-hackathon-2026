"""USB camera service (OpenCV).

One background thread owns the `cv2.VideoCapture`. It reads frames, JPEG-encodes
the latest one, and every HTTP client streams from that shared buffer, so the
device is opened exactly once no matter how many browsers are watching.

If the device can't be opened or stops delivering frames, the thread releases it
and retries with backoff; status reports the error. Nothing here ever fabricates
frames: no camera means no stream.
"""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import AsyncIterator
from datetime import UTC, datetime

import asyncio
import cv2

log = logging.getLogger(__name__)

_BACKENDS = {
    "any": cv2.CAP_ANY,
    "dshow": cv2.CAP_DSHOW,
    "msmf": cv2.CAP_MSMF,
    "v4l2": cv2.CAP_V4L2,
    "avfoundation": cv2.CAP_AVFOUNDATION,
}

MAX_CONSECUTIVE_READ_FAILURES = 15
RETRY_BACKOFF_S = (1.0, 2.0, 4.0, 8.0)
STALE_FRAME_S = 3.0


def _iso(ts: float) -> str:
    return datetime.fromtimestamp(ts, UTC).isoformat(timespec="milliseconds").replace("+00:00", "Z")


class CameraService:
    def __init__(self, index: int, backend: str = "any", width: int = 1280, height: int = 720,
                 fps: int = 15, jpeg_quality: int = 80, overlay: bool = True):
        self.index = index
        self.backend_name = backend if backend in _BACKENDS else "any"
        self.req_width, self.req_height, self.req_fps = width, height, fps
        self.jpeg_quality = jpeg_quality
        self.overlay = overlay

        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._shutdown = threading.Event()

        self._state = "stopped"
        self._error: str | None = None
        self._jpeg: bytes | None = None
        self._frame_id = 0
        self._last_frame_ts: float | None = None
        self._size: tuple[int, int] | None = None
        self._fps_measured: float | None = None
        self._clients = 0

    # ---------------- lifecycle ----------------
    def start(self) -> None:
        with self._lock:
            if self._thread and self._thread.is_alive():
                return
            self._stop.clear()
            self._state, self._error = "connecting", None
            self._thread = threading.Thread(target=self._run, name=f"camera-{self.index}", daemon=True)
            self._thread.start()

    def stop(self, timeout: float = 3.0) -> None:
        """Stop capturing and release the device (so other apps can use it)."""
        self._stop.set()
        thread = self._thread
        if thread and thread.is_alive():
            thread.join(timeout)
        with self._lock:
            self._thread = None
            self._state = "stopped"
            self._jpeg = None
            self._fps_measured = None

    def shutdown(self) -> None:
        self._shutdown.set()
        self.stop()

    def set_index(self, index: int) -> None:
        was_running = self._thread is not None and self._thread.is_alive()
        self.stop()
        self.index = index
        self._error = None
        if was_running:
            self.start()

    # ---------------- capture thread ----------------
    def _open(self) -> cv2.VideoCapture | None:
        cap = cv2.VideoCapture(self.index, _BACKENDS[self.backend_name])
        if not cap.isOpened():
            cap.release()
            return None
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.req_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.req_height)
        cap.set(cv2.CAP_PROP_FPS, self.req_fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def _run(self) -> None:
        attempt = 0
        while not self._stop.is_set():
            self._set_state("connecting")
            cap = self._open()
            if cap is None:
                delay = RETRY_BACKOFF_S[min(attempt, len(RETRY_BACKOFF_S) - 1)]
                self._set_state(
                    "error",
                    f"Could not open camera index {self.index} ({self.backend_name}). "
                    f"Check it is plugged in, not used by another app, and that camera permission is granted. "
                    f"Retrying in {delay:.0f}s.",
                )
                attempt += 1
                self._stop.wait(delay)
                continue
            log.info("Camera %d opened via %s", self.index, self.backend_name)
            attempt = 0
            try:
                self._capture_loop(cap)
            finally:
                cap.release()
                log.info("Camera %d released", self.index)
            if not self._stop.is_set():
                self._stop.wait(1.0)

    def _capture_loop(self, cap: cv2.VideoCapture) -> None:
        failures = 0
        min_interval = 1.0 / max(self.req_fps, 1)
        last_emit = 0.0
        fps_window: list[float] = []
        encode = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        while not self._stop.is_set():
            ok, frame = cap.read()
            if not ok or frame is None:
                failures += 1
                if failures >= MAX_CONSECUTIVE_READ_FAILURES:
                    self._set_state("error", f"Camera {self.index} stopped delivering frames (disconnected?). Reconnecting…")
                    return
                time.sleep(0.05)
                continue
            failures = 0
            now = time.time()
            if now - last_emit < min_interval:
                continue  # read() drains the driver buffer; only encode at target fps
            last_emit = now
            if self.overlay:
                self._draw_overlay(frame, now)
            ok, buf = cv2.imencode(".jpg", frame, encode)
            if not ok:
                continue
            fps_window = [t for t in fps_window if now - t < 2.0] + [now]
            with self._lock:
                self._jpeg = buf.tobytes()
                self._frame_id += 1
                self._last_frame_ts = now
                self._size = (frame.shape[1], frame.shape[0])
                self._fps_measured = round((len(fps_window) - 1) / max(now - fps_window[0], 1e-6), 1) \
                    if len(fps_window) > 1 else None
                self._state, self._error = "streaming", None

    def _draw_overlay(self, frame, now: float) -> None:
        text = f"UthaLe CAM{self.index}  {datetime.fromtimestamp(now).strftime('%Y-%m-%d %H:%M:%S')}"
        h = frame.shape[0]
        cv2.putText(frame, text, (12, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, text, (12, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (235, 235, 235), 1, cv2.LINE_AA)

    def _set_state(self, state: str, error: str | None = None) -> None:
        with self._lock:
            self._state, self._error = state, error
            if state != "streaming":
                self._jpeg = None
                self._fps_measured = None

    # ---------------- consumers ----------------
    def status(self) -> dict:
        with self._lock:
            state = self._state
            if state == "streaming" and self._last_frame_ts and time.time() - self._last_frame_ts > STALE_FRAME_S:
                state = "error"
            return {
                "state": state,
                "device_index": self.index,
                "backend": self.backend_name,
                "width": self._size[0] if self._size else None,
                "height": self._size[1] if self._size else None,
                "fps": self._fps_measured,
                "frames": self._frame_id,
                "last_frame_at": _iso(self._last_frame_ts) if self._last_frame_ts else None,
                "error": self._error if state != "streaming" else None,
                "clients": self._clients,
            }

    def latest_jpeg(self, max_age_s: float = STALE_FRAME_S) -> bytes | None:
        with self._lock:
            if self._jpeg and self._last_frame_ts and time.time() - self._last_frame_ts <= max_age_s:
                return self._jpeg
            return None

    @property
    def is_streaming(self) -> bool:
        return self.latest_jpeg() is not None

    async def mjpeg(self, is_disconnected) -> AsyncIterator[bytes]:
        """multipart/x-mixed-replace generator. Ends if the camera stops or the client leaves."""
        self._clients += 1
        last_id = -1
        waited = 0.0
        poll = 1.0 / max(self.req_fps * 2, 1)
        try:
            while not self._shutdown.is_set():
                if await is_disconnected():
                    break
                with self._lock:
                    frame_id, jpeg = self._frame_id, self._jpeg
                if jpeg is None:
                    waited += poll
                    if waited > STALE_FRAME_S:
                        break  # camera gone; client falls back to its offline state
                elif frame_id != last_id:
                    waited, last_id = 0.0, frame_id
                    yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                           + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                await asyncio.sleep(poll)
        finally:
            self._clients -= 1
