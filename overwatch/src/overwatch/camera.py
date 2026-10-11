"""Frame sources: a USB camera (found by name, because indices shift), an RTSP/HTTP
stream, or a video file (existing CCTV footage, offline replay).

One thread owns the capture and keeps only the newest frame. If the source dies it
reconnects with backoff. Nothing here ever invents a frame: no camera, no frames.
"""

from __future__ import annotations

import logging
import os
import re
import select
import shutil
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import cv2
import numpy as np

log = logging.getLogger(__name__)

RETRY_BACKOFF_S = (1.0, 2.0, 4.0, 8.0)
MAX_READ_FAILURES = 15
STALE_S = 3.0


_ROTATIONS = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}


def apply_rotation(img: np.ndarray, degrees: int) -> np.ndarray:
    """Rotate clockwise by 0/90/180/270. Done at capture so the reference, detectors, stream and
    snapshots all see an upright image."""
    if degrees not in (0, *_ROTATIONS):
        raise ValueError(f"rotate must be 0, 90, 180 or 270 (got {degrees})")
    return img if degrees == 0 else cv2.rotate(img, _ROTATIONS[degrees])


@dataclass
class Frame:
    image: np.ndarray   # BGR
    ts: float
    id: int


class FrameSource(Protocol):
    def start(self) -> None: ...
    def stop(self) -> None: ...
    def latest(self) -> Frame | None: ...
    def status(self) -> dict: ...


# ---------------------------------------------------------------------------
# Device discovery
# ---------------------------------------------------------------------------

def list_video_devices() -> list[tuple[int, str]]:
    """(index, name) of local video devices, in the order OpenCV numbers them."""
    if sys.platform == "darwin":
        return _list_avfoundation()
    if sys.platform.startswith("linux"):
        return _list_v4l2()
    return []


def _list_avfoundation() -> list[tuple[int, str]]:
    try:
        out = subprocess.run(
            ["ffmpeg", "-hide_banner", "-f", "avfoundation", "-list_devices", "true", "-i", ""],
            capture_output=True, text=True, timeout=10,
        ).stderr
    except (OSError, subprocess.SubprocessError):
        return []
    devices: list[tuple[int, str]] = []
    in_video = False
    for line in out.splitlines():
        if "AVFoundation video devices" in line:
            in_video = True
        elif "AVFoundation audio devices" in line:
            break
        elif in_video:
            m = re.search(r"\[(\d+)\]\s+(.+)$", line)
            if m and not m.group(2).startswith("Capture screen"):
                devices.append((int(m.group(1)), m.group(2).strip()))
    return devices


def _list_v4l2() -> list[tuple[int, str]]:
    devices = []
    for p in sorted(Path("/sys/class/video4linux").glob("video*"), key=lambda q: int(q.name[5:] or 0)):
        try:
            # Each USB camera also exposes a metadata node; capture nodes have index 0.
            if (p / "index").read_text().strip() != "0":
                continue
            devices.append((int(p.name[5:]), (p / "name").read_text().strip()))
        except (OSError, ValueError):
            continue
    return devices


def resolve_source(spec: str) -> tuple[object, str]:
    """Turn a source spec into something that can be opened, plus a label.

    `name:` is exact (macOS: captured through ffmpeg by device name; Linux: /dev/videoN via
    sysfs). A bare index is allowed but is not stable on macOS and can be the built-in camera.
    """
    spec = spec.strip()
    if spec.startswith("name:"):
        want = spec[5:].strip().lower()
        devices = list_video_devices()
        matches = sorted({n for _, n in devices if want in n.lower()})
        if not matches:
            raise LookupError(f"No video device matching {spec[5:]!r}. Found: {[n for _, n in devices] or 'none'}")
        exact = next((n for n in matches if n.lower() == want), None)
        if exact is None:
            if len(matches) > 1:
                raise LookupError(f"{spec[5:]!r} matches several cameras {matches}. Use the full name.")
            exact = matches[0]
        if sys.platform == "darwin":
            if not shutil.which("ffmpeg"):
                raise LookupError("Selecting a camera by name on macOS needs ffmpeg: brew install ffmpeg")
            return FfmpegName(exact), f"{exact} (selected by exact name)"
        idx = next(i for i, n in devices if n == exact)
        return f"/dev/video{idx}", f"{exact} (/dev/video{idx})"
    if spec.startswith("index:") or spec.isdigit():
        n = int(spec.split(":")[-1])
        if sys.platform == "darwin":
            log.warning("camera index %d: macOS numbering is not stable between runs and can be the "
                        "built-in camera. Prefer source name:<device name>.", n)
        return n, f"camera index {n}"
    return spec, spec



# ---------------------------------------------------------------- capture by exact device name
class FfmpegName(str):
    """A camera selected by its exact device name and captured through ffmpeg.

    Why not an OpenCV index? On macOS the device numbering differs between ffmpeg and
    OpenCV and changes between runs (built-in and external cameras swap places), so
    "look up the index by name, then open it" can silently open the wrong camera,
    e.g. the laptop's own. A name is the only exact selector.
    """

def split_jpegs(buf: bytearray) -> list[bytes]:
    """Remove and return every complete JPEG (SOI..EOI) at the front of `buf`."""
    out: list[bytes] = []
    while True:
        s = buf.find(b"\xff\xd8")
        if s < 0:
            del buf[:-1]               # keep a trailing 0xFF: it may be half a marker
            return out
        e = buf.find(b"\xff\xd9", s + 2)
        if e < 0:
            if s:
                del buf[:s]
            return out
        out.append(bytes(buf[s:e + 2]))
        del buf[:e + 2]

class FfmpegCapture:
    """cv2.VideoCapture-shaped reader (isOpened/read/release) over an ffmpeg avfoundation pipe."""

    def __init__(self, name: str, width: int = 0, height: int = 0):
        self._buf = bytearray()
        self._proc: subprocess.Popen | None = None
        self._err = ""
        sizes = ([f"{width}x{height}"] if width and height else []) + [None]
        for size in sizes:             # a size the device does not offer makes ffmpeg exit: fall back
            self._start(name, size)
            if self._wait_first(6.0):
                return
            self._kill()
        raise OSError(f"ffmpeg could not capture from {name!r}: {self._err or 'no frames'}")

    def _start(self, name: str, size: str | None) -> None:
        cmd = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-f", "avfoundation", "-framerate", "30"]
        if size:
            cmd += ["-video_size", size]
        cmd += ["-i", name, "-an", "-c:v", "mjpeg", "-q:v", "3", "-f", "image2pipe", "-"]
        self._buf.clear()
        self._proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    def _pull(self, timeout: float) -> bytes | None:
        """Bytes, b"" if nothing yet, None at EOF."""
        assert self._proc and self._proc.stdout
        r, _, _ = select.select([self._proc.stdout], [], [], timeout)
        if not r:
            return b""
        chunk = os.read(self._proc.stdout.fileno(), 1 << 16)
        return chunk if chunk else None

    def _wait_first(self, timeout: float) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            chunk = self._pull(0.25)
            if chunk is None:
                if self._proc and self._proc.stderr:
                    try:
                        self._err = self._proc.stderr.read().decode("utf-8", "replace").strip().splitlines()[-1]
                    except (IndexError, OSError):
                        pass
                return False
            self._buf += chunk
            if b"\xff\xd9" in self._buf:
                return True
        return False

    def _kill(self) -> None:
        if self._proc and self._proc.poll() is None:
            self._proc.kill()
        if self._proc:
            try:
                self._proc.wait(2)
            except subprocess.TimeoutExpired:
                pass

    def isOpened(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def read(self):
        end = time.time() + 3.0
        while time.time() < end:
            jpegs = split_jpegs(self._buf)
            if jpegs:                  # newest only: anything older is already stale
                img = cv2.imdecode(np.frombuffer(jpegs[-1], np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    return True, img
                continue
            chunk = self._pull(0.5)
            if chunk is None:
                return False, None
            self._buf += chunk
        return False, None

    def release(self) -> None:
        self._kill()

    def set(self, *_a) -> bool:        # keep cv2.VideoCapture's interface
        return False

    def get(self, *_a) -> float:
        return 0.0


# ---------------------------------------------------------------------------
# Capture
# ---------------------------------------------------------------------------

class CvCameraSource:
    def __init__(self, spec: str, width: int = 0, height: int = 0, loop_file: bool = True, rotate: int = 0):
        apply_rotation(np.zeros((1, 1, 3), np.uint8), rotate)   # validate early
        self.spec = spec
        self.width, self.height = width, height
        self.loop_file = loop_file
        self.rotate = rotate
        self.label = spec

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._frame: Frame | None = None
        self._frame_id = 0
        self._state = "stopped"
        self._error: str | None = None
        self._size: tuple[int, int] | None = None
        self._times: list[float] = []

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="overwatch-camera", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(3.0)
        with self._lock:
            self._state, self._frame = "stopped", None

    def latest(self) -> Frame | None:
        with self._lock:
            f = self._frame
        if f is None or time.time() - f.ts > STALE_S:
            return None
        return f

    def status(self) -> dict:
        with self._lock:
            f = self._frame
            fps = None
            if len(self._times) > 1:
                fps = round((len(self._times) - 1) / max(self._times[-1] - self._times[0], 1e-6), 1)
            state = self._state
            if state == "streaming" and f and time.time() - f.ts > STALE_S:
                state = "error"
            return {
                "state": state, "source": self.label, "error": self._error if state != "streaming" else None,
                "width": self._size[0] if self._size else None,
                "height": self._size[1] if self._size else None,
                "fps": fps, "frames": self._frame_id,
                "last_frame_at": f.ts if f else None,
            }

    # -- thread --
    def _set(self, state: str, error: str | None = None) -> None:
        with self._lock:
            self._state, self._error = state, error
            if state != "streaming":
                self._frame = None
                self._times.clear()

    def _open(self):
        target, self.label = resolve_source(self.spec)
        if isinstance(target, FfmpegName):
            cap = FfmpegCapture(str(target), self.width, self.height)
            log.info("Opened %s", self.label)
            return cap, False
        backend = cv2.CAP_AVFOUNDATION if (isinstance(target, int) and sys.platform == "darwin") else cv2.CAP_ANY
        cap = cv2.VideoCapture(target, backend)
        if not cap.isOpened():
            cap.release()
            raise OSError(f"could not open {self.label}")
        if isinstance(target, int):
            if self.width:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            if self.height:
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap, isinstance(target, str) and os.path.isfile(target)

    def _run(self) -> None:
        attempt = 0
        while not self._stop.is_set():
            self._set("connecting")
            try:
                cap, is_file = self._open()
            except (OSError, LookupError) as e:
                delay = RETRY_BACKOFF_S[min(attempt, len(RETRY_BACKOFF_S) - 1)]
                self._set("error", f"{e}. Check the cable, that no other app holds the camera, and macOS camera "
                                   f"permission for this terminal. Retrying in {delay:.0f}s.")
                attempt += 1
                self._stop.wait(delay)
                continue
            attempt = 0
            try:
                self._capture(cap, is_file)
            finally:
                cap.release()
            if not self._stop.is_set():
                self._stop.wait(1.0)

    def _capture(self, cap, is_file: bool) -> None:
        failures = 0
        file_fps = (cap.get(cv2.CAP_PROP_FPS) or 25.0) if is_file else 0.0
        while not self._stop.is_set():
            t0 = time.time()
            ok, img = cap.read()
            if not ok or img is None:
                if is_file and self.loop_file:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                failures += 1
                if is_file or failures >= MAX_READ_FAILURES:
                    self._set("error", "Source stopped delivering frames (unplugged / stream ended). Reconnecting…")
                    return
                time.sleep(0.05)
                continue
            failures = 0
            now = time.time()
            img = apply_rotation(img, self.rotate)
            with self._lock:
                self._frame_id += 1
                self._frame = Frame(img, now, self._frame_id)
                self._size = (img.shape[1], img.shape[0])
                self._times = [t for t in self._times if now - t < 2.0] + [now]
                self._state, self._error = "streaming", None
            if is_file:  # pace file playback at its native rate
                self._stop.wait(max(0.0, 1.0 / file_fps - (time.time() - t0)))


class RemoteFrameSource:
    """Frames pushed by a camera agent (camera-agent/) over a WebSocket.

    The agent can be on any machine, behind any network, reaching this service through
    an ngrok tunnel: it connects *out*, so no port forwarding is needed. One sender at a
    time. Every frame is decoded, so only real JPEG images ever reach the detectors.
    """

    MAX_BYTES = 4 * 1024 * 1024

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._enabled = False
        self._attached = False
        self._sender: str | None = None
        self._frame: Frame | None = None
        self._frame_id = 0
        self._size: tuple[int, int] | None = None
        self._times: list[float] = []
        self._error: str | None = None
        self._state = "stopped"

    # lifecycle (FrameSource)
    def start(self) -> None:
        with self._lock:
            self._enabled = True
            if not self._attached:
                self._state = "waiting"

    def stop(self) -> None:
        with self._lock:
            self._enabled = False
            self._frame, self._state, self._times = None, "stopped", []

    # sender side
    def attach(self, sender: str) -> str | None:
        """Claim the single sender slot. Returns a refusal reason, or None if attached."""
        with self._lock:
            if not self._enabled:
                return "Monitor is stopped"
            if self._attached:
                return "Another camera agent is already connected"
            self._attached, self._sender, self._state, self._error = True, sender, "connecting", None
            return None

    def detach(self, reason: str) -> None:
        with self._lock:
            self._attached, self._sender, self._frame, self._times = False, None, None, []
            if self._enabled:
                self._state, self._error = "waiting", f"Camera agent disconnected ({reason}). Waiting for it to reconnect…"
            else:
                self._state, self._error = "stopped", None

    def push(self, jpeg: bytes) -> bool:
        """Accept one JPEG. False means the sender should disconnect."""
        if not self._enabled:
            return False
        if len(jpeg) > self.MAX_BYTES:
            return True                       # drop an oversized frame, keep the connection
        img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return True                       # not a decodable JPEG; drop it
        now = time.time()
        with self._lock:
            self._frame_id += 1
            self._frame = Frame(img, now, self._frame_id)
            self._size = (img.shape[1], img.shape[0])
            self._times = [t for t in self._times if now - t < 2.0] + [now]
            self._state, self._error = "streaming", None
        return True

    # consumer side (FrameSource)
    def latest(self) -> Frame | None:
        with self._lock:
            f = self._frame
        if f is None or time.time() - f.ts > STALE_S:
            return None
        return f

    def status(self) -> dict:
        with self._lock:
            f, state = self._frame, self._state
            if state == "streaming" and f and time.time() - f.ts > STALE_S:
                state = "error"
            fps = None
            if len(self._times) > 1:
                fps = round((len(self._times) - 1) / max(self._times[-1] - self._times[0], 1e-6), 1)
            return {
                "state": state, "source": f"remote agent ({self._sender})" if self._sender else "remote agent",
                "error": (self._error if state != "streaming" else None)
                         or ("No camera agent connected yet. Start camera-agent with this server's URL and token."
                             if state == "waiting" else None),
                "width": self._size[0] if self._size else None,
                "height": self._size[1] if self._size else None,
                "fps": fps, "frames": self._frame_id, "last_frame_at": f.ts if f else None,
                "agent_connected": self._attached,
            }
