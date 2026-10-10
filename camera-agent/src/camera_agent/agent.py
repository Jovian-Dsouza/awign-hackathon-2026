"""camera-agent: capture a camera and push JPEG frames to a monitoring server.

Runs anywhere (laptop, mini PC, Raspberry Pi) with only OpenCV and `websockets`. It
connects OUT to the server, so the device can sit behind any NAT; to reach a server on
another network, point --server at that server's ngrok URL.

    camera-agent --server https://abc.ngrok-free.app --token <token>

Targets (--path):
    /api/ingest           Overwatch directly              (default)
    /api/monitor/ingest   UthaLe backend, which relays to Overwatch  (--via uthale)
"""

from __future__ import annotations

import argparse
import logging
import os
import re
import select
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from urllib.parse import urlsplit, urlunsplit

import cv2
import numpy as np
from websockets.exceptions import ConnectionClosed, InvalidHandshake, InvalidStatus, InvalidURI
from websockets.sync.client import connect

log = logging.getLogger("camera-agent")
stop = threading.Event()

BACKOFF_S = (1, 2, 4, 8, 10)
STATS_EVERY_S = 10.0


# ---------------------------------------------------------------- device discovery
def list_devices() -> list[tuple[int, str]]:
    if sys.platform == "darwin":
        try:
            err = subprocess.run(["ffmpeg", "-hide_banner", "-f", "avfoundation", "-list_devices", "true", "-i", ""],
                                 capture_output=True, text=True, timeout=10).stderr
        except (OSError, subprocess.SubprocessError):
            return []
        out, in_video = [], False
        for line in err.splitlines():
            if "AVFoundation video devices" in line:
                in_video = True
            elif "AVFoundation audio devices" in line:
                break
            elif in_video:
                m = re.search(r"\[(\d+)\]\s+(.+)$", line)
                if m and not m.group(2).startswith("Capture screen"):
                    out.append((int(m.group(1)), m.group(2).strip()))
        return out
    if sys.platform.startswith("linux"):
        out = []
        for p in sorted(Path("/sys/class/video4linux").glob("video*"), key=lambda q: int(q.name[5:] or 0)):
            try:
                if (p / "index").read_text().strip() == "0":
                    out.append((int(p.name[5:]), (p / "name").read_text().strip()))
            except (OSError, ValueError):
                pass
        return out
    return []


def resolve(spec: str) -> tuple[object, str]:
    """"name:USB 2.0 Camera" | "index:1" | "1" | rtsp://… | /path/video.mp4

    `name:` is the safe selector: it is exact on macOS (captured via ffmpeg by name) and on
    Linux (/dev/videoN found through sysfs). A bare index is accepted but is not stable
    across runs on macOS.
    """
    spec = spec.strip()
    if spec.startswith("name:"):
        want = spec[5:].strip().lower()
        devices = list_devices()
        matches = [n for _, n in devices if want in n.lower()]
        if not matches:
            raise SystemExit(f"No camera matching {spec[5:]!r}. Found: {[n for _, n in devices] or 'none'}")
        exact = next((n for n in matches if n.lower() == want), matches[0])
        if len(set(matches)) > 1 and exact.lower() != want:
            raise SystemExit(f"{spec[5:]!r} matches several cameras {sorted(set(matches))}. Use the full name.")
        if sys.platform == "darwin":
            if not shutil.which("ffmpeg"):
                raise SystemExit("Selecting a camera by name on macOS needs ffmpeg: brew install ffmpeg")
            return FfmpegName(exact), f"{exact} (selected by exact name)"
        idx = next(i for i, n in devices if n == exact)
        return f"/dev/video{idx}", f"{exact} (/dev/video{idx})"
    if spec.startswith("index:") or spec.isdigit():
        n = int(spec.split(":")[-1])
        if sys.platform == "darwin":
            log.warning("camera index %d: macOS numbering is not stable between runs and can be the "
                        "built-in camera. Prefer --source name:<device name>.", n)
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


# ---------------------------------------------------------------- capture
class Grabber:
    """Reads the camera as fast as it delivers and keeps only the newest frame, so a slow
    network never makes the server look at old footage."""

    def __init__(self, target, width: int, height: int, loop_file: bool = True):
        if isinstance(target, FfmpegName):
            self.cap = FfmpegCapture(str(target), width, height)
        else:
            backend = cv2.CAP_AVFOUNDATION if (isinstance(target, int) and sys.platform == "darwin") else cv2.CAP_ANY
            self.cap = cv2.VideoCapture(target, backend)
        if not self.cap.isOpened():
            raise OSError(f"could not open {target!r}. Is it plugged in, not used by another app, "
                          "and does this terminal have camera permission?")
        if isinstance(target, int):
            if width:
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            if height:
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.is_file = isinstance(target, str) and os.path.isfile(target)
        self.loop_file = loop_file
        self._file_fps = (self.cap.get(cv2.CAP_PROP_FPS) or 25.0) if self.is_file else 0.0
        self._lock = threading.Lock()
        self._frame = None
        self._seq = 0
        self._dead = False
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def _run(self) -> None:
        fails = 0
        while not stop.is_set() and not self._dead:
            t0 = time.time()
            ok, img = self.cap.read()
            if not ok or img is None:
                if self.is_file and self.loop_file:
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                fails += 1
                if fails >= 15 or self.is_file:
                    self._dead = True
                    return
                time.sleep(0.05)
                continue
            fails = 0
            with self._lock:
                self._frame, self._seq = img, self._seq + 1
            if self.is_file:
                time.sleep(max(0.0, 1.0 / self._file_fps - (time.time() - t0)))

    @property
    def dead(self) -> bool:
        return self._dead

    def latest(self, after_seq: int):
        deadline = time.time() + 2.0
        while time.time() < deadline and not stop.is_set():
            with self._lock:
                if self._seq != after_seq and self._frame is not None:
                    return self._frame.copy(), self._seq
            if self._dead:
                return None, after_seq
            time.sleep(0.005)
        return None, after_seq

    def close(self) -> None:
        self._dead = True
        self._t.join(1.0)
        self.cap.release()


def encode(frame, max_width: int, quality: int, rotate: int):
    if rotate:
        frame = cv2.rotate(frame, {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
                                   270: cv2.ROTATE_90_COUNTERCLOCKWISE}[rotate])
    h, w = frame.shape[:2]
    if max_width and w > max_width:
        frame = cv2.resize(frame, (max_width, int(h * max_width / w)), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    return buf.tobytes() if ok else None, frame.shape[1], frame.shape[0]


# ---------------------------------------------------------------- network
def ws_url(server: str, path: str) -> str:
    """https://abc.ngrok-free.app -> wss://abc.ngrok-free.app/api/ingest"""
    if "://" not in server:
        server = "https://" + server
    parts = urlsplit(server)
    scheme = {"https": "wss", "http": "ws", "wss": "wss", "ws": "ws"}.get(parts.scheme)
    if not scheme or not parts.netloc:
        raise SystemExit(f"Invalid --server URL: {server!r}")
    return urlunsplit((scheme, parts.netloc, path, "", ""))


def stream(ws, grab: Grabber, a) -> None:
    interval = 1.0 / max(a.fps, 0.5)
    seq, sent, sent_bytes, t_stats = 0, 0, 0, time.monotonic()
    while not stop.is_set():
        t0 = time.monotonic()
        frame, seq = grab.latest(seq)
        if frame is None:
            if grab.dead:
                raise OSError("camera stopped delivering frames (unplugged?)")
            continue
        data, w, h = encode(frame, a.max_width, a.quality, a.rotate)
        if data is None:
            continue
        ws.send(data)
        sent += 1
        sent_bytes += len(data)
        now = time.monotonic()
        if now - t_stats >= STATS_EVERY_S:
            secs = now - t_stats
            log.info("sending %dx%d at %.1f fps, %.0f kB/s (%.1f MB/hour)", w, h, sent / secs,
                     sent_bytes / secs / 1024, sent_bytes / secs * 3600 / 1e6)
            sent, sent_bytes, t_stats = 0, 0, now
        stop.wait(max(0.0, interval - (time.monotonic() - t0)))


def run(a) -> int:
    target, label = resolve(a.source)
    url = ws_url(a.server, a.path)
    headers = {"Authorization": f"Bearer {a.token}"}
    log.info("camera: %s  ->  %s", label, url)
    attempt = 0
    grab: Grabber | None = None
    while not stop.is_set():
        try:
            if grab is None or grab.dead:
                if grab:
                    grab.close()
                grab = Grabber(target, a.width, a.height, loop_file=True)
                log.info("camera open")
            with connect(url, additional_headers=headers, open_timeout=15, compression=None,
                         max_size=2**20, ping_interval=20, ping_timeout=20) as ws:
                log.info("connected; streaming")
                attempt = 0
                stream(ws, grab, a)
        except InvalidStatus as e:
            code = e.response.status_code
            hint = {403: "Token mismatch, or the server is not in remote mode "
                         "(OVERWATCH_SOURCE=remote and OVERWATCH_INGEST_TOKEN set).",
                    404: "Wrong URL/path, or the tunnel points at the wrong port. Try --path / --via uthale."
                    }.get(code, "")
            log.error("server refused the connection (HTTP %s). %s", code, hint)
        except (ConnectionClosed, InvalidHandshake, InvalidURI, OSError, TimeoutError) as e:
            log.warning("%s: %s", type(e).__name__, e)
        if stop.is_set():
            break
        delay = BACKOFF_S[min(attempt, len(BACKOFF_S) - 1)]
        attempt += 1
        log.info("reconnecting in %ds", delay)
        stop.wait(delay)
    if grab:
        grab.close()
    return 0


def dry_run(a) -> int:
    """Open the camera, encode a few seconds of frames and report size and bandwidth. Sends nothing."""
    target, label = resolve(a.source)
    grab = Grabber(target, a.width, a.height)
    log.info("camera: %s", label)
    seq, n, total, t0, dims = 0, 0, 0, time.time(), (0, 0)
    while time.time() - t0 < a.dry_run and not stop.is_set():
        frame, seq = grab.latest(seq)
        if frame is None:
            break
        data, w, h = encode(frame, a.max_width, a.quality, a.rotate)
        if data:
            n, total, dims = n + 1, total + len(data), (w, h)
            stop.wait(1.0 / a.fps)
    grab.close()
    if not n:
        log.error("no frames captured")
        return 1
    secs = time.time() - t0
    log.info("dry run: %dx%d, %d frames in %.1fs, avg %.0f kB/frame -> %.0f kB/s, %.0f MB/hour at %.1f fps",
             dims[0], dims[1], n, secs, total / n / 1024, total / secs / 1024, total / secs * 3600 / 1e6, n / secs)
    return 0


def main() -> None:
    p = argparse.ArgumentParser(prog="camera-agent", description=__doc__.split("\n\n")[0])
    p.add_argument("--list-cameras", action="store_true")
    p.add_argument("--server", default=os.environ.get("CAMERA_AGENT_SERVER", ""),
                   help="server URL, e.g. https://abc.ngrok-free.app or http://127.0.0.1:8010")
    p.add_argument("--token", default=os.environ.get("CAMERA_AGENT_TOKEN", ""), help="shared ingest token")
    p.add_argument("--path", default="/api/ingest", help="WebSocket path (default: Overwatch /api/ingest)")
    p.add_argument("--via-uthale", action="store_true", help="shortcut for --path /api/monitor/ingest")
    p.add_argument("--source", default=os.environ.get("CAMERA_AGENT_SOURCE", ""),
                   help='REQUIRED. "name:USB 2.0 Camera" (recommended), "index:1", an rtsp:// URL or a video file. '
                        "Use --list-cameras to see names. There is deliberately no default camera.")
    p.add_argument("--width", type=int, default=0, help="requested capture width (0 = device default)")
    p.add_argument("--height", type=int, default=0)
    p.add_argument("--max-width", type=int, default=960, help="downscale before sending (0 = keep)")
    p.add_argument("--fps", type=float, default=5.0, help="frames sent per second")
    p.add_argument("--quality", type=int, default=75, help="JPEG quality 1-100")
    p.add_argument("--rotate", type=int, choices=[0, 90, 180, 270], default=0)
    p.add_argument("--dry-run", type=float, metavar="SECONDS", default=0.0,
                   help="capture and encode for N seconds, report bandwidth, send nothing")
    a = p.parse_args()
    if a.via_uthale:
        a.path = "/api/monitor/ingest"

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    if a.list_cameras:
        devs = list_devices()
        for i, n in devs:
            print(f"{i}\t{n}")
        if not devs:
            print("no devices listed (on macOS this needs ffmpeg; otherwise use --source index:N)")
        return
    if not a.source:
        p.error("--source is required. Run --list-cameras, then e.g. --source \"name:USB 2.0 Camera\". "
                "There is no default so the laptop's own camera is never streamed by accident.")
    if a.dry_run:
        raise SystemExit(dry_run(a))
    if not a.server or not a.token:
        p.error("--server and --token are required (or CAMERA_AGENT_SERVER / CAMERA_AGENT_TOKEN)")
    raise SystemExit(run(a))


if __name__ == "__main__":
    main()
