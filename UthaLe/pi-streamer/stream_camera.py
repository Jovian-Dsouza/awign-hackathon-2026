#!/usr/bin/env python3
"""Stream a Raspberry Pi camera to the UthaLe dashboard through an ngrok tunnel.

The Pi is the *client*: it opens one WebSocket to the PC's public ngrok URL
(`wss://<your-ngrok-domain>/api/camera/ingest`) and sends every frame as one
binary JPEG message. The PC's UthaLe backend (run with CAMERA_SOURCE=remote)
puts those frames into its normal camera stream, so the Live Robot page and QC
evidence use the Pi camera exactly like a USB camera.

One WebSocket is used instead of one HTTP request per frame because ngrok's free
plan limits HTTP request counts, and per-request round trips through the tunnel
would cap the frame rate.

Camera: the Pi Camera Module via Picamera2 when available, else a USB webcam via
OpenCV. Or `--camera relay`: re-send the MJPEG stream of a robot script that
already owns the camera (web_combined / web_drive / ACT infer / record), so this
never competes with those scripts for the device. Network drops, ngrok restarts
and camera hiccups are retried with backoff.

    python3 stream_camera.py --server https://<your-ngrok-domain> --token <CAMERA_INGEST_TOKEN>
"""

from __future__ import annotations

import argparse
import logging
import os
import signal
import threading
import time
import urllib.request
from datetime import datetime
from urllib.parse import urlsplit, urlunsplit

try:
    import cv2  # not needed for --camera relay
except ImportError:
    cv2 = None
from websockets.exceptions import ConnectionClosed, InvalidHandshake, InvalidStatus, InvalidURI
from websockets.sync.client import connect

log = logging.getLogger("pi-streamer")

INGEST_PATH = "/api/camera/ingest"
BACKOFF_S = (1, 2, 4, 8, 10)
MAX_READ_FAILURES = 30
HEALTHY_SESSION_S = 10.0  # a connection lasting this long resets the retry backoff
STATS_EVERY_S = 10.0
# MJPEG streams served by the existing robot scripts in firmware/, most capable first.
DEFAULT_RELAY_URLS = (
    "http://127.0.0.1:8003/stream",  # combined-web/web_combined.py
    "http://127.0.0.1:8002/stream",  # mecanum-drive/web_drive.py
    "http://127.0.0.1:8004/stream",  # act-pick-infer/infer.py
    "http://127.0.0.1:8005/stream",  # act-pick-infer/record.py
)
RELAY_FRAME_TIMEOUT_S = 0.5
MAX_RELAY_BUFFER = 8 * 1024 * 1024

stop_event = threading.Event()


class CameraError(RuntimeError):
    pass


# ---------------- camera sources ----------------
class PiCameraSource:
    """Raspberry Pi Camera Module (CSI ribbon cable) via Picamera2 / libcamera."""

    name = "picamera2"

    def __init__(self, width: int, height: int, fps: int):
        from picamera2 import Picamera2  # installed with: sudo apt install python3-picamera2

        if not Picamera2.global_camera_info():
            raise CameraError("No Pi camera module detected (check the ribbon cable)")
        self.cam = Picamera2()
        # "RGB888" is stored as B,G,R per pixel, which is what OpenCV expects.
        config = self.cam.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"},
            controls={"FrameRate": float(fps)},
        )
        self.cam.configure(config)
        self.cam.start()

    def read(self):
        return self.cam.capture_array("main")

    def close(self) -> None:
        try:
            self.cam.stop()
        finally:
            self.cam.close()


class USBCameraSource:
    """USB webcam via OpenCV / V4L2."""

    name = "usb"

    def __init__(self, index: int, width: int, height: int, fps: int):
        self.cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap.release()
            raise CameraError(f"Could not open USB camera /dev/video{index}")
        # MJPG lets most webcams deliver 720p at full rate over USB 2.
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def read(self):
        ok, frame = self.cap.read()
        return frame if ok else None

    def close(self) -> None:
        self.cap.release()


class RelaySource:
    """Re-sends the MJPEG stream of a robot script that already owns the camera.

    It never opens the camera device, so scripts that need the device keep working.
    Raises CameraError when none of the URLs is serving a stream.
    """

    def __init__(self, urls: list[str]):
        self.resp = None
        for url in urls:
            try:
                resp = urllib.request.urlopen(url, timeout=5)
            except (OSError, ValueError):
                continue
            if "multipart" not in resp.headers.get("Content-Type", ""):
                resp.close()
                continue
            self.resp, self.url = resp, url
            break
        if self.resp is None:
            raise CameraError("No robot script is serving the camera yet (web_combined / web_drive / "
                              "infer / record). Waiting for one to start.")
        self.name = f"relay of {self.url}"
        self._lock = threading.Lock()
        self._latest: bytes | None = None
        self._seq = self._taken = 0
        self._dead = False
        threading.Thread(target=self._reader, name="mjpeg-relay", daemon=True).start()

    def _reader(self) -> None:
        """Split the multipart stream on JPEG start/end markers; keep only the newest frame."""
        buf = bytearray()
        try:
            while True:
                chunk = self.resp.read1(65536)
                if not chunk:
                    break
                buf += chunk
                while True:
                    start = buf.find(b"\xff\xd8")
                    if start < 0:
                        del buf[:-1]
                        break
                    end = buf.find(b"\xff\xd9", start + 2)
                    if end < 0:
                        del buf[:start]
                        break
                    jpeg = bytes(buf[start:end + 2])
                    del buf[:end + 2]
                    with self._lock:
                        if jpeg != self._latest:  # the scripts re-send unchanged frames
                            self._latest, self._seq = jpeg, self._seq + 1
                if len(buf) > MAX_RELAY_BUFFER:
                    buf.clear()
        except (OSError, ValueError):
            pass
        finally:
            self._dead = True

    def read_jpeg(self) -> bytes | None:
        deadline = time.monotonic() + RELAY_FRAME_TIMEOUT_S
        while time.monotonic() < deadline:
            if self._dead:
                raise CameraError(f"Camera stream at {self.url} ended (robot script stopped?)")
            with self._lock:
                if self._seq != self._taken:
                    self._taken = self._seq
                    return self._latest
            time.sleep(0.01)
        return None

    def close(self) -> None:
        try:
            self.resp.close()
        except OSError:
            pass


def open_camera(args):
    if args.camera == "relay":
        return RelaySource(args.relay_urls)
    if cv2 is None:
        raise SystemExit("OpenCV is required for this camera mode: pip install opencv-python-headless")
    if args.camera in ("auto", "picamera"):
        try:
            return PiCameraSource(args.width, args.height, args.fps)
        except ImportError:
            if args.camera == "picamera":
                raise CameraError("picamera2 is not installed: sudo apt install python3-picamera2")
            log.info("picamera2 not installed; using USB camera")
        except Exception as e:  # noqa: BLE001 - libcamera raises assorted errors
            if args.camera == "picamera":
                raise CameraError(f"Pi camera failed to start: {e}") from e
            log.info("Pi camera unavailable (%s); using USB camera", e)
    return USBCameraSource(args.usb_index, args.width, args.height, args.fps)


# ---------------- streaming ----------------
def ingest_url(server: str) -> str:
    """https://abc.ngrok-free.app -> wss://abc.ngrok-free.app/api/camera/ingest"""
    if "://" not in server:
        server = "https://" + server
    parts = urlsplit(server)
    scheme = {"https": "wss", "http": "ws", "wss": "wss", "ws": "ws"}.get(parts.scheme)
    if not scheme or not parts.netloc:
        raise SystemExit(f"Invalid --server URL: {server!r}")
    return urlunsplit((scheme, parts.netloc, INGEST_PATH, "", ""))


def draw_overlay(frame, label: str) -> None:
    text = f"{label}  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}"
    h = frame.shape[0]
    cv2.putText(frame, text, (12, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(frame, text, (12, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (235, 235, 235), 1, cv2.LINE_AA)


def relay_frames(ws, source: RelaySource, args) -> None:
    """Forward JPEGs unchanged (no decode/re-encode), at most --fps per second."""
    interval = 1.0 / max(args.fps, 1)
    failures, sent, sent_bytes = 0, 0, 0
    stats_t = time.monotonic()
    while not stop_event.is_set():
        started = time.monotonic()
        data = source.read_jpeg()
        if data is None:
            failures += 1
            if failures >= MAX_READ_FAILURES:
                raise CameraError(f"No new frames from {source.url} (is the camera plugged in?)")
            continue
        failures = 0
        ws.send(data)
        sent += 1
        sent_bytes += len(data)
        now = time.monotonic()
        if now - stats_t >= STATS_EVERY_S:
            secs = now - stats_t
            log.info("Relaying at %.1f fps, %.0f kB/s", sent / secs, sent_bytes / secs / 1024)
            sent, sent_bytes, stats_t = 0, 0, now
        stop_event.wait(max(0.0, interval - (time.monotonic() - started)))


def stream_frames(ws, camera, args) -> None:
    if isinstance(camera, RelaySource):
        return relay_frames(ws, camera, args)
    interval = 1.0 / max(args.fps, 1)
    encode = [int(cv2.IMWRITE_JPEG_QUALITY), args.quality]
    rotate = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}.get(args.rotate)
    failures, sent, sent_bytes = 0, 0, 0
    stats_t = time.monotonic()

    while not stop_event.is_set():
        started = time.monotonic()
        frame = camera.read()
        if frame is None:
            failures += 1
            if failures >= MAX_READ_FAILURES:
                raise CameraError("Camera stopped delivering frames")
            time.sleep(0.05)
            continue
        failures = 0

        if rotate is not None:
            frame = cv2.rotate(frame, rotate)
        if args.overlay:
            draw_overlay(frame, args.label)
        ok, buf = cv2.imencode(".jpg", frame, encode)
        if not ok:
            continue
        data = buf.tobytes()
        ws.send(data)  # blocks when the uplink is slow, so frames never pile up here
        sent += 1
        sent_bytes += len(data)

        now = time.monotonic()
        if now - stats_t >= STATS_EVERY_S:
            secs = now - stats_t
            log.info("Streaming %dx%d at %.1f fps, %.0f kB/s",
                     frame.shape[1], frame.shape[0], sent / secs, sent_bytes / secs / 1024)
            sent, sent_bytes, stats_t = 0, 0, now
        stop_event.wait(max(0.0, interval - (time.monotonic() - started)))


def run(args) -> None:
    url = ingest_url(args.server)
    headers = {
        "Authorization": f"Bearer {args.token}",
        "ngrok-skip-browser-warning": "1",  # skip the free-plan interstitial page
    }
    camera = None
    attempt = 0
    last_camera_error = None
    log.info("Sending camera frames to %s", url)

    while not stop_event.is_set():
        quiet = False
        try:
            if camera is None:
                camera = open_camera(args)
                log.info("Camera opened (%s)", camera.name)
                last_camera_error = None
            with connect(url, additional_headers=headers, open_timeout=15, compression=None,
                         max_size=2**20, ping_interval=20, ping_timeout=20) as ws:
                log.info("Connected to UthaLe backend")
                connected_at = time.monotonic()
                try:
                    stream_frames(ws, camera, args)
                finally:
                    if time.monotonic() - connected_at >= HEALTHY_SESSION_S:
                        attempt = 0
        except InvalidStatus as e:
            code = e.response.status_code
            if code == 403:
                log.error("Backend refused the connection (HTTP 403). Check --token matches CAMERA_INGEST_TOKEN "
                          "and the backend runs with CAMERA_SOURCE=remote.")
            elif code == 404:
                log.error("HTTP 404: is the ngrok tunnel up and pointing at the UthaLe backend (port 8000)?")
            else:
                log.error("Backend rejected the connection: HTTP %s", code)
        except ConnectionClosed as e:
            reason = e.rcvd.reason if e.rcvd and e.rcvd.reason else "connection lost"
            log.warning("Disconnected: %s", reason)
        except CameraError as e:
            # Log a repeated camera error once, not every retry (e.g. while no robot script runs).
            quiet = str(e) == last_camera_error
            log.log(logging.DEBUG if quiet else logging.WARNING, "%s", e)
            last_camera_error = str(e)
            if camera is not None:
                camera.close()
                camera = None
        except InvalidURI as e:
            raise SystemExit(f"Invalid server URL: {e}") from e
        except (OSError, TimeoutError, InvalidHandshake) as e:
            log.warning("Could not reach backend: %s", e)

        if stop_event.is_set():
            break
        delay = BACKOFF_S[min(attempt, len(BACKOFF_S) - 1)]
        attempt += 1
        log.log(logging.DEBUG if quiet else logging.INFO, "Retrying in %ds", delay)
        stop_event.wait(delay)

    if camera is not None:
        camera.close()
    log.info("Stopped")


def parse_args():
    p = argparse.ArgumentParser(description="Stream a Raspberry Pi camera to the UthaLe dashboard via ngrok.")
    p.add_argument("--server", default=os.environ.get("UTHALE_SERVER"),
                   help="Public ngrok URL of the PC, e.g. https://abc123.ngrok-free.app (env UTHALE_SERVER)")
    p.add_argument("--token", default=os.environ.get("UTHALE_INGEST_TOKEN"),
                   help="Must equal CAMERA_INGEST_TOKEN on the PC (env UTHALE_INGEST_TOKEN)")
    p.add_argument("--camera", choices=("auto", "picamera", "usb", "relay"), default="auto",
                   help="auto = Pi Camera Module if present, else USB webcam; relay = re-send the stream "
                        "of a running robot script instead of opening the camera")
    p.add_argument("--relay-urls", default=",".join(DEFAULT_RELAY_URLS),
                   help="Comma-separated MJPEG URLs tried in order for --camera relay")
    p.add_argument("--usb-index", type=int, default=0, help="USB webcam index (/dev/videoN)")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=int, default=10)
    p.add_argument("--quality", type=int, default=70, help="JPEG quality 1-100 (lower = less bandwidth)")
    p.add_argument("--rotate", type=int, choices=(0, 90, 180, 270), default=0)
    p.add_argument("--label", default="UthaLe PI-CAM", help="Text burned into the frame before the timestamp")
    p.add_argument("--no-overlay", dest="overlay", action="store_false", help="Don't burn a label/timestamp in")
    args = p.parse_args()
    if not args.server:
        p.error("--server is required (or set UTHALE_SERVER)")
    if not args.token:
        p.error("--token is required (or set UTHALE_INGEST_TOKEN)")
    args.relay_urls = [u.strip() for u in args.relay_urls.split(",") if u.strip()]
    return args


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    args = parse_args()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop_event.set())
    run(args)


if __name__ == "__main__":
    main()
