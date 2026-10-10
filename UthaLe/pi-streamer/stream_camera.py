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
OpenCV. Network drops, ngrok restarts and camera hiccups are retried with backoff.

    python3 stream_camera.py --server https://<your-ngrok-domain> --token <CAMERA_INGEST_TOKEN>
"""

from __future__ import annotations

import argparse
import logging
import os
import signal
import threading
import time
from datetime import datetime
from urllib.parse import urlsplit, urlunsplit

import cv2
from websockets.exceptions import ConnectionClosed, InvalidHandshake, InvalidStatus, InvalidURI
from websockets.sync.client import connect

log = logging.getLogger("pi-streamer")

INGEST_PATH = "/api/camera/ingest"
BACKOFF_S = (1, 2, 4, 8, 10)
MAX_READ_FAILURES = 30
HEALTHY_SESSION_S = 10.0  # a connection lasting this long resets the retry backoff
STATS_EVERY_S = 10.0

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


def open_camera(args):
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


def stream_frames(ws, camera, args) -> None:
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
    log.info("Sending camera frames to %s", url)

    while not stop_event.is_set():
        try:
            if camera is None:
                camera = open_camera(args)
                log.info("Camera opened (%s)", camera.name)
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
            log.error("%s", e)
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
        log.info("Retrying in %ds", delay)
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
    p.add_argument("--camera", choices=("auto", "picamera", "usb"), default="auto",
                   help="auto = Pi Camera Module if present, else USB webcam")
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
    return args


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    args = parse_args()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop_event.set())
    run(args)


if __name__ == "__main__":
    main()
