"""`uv run overwatch`: start the service. `uv run overwatch --list-cameras` to see devices."""

from __future__ import annotations

import argparse
import dataclasses
import logging

import uvicorn

from overwatch.app import create_app
from overwatch.camera import list_video_devices
from overwatch.config import Settings


def main() -> None:
    p = argparse.ArgumentParser(prog="overwatch", description="External-camera monitor for the dark-store robot")
    p.add_argument("--list-cameras", action="store_true", help="print local video devices and exit")
    p.add_argument("--source", help='"name:USB 2.0 Camera", "index:1", an rtsp:// URL or a video file')
    p.add_argument("--rotate", type=int, choices=[0, 90, 180, 270], help="rotate the picture clockwise (180 = mounted upside down)")
    p.add_argument("--port", type=int)
    p.add_argument("--host")
    p.add_argument("--laya", choices=["off", "shadow", "advisory"], help="Laya decision model mode")
    a = p.parse_args()

    if a.list_cameras:
        devs = list_video_devices()
        for idx, name in devs:
            print(f"{idx}\t{name}")
        if not devs:
            print("no video devices found (is ffmpeg installed on macOS?)")
        return

    s = Settings.from_env()
    over = {k: v for k, v in {"source": a.source, "port": a.port, "host": a.host, "laya_mode": a.laya, "rotate": a.rotate}.items()
            if v is not None}
    s = dataclasses.replace(s, **over)

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    uvicorn.run(create_app(s), host=s.host, port=s.port, log_level="warning")


if __name__ == "__main__":
    main()
