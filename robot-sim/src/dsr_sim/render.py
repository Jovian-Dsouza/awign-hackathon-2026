"""Render the settled scene from each named camera to PNG (headless, no viewer needed).

Usage:  uv run python -m dsr_sim.render [--scene S] [--out-dir renders] [--settle 1.0]
"""

from __future__ import annotations

import argparse
import sys
import zlib
from pathlib import Path

import mujoco
import numpy as np

from dsr_sim.constants import DEFAULT_SCENE, PROJECT_ROOT

DEFAULT_CAMERAS = ("overview", "top_down", "shelf_A_cam", "drop_tray_cam", "wrist_cam", "chassis_cam")
WIDTH, HEIGHT = 1280, 800


def write_png(path: Path, rgb: np.ndarray) -> None:
    """Minimal dependency-free RGB PNG writer."""
    height, width, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[row].tobytes() for row in range(height))

    def chunk(tag: bytes, data: bytes) -> bytes:
        body = tag + data
        return len(data).to_bytes(4, "big") + body + zlib.crc32(body).to_bytes(4, "big")

    header = width.to_bytes(4, "big") + height.to_bytes(4, "big") + bytes([8, 2, 0, 0, 0])
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header) + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def settle(model: mujoco.MjModel, seconds: float) -> mujoco.MjData:
    data = mujoco.MjData(model)
    if model.nkey and mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home") >= 0:
        mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)  # arm folded, ctrl holds it
    while data.time < seconds:
        mujoco.mj_step(model, data)
    return data


def render_cameras(model: mujoco.MjModel, data: mujoco.MjData, cameras: tuple[str, ...], out_dir: Path) -> list[Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    written = []
    with mujoco.Renderer(model, HEIGHT, WIDTH) as renderer:
        for cam in cameras:
            if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, cam) < 0:
                raise ValueError(f"camera {cam!r} not found in scene")
            renderer.update_scene(data, camera=cam)
            path = out_dir / f"{cam}.png"
            write_png(path, renderer.render())
            written.append(path)
    return written


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--out-dir", type=Path, default=PROJECT_ROOT / "renders")
    parser.add_argument("--settle", type=float, default=1.0, help="seconds to simulate before rendering")
    parser.add_argument("--cameras", nargs="+", default=list(DEFAULT_CAMERAS))
    args = parser.parse_args(argv)

    if not args.scene.exists():
        print(f"[error] scene not found: {args.scene}. Run `uv run python -m dsr_sim.build_scene`.", file=sys.stderr)
        return 1
    try:
        model = mujoco.MjModel.from_xml_path(str(args.scene))
        data = settle(model, args.settle)
        paths = render_cameras(model, data, tuple(args.cameras), args.out_dir)
    except ValueError as exc:
        print(f"[error] {exc}", file=sys.stderr)
        return 1
    for path in paths:
        print(f"[ok] {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
