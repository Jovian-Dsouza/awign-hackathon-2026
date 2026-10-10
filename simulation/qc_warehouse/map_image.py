"""Render the SLAM occupancy map with the estimated and true trajectories to a PNG."""

from __future__ import annotations

import struct
import zlib
from collections.abc import Sequence
from pathlib import Path

import numpy as np

from qc_warehouse.autonomy.frames import BasePose

FREE, UNKNOWN, OCCUPIED = (255, 255, 255), (200, 200, 200), (30, 30, 30)
ESTIMATE, TRUTH = (220, 40, 40), (40, 160, 60)
SCALE = 4  # pixels per map cell


def write_png(path: Path, rgb: np.ndarray) -> None:
    """Minimal RGB PNG encoder (no image library needed)."""
    if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
        raise ValueError("expected an (H, W, 3) uint8 image")
    height, width, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[row].tobytes() for row in range(height))

    def chunk(kind: bytes, payload: bytes) -> bytes:
        return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF)

    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header) + chunk(b"IDAT", zlib.compress(raw)) + chunk(b"IEND", b""))


def map_image(
    values: np.ndarray,
    origin: tuple[float, float],
    resolution: float,
    trajectory: Sequence[tuple[BasePose, BasePose]] = (),
) -> np.ndarray:
    """values: [nx, ny] occupancy (-1 unknown, 0..100). Returns an image with +y up."""
    img = np.empty(values.shape + (3,), dtype=np.uint8)
    img[:] = UNKNOWN
    img[(values >= 0) & (values < 65)] = FREE
    img[values >= 65] = OCCUPIED
    for estimate, truth in trajectory:
        for pose, colour in ((truth, TRUTH), (estimate, ESTIMATE)):
            ix = int(round((pose.x - origin[0]) / resolution))
            iy = int(round((pose.y - origin[1]) / resolution))
            if 0 <= ix < values.shape[0] and 0 <= iy < values.shape[1]:
                img[ix, iy] = colour
    picture = img.transpose(1, 0, 2)[::-1]  # rows = -y, columns = x
    return np.repeat(np.repeat(picture, SCALE, axis=0), SCALE, axis=1)
