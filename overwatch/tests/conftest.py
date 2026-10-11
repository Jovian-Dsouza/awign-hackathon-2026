import time

import cv2
import numpy as np
import pytest

from overwatch.camera import Frame


def textured_background(w: int = 640, h: int = 480, seed: int = 1) -> np.ndarray:
    """A floor-like scene with real texture so edge/detail statistics behave like a camera image."""
    rng = np.random.default_rng(seed)
    base = rng.integers(90, 140, (h // 8, w // 8, 3), dtype=np.uint8)
    img = cv2.resize(base, (w, h), interpolation=cv2.INTER_CUBIC)
    noise = rng.normal(0, 3, img.shape)
    return np.clip(img + noise, 0, 255).astype(np.uint8)


def with_box(img: np.ndarray, x: int, y: int, w: int, h: int, color=(30, 30, 200)) -> np.ndarray:
    out = img.copy()
    cv2.rectangle(out, (x, y), (x + w, y + h), color, -1)
    cv2.rectangle(out, (x + 3, y + 3), (x + w - 3, y + h - 3), (240, 240, 240), 2)  # edges, like a label
    return out


class FakeSource:
    """Stands in for a camera. Every latest() call is a new frame, like a fast camera."""

    def __init__(self, image: np.ndarray):
        self.image = image
        self._id = 0
        self.started = False

    def set(self, image: np.ndarray) -> None:
        self.image = image

    def start(self):
        self.started = True

    def stop(self):
        self.started = False

    def latest(self):
        self._id += 1
        noise = np.random.default_rng(self._id).normal(0, 2.0, self.image.shape)   # sensor noise
        img = np.clip(self.image.astype(np.float32) + noise, 0, 255).astype(np.uint8)
        return Frame(img, time.time(), self._id)

    def status(self):
        return {"state": "streaming", "source": "fake", "error": None, "width": self.image.shape[1],
                "height": self.image.shape[0], "fps": 30.0, "frames": self._id, "last_frame_at": time.time()}


@pytest.fixture
def bg():
    return textured_background()
