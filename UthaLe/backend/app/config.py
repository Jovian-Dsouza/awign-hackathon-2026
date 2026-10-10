"""Runtime settings, read once from environment variables (and an optional .env file)."""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field
from pathlib import Path

BACKEND_DIR = Path(__file__).resolve().parent.parent
REPO_ROOT = BACKEND_DIR.parent.parent


def _load_dotenv(path: Path) -> None:
    """Minimal KEY=VALUE loader so we don't need python-dotenv. Real env vars win."""
    if not path.is_file():
        return
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        os.environ.setdefault(key.strip(), value.strip().strip('"').strip("'"))


_load_dotenv(BACKEND_DIR / ".env")
_load_dotenv(BACKEND_DIR.parent / ".env")


def _env(name: str, default: str) -> str:
    return os.environ.get(name, default)


def _env_bool(name: str, default: bool) -> bool:
    return _env(name, "1" if default else "0").strip().lower() in {"1", "true", "yes", "on"}


@dataclass(frozen=True)
class Settings:
    # Camera
    camera_index: int = int(_env("CAMERA_INDEX", "0"))
    # "auto" -> DirectShow on Windows (MSMF is slow/flaky with many USB cams), default elsewhere.
    camera_backend: str = _env("CAMERA_BACKEND", "auto").lower()
    camera_width: int = int(_env("CAMERA_WIDTH", "1280"))
    camera_height: int = int(_env("CAMERA_HEIGHT", "720"))
    camera_fps: int = int(_env("CAMERA_FPS", "15"))
    camera_jpeg_quality: int = int(_env("CAMERA_JPEG_QUALITY", "80"))
    camera_autostart: bool = _env_bool("CAMERA_AUTOSTART", True)
    camera_overlay: bool = _env_bool("CAMERA_OVERLAY", True)
    # "usb" = local OpenCV device. "remote" = frames pushed over a WebSocket by
    # pi-streamer/stream_camera.py (e.g. a Raspberry Pi reaching this PC through ngrok).
    camera_source: str = _env("CAMERA_SOURCE", "usb").lower()
    # Shared secret the remote sender must present. Remote ingest is refused while empty.
    camera_ingest_token: str = _env("CAMERA_INGEST_TOKEN", "")

    # External monitor camera (the Overwatch service, see ../../overwatch). Empty token = the
    # relay and webhook are refused.
    monitor_url: str = _env("MONITOR_URL", "http://127.0.0.1:8010")
    monitor_ingest_token: str = _env("MONITOR_INGEST_TOKEN", "")

    # Storage
    db_path: Path = Path(_env("DB_PATH", str(BACKEND_DIR / "data" / "uthale.db")))
    evidence_dir: Path = Path(_env("EVIDENCE_DIR", str(BACKEND_DIR / "data" / "evidence")))
    inventory_path: Path = Path(_env("INVENTORY_PATH", str(REPO_ROOT / "config" / "inventory.yaml")))

    # Robot (only the simulated adapter exists today)
    robot_adapter: str = _env("ROBOT_ADAPTER", "simulated")
    sim_speed: float = float(_env("SIM_SPEED", "1.0"))          # >1 = faster demo
    sim_anomaly_rate: float = float(_env("SIM_ANOMALY_RATE", "0.15"))
    sim_sensors: bool = _env_bool("SIM_SENSORS", True)           # simulated NFC + load cell
    sim_seed: int | None = int(_env("SIM_SEED", "")) if _env("SIM_SEED", "") else None

    cors_origins: list[str] = field(
        default_factory=lambda: [
            o.strip()
            for o in _env("CORS_ORIGINS", "http://localhost:5173,http://127.0.0.1:5173").split(",")
            if o.strip()
        ]
    )

    @property
    def resolved_camera_backend(self) -> str:
        if self.camera_backend != "auto":
            return self.camera_backend
        return "dshow" if sys.platform == "win32" else "any"


settings = Settings()
