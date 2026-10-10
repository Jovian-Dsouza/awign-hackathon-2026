"""Settings, all from OVERWATCH_* environment variables (or a .env next to where you run it)."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path


def _load_dotenv(path: Path) -> None:
    """Tiny .env reader. Real environment variables win."""
    if not path.is_file():
        return
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        os.environ.setdefault(key.strip(), value.strip().strip('"').strip("'"))


def _env(name: str, default: str) -> str:
    return os.environ.get(f"OVERWATCH_{name}", default)


@dataclass(frozen=True)
class Settings:
    # --- camera -----------------------------------------------------------
    # "name:USB 2.0 Camera" | "index:1" | rtsp://... | /path/to/footage.mp4
    # | "remote": frames are pushed by camera-agent/ to /api/ingest (needs INGEST_TOKEN)
    # No default camera on purpose: guessing one risks streaming the laptop's own camera.
    source: str = ""
    ingest_token: str = ""         # shared secret a remote camera agent must present
    width: int = 1280          # requested; the device may give something else
    height: int = 960
    rotate: int = 0            # 0 | 90 | 180 | 270 clockwise, applied at capture (camera mounted upside down = 180)
    analyze_fps: float = 6.0   # detectors + annotated stream run at this rate
    loop_file: bool = True     # replay a video file endlessly (demo / offline tests)

    # --- server / storage -------------------------------------------------
    host: str = "127.0.0.1"
    port: int = 8010
    data_dir: Path = Path("data")

    # --- when to watch: only while the robot is picking -----------------------
    uthale_url: str = ""           # e.g. http://127.0.0.1:8000  (polls /api/robot/status)
    act_url: str = ""              # e.g. http://raspberrypi.local:8004 (polls /state, mode "driving")
    grace_s: float = 20.0          # keep watching this long after the pickup ends
    manual_s: float = 90.0         # length of a window started from the dashboard button

    # --- dropped-item detector ---------------------------------------------
    diff_threshold: int = 28       # colour difference that counts as changed
    item_min_frac: float = 0.0002  # smaller blobs are noise (0.02 % of the picture)
    item_max_frac: float = 0.015   # bigger blobs are the robot / a person (1.5 % of the picture)
    persist_s: float = 3.0         # an item must sit still this long before it is reported
    robot_marker_id: int = -1      # ArUco id (DICT_4X4_50) stuck on the robot; -1 = off

    # --- decision ---------------------------------------------------------
    laya_mode: str = "off"                   # off | shadow | advisory
    laya_model: str = "typed-decisions"      # checkpoint name Router understands
    laya_path: str = ""                      # fine-tuned checkpoint dir (overrides laya_model)

    # --- responders / notifiers ------------------------------------------
    responder: str = "null"        # null (dry-run) | pi
    pi_url: str = ""               # e.g. http://raspberrypi.local:8003 (firmware/combined-web)
    pi_recover_traj: str = ""      # name of a recorded arm trajectory to play on approved recovery
    auto_hold: bool = False        # stop the base automatically on a camera-confirmed incident
    webhook_url: str = ""          # JSON POST per new incident (e.g. the UthaLe backend)
    webhook_token: str = ""

    @classmethod
    def from_env(cls) -> "Settings":
        _load_dotenv(Path(".env"))
        d = cls()
        return cls(
            source=_env("SOURCE", d.source),
            ingest_token=_env("INGEST_TOKEN", d.ingest_token),
            width=int(_env("WIDTH", str(d.width))),
            height=int(_env("HEIGHT", str(d.height))),
            rotate=int(_env("ROTATE", str(d.rotate))),
            analyze_fps=float(_env("ANALYZE_FPS", str(d.analyze_fps))),
            loop_file=_env("LOOP_FILE", "1") not in ("0", "false", "no"),
            host=_env("HOST", d.host),
            port=int(_env("PORT", str(d.port))),
            data_dir=Path(_env("DATA_DIR", str(d.data_dir))),
            uthale_url=_env("UTHALE_URL", d.uthale_url).rstrip("/"),
            act_url=_env("ACT_URL", d.act_url).rstrip("/"),
            grace_s=float(_env("GRACE_S", str(d.grace_s))),
            manual_s=float(_env("MANUAL_S", str(d.manual_s))),
            diff_threshold=int(_env("DIFF_THRESHOLD", str(d.diff_threshold))),
            item_min_frac=float(_env("ITEM_MIN_FRAC", str(d.item_min_frac))),
            item_max_frac=float(_env("ITEM_MAX_FRAC", str(d.item_max_frac))),
            persist_s=float(_env("PERSIST_S", str(d.persist_s))),
            robot_marker_id=int(_env("ROBOT_MARKER_ID", str(d.robot_marker_id))),
            laya_mode=_env("LAYA", d.laya_mode),
            laya_model=_env("LAYA_MODEL", d.laya_model),
            laya_path=_env("LAYA_PATH", d.laya_path),
            responder=_env("RESPONDER", d.responder),
            pi_url=_env("PI_URL", d.pi_url).rstrip("/"),
            pi_recover_traj=_env("PI_RECOVER_TRAJ", d.pi_recover_traj),
            auto_hold=_env("AUTO_HOLD", "0") in ("1", "true", "yes"),
            webhook_url=_env("WEBHOOK_URL", d.webhook_url),
            webhook_token=_env("WEBHOOK_TOKEN", d.webhook_token),
        )
