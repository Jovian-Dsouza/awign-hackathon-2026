"""The orchestrator: camera frames -> (only while the robot is picking) detector -> decision -> incident.

    robot activity ─► WatchWindow ──► opens ─► baseline picture of the arena
                                            │
    FrameSource ─► DroppedItemDetector ◄────┘   runs ONLY inside the window
                          │
                  Observation (a small item appeared and stopped)
                          │
                 RulesDecider ─┐
                 Laya (opt.)  ─┴─► combine ─► IncidentStore ─► webhook / dashboard

Outside a pickup nothing is evaluated, so people walking by, shadows and a tidy-up never
become incidents. Camera health (covered lens, frozen feed) is always watched.

The frame loop never waits on a model: observations go on a queue and a worker thread runs
the (possibly slow) decision step.
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from overwatch.arming import RobotActivityPoller, WatchWindow
from overwatch.camera import FrameSource
from overwatch.config import Settings
from overwatch.decision import combine
from overwatch.decision.laya_decider import LayaDecider
from overwatch.decision.rules import RulesDecider
from overwatch.detectors.health import HealthMonitor
from overwatch.detectors.presence import MotionPresence
from overwatch.detectors.robot_marker import RobotMarker
from overwatch.detectors.scene_change import SceneChangeDetector
from overwatch.events import CAMERA_FAULT, DROPPED_ITEM, SPILL, Observation
from overwatch.notifiers import WebhookNotifier
from overwatch.responders import Responder
from overwatch.store import IncidentStore
from overwatch.zones import ZoneSet

log = logging.getLogger(__name__)

RED, AMBER, GREEN, GREY = (60, 60, 235), (0, 170, 255), (90, 200, 90), (170, 170, 170)


class Pipeline:
    def __init__(self, settings: Settings, source: FrameSource, store: IncidentStore, zones: ZoneSet,
                 responder: Responder, laya: LayaDecider | None = None):
        self.s = settings
        self.source, self.store, self.zones, self.responder = source, store, zones, responder
        self.laya = laya
        self.rules = RulesDecider()
        self.health = HealthMonitor()
        self.scene = SceneChangeDetector(
            zones, settings.diff_threshold, settings.item_min_frac, settings.item_max_frac,
            settings.spill_max_frac, persist_s=settings.persist_s, spill_persist_s=settings.spill_persist_s)
        self.marker = RobotMarker(settings.robot_marker_id)
        self.presence = MotionPresence(min_frac=settings.robot_min_frac)
        self.notifier = WebhookNotifier(settings.webhook_url, settings.webhook_token)
        self.window = WatchWindow(settings.grace_s, settings.manual_s)
        self.poller = RobotActivityPoller(self.window, settings.uthale_url, settings.pi_url, settings.act_url)

        self.data_dir = Path(settings.data_dir)
        self.snap_dir = self.data_dir / "snapshots"
        self.snap_dir.mkdir(parents=True, exist_ok=True)
        self.zones_path = self.data_dir / "zones.json"

        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._worker: threading.Thread | None = None
        self._q: queue.Queue = queue.Queue(maxsize=32)
        self._lock = threading.Lock()
        self._jpeg: bytes | None = None
        self._frame_id = -1
        self._robot_in_view = False
        # Monitoring is OFF until a person says "start from here": the robot and camera are not at
        # fixed positions, so what the camera sees at startup is not assumed to be right.
        self._monitoring = False
        self._setup_at: float | None = None
        self._scene_lock = threading.Lock()
        self.analysed = 0

    # -- lifecycle -----------------------------------------------------------
    def start(self) -> None:
        self.source.start()
        self.poller.start()
        self._stop.clear()
        self._worker = threading.Thread(target=self._decide_loop, name="overwatch-decide", daemon=True)
        self._worker.start()
        self._thread = threading.Thread(target=self._frame_loop, name="overwatch-frames", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self.poller.stop()
        for t in (self._thread, self._worker):
            if t:
                t.join(3.0)
        self.source.stop()

    def start_monitoring(self) -> bool:
        """Take the picture as it is right now as the baseline, and monitor from this position."""
        f = self.source.latest()
        if f is None:
            return False
        region = RobotMarker(self.s.robot_marker_id).observe(f.image)    # own instance: no shared state
        with self._scene_lock:
            self.scene.set_reference(f.image, region)
            self._monitoring, self._setup_at = True, time.time()
        log.info("Monitoring from this position (robot tag %s)", "in view" if region is not None else "not in view")
        return True

    def stop_monitoring(self) -> None:
        with self._scene_lock:
            self._monitoring, self._setup_at = False, None
            self.scene.clear_reference()

    def save_zones(self) -> None:
        self.zones.save(self.zones_path)

    # -- consumers -------------------------------------------------------------
    def latest_jpeg(self) -> bytes | None:
        with self._lock:
            return self._jpeg

    def latest_frame_id(self) -> int:
        with self._lock:
            return self._frame_id

    def status(self) -> dict:
        laya = None
        if self.s.laya_mode != "off":
            laya = {"mode": self.s.laya_mode, "available": bool(self.laya and self.laya.available),
                    "error": self.laya.error if self.laya else "not configured",
                    "model": self.s.laya_path or self.s.laya_model,
                    "load_s": self.laya.load_s if self.laya else None}
        ig = self.scene.ignored
        warnings = [f"Camera: {f.replace('_', ' ')}" for f in sorted(self.health.active)]
        if self._monitoring and self.scene.paused_reason:
            warnings.append(self.scene.paused_reason)
        return {
            "camera": self.source.status(),
            "robot": {"in_view": self._robot_in_view, "tracking": "tag" if self.marker.enabled else "motion",
                      "tag_id": self.s.robot_marker_id if self.marker.enabled else None},
            "monitoring": {"on": self._monitoring, "since": self._setup_at,
                           "view_moved": bool(self._monitoring and self.scene.paused_reason)},
            "camera_warnings": warnings,
            "watch": self.window.state(),
            "alignment": self.scene.alignment,
            "ignored_changes": {"robot": ig.robot, "too_big": ig.too_big, "too_small": ig.too_small,
                                "not_compact": ig.not_compact, "outside_zone": ig.outside_zone, "left": ig.left},
            "health": {"active_faults": sorted(self.health.active), **self.health.metrics},
            "zones": [z.to_dict() for z in self.zones.zones],
            "robot_marker": {"enabled": self.marker.enabled, "visible": self.marker.visible,
                             "id": self.s.robot_marker_id},
            "laya": laya,
            "responder": self.responder.status(),
            "auto_hold": self.s.auto_hold,
            "webhook": self.notifier.enabled,
            "analysed_frames": self.analysed,
            "open_incidents": len(self.store.list("open")),
            "persist_s": self.s.persist_s,
            "item_size_pct": [round(self.s.item_min_frac * 100, 3), round(self.s.item_max_frac * 100, 2)],
        }

    # -- frame loop --------------------------------------------------------------
    def _frame_loop(self) -> None:
        period = 1.0 / max(self.s.analyze_fps, 0.5)
        last_id = -1
        while not self._stop.is_set():
            t0 = time.time()
            f = self.source.latest()
            if f is None or f.id == last_id:
                self._stop.wait(0.03)
                continue
            last_id = f.id
            img, now = f.image, f.ts
            try:
                self.health.process(img, now)           # camera trouble is a status note, never an incident
                region = self.marker.observe(img)
                # Is the robot in view and working? Tag if there is one, else robot-sized motion.
                if self.marker.enabled:
                    in_view = self.marker.visible
                    self.window.set_signal("robot tag", in_view, "tag in view")
                else:
                    in_view = self.presence.process(img)
                    self.window.set_signal("robot motion", in_view, "robot-sized motion in view")
                self._robot_in_view = in_view
                w = self.window.state()
                obs: list[Observation] = []
                with self._scene_lock:
                    if self._monitoring:
                        if not w["watching"]:
                            self.scene.check_view(img)         # notice a moved camera even while idle
                            self.scene.blobs = []
                        else:
                            new, gone = self.scene.process(img, now, region, spills=self.marker.enabled)
                            obs += new
                            for tid in gone:
                                for iid in self.store.mark_cleared(tid):
                                    log.info("INC-%04d: no longer visible", iid)
                    else:
                        self.scene.blobs = []
                for o in obs:
                    try:
                        self._q.put_nowait((o, img.copy()))
                    except queue.Full:
                        log.warning("Decision queue full; dropped observation %s", o.kind)
                self._publish(img, now, w)
                self.analysed += 1
            except Exception:
                log.exception("frame analysis failed")
            self._stop.wait(max(0.0, period - (time.time() - t0)))

    def _publish(self, img: np.ndarray, now: float, w: dict) -> None:
        vis = img.copy()
        h, wd = vis.shape[:2]
        for z in self.zones.zones:
            pts = np.array([[int(x * (wd - 1)), int(y * (h - 1))] for x, y in z.points], np.int32)
            col = GREY if z.mode == "ignore" else GREEN
            cv2.polylines(vis, [pts], True, col, 1, cv2.LINE_AA)
            cv2.putText(vis, f"{z.name}{' (ignored)' if z.mode == 'ignore' else ''}", tuple(pts[0] + [4, 14]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)
        if self._monitoring and w["watching"]:
            for b in self.scene.blobs:
                x, y, bw, bh = b.bbox
                color = RED if b.reported else AMBER
                cv2.rectangle(vis, (x - 3, y - 3), (x + bw + 3, y + bh + 3), color, 2)
                label = ("SPILL" if b.kind == SPILL else "DROPPED") if b.reported else "checking"
                cv2.putText(vis, label, (x, max(14, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2, cv2.LINE_AA)
        if not self._monitoring:
            banner = [("NOT MONITORING: place the camera and robot, then press Start monitoring from here", AMBER)]
        elif w["watching"]:
            banner = [(f"WATCHING THE ROBOT: {w['why']}", GREEN)]
        else:
            banner = [("monitoring from your position: waiting for the robot to move", GREY)]
        if self._monitoring and self.scene.paused_reason:
            banner.append((self.scene.paused_reason, AMBER))
        for fault in sorted(self.health.active):
            banner.append((f"CAMERA: {fault.replace('_', ' ')}", AMBER))
        for i, (text, col) in enumerate(banner):
            cv2.rectangle(vis, (0, i * 24), (wd, i * 24 + 22), (28, 28, 28), -1)
            cv2.putText(vis, text, (8, i * 24 + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
        stamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now))
        cv2.putText(vis, stamp, (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(vis, stamp, (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (235, 235, 235), 1, cv2.LINE_AA)
        ok, buf = cv2.imencode(".jpg", vis, [int(cv2.IMWRITE_JPEG_QUALITY), 78])
        if ok:
            with self._lock:
                self._jpeg = buf.tobytes()
                self._frame_id += 1

    # -- decision worker -----------------------------------------------------------
    def _decide_loop(self) -> None:
        while not self._stop.is_set():
            try:
                obs, frame = self._q.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self._handle(obs, frame)
            except Exception:
                log.exception("handling observation failed")

    def _handle(self, obs: Observation, frame: np.ndarray) -> dict:
        ctx = {"robot_marker_visible": self.marker.visible}
        rules = self.rules.decide(obs, ctx)
        model = None
        if self.laya and self.s.laya_mode in ("shadow", "filter"):
            model = self.laya.decide(obs, ctx)
        final = combine(rules, model, self.s.laya_mode)

        snap = self._snapshot(frame, obs)
        inc = self.store.add(kind=obs.kind, zone=obs.zone, track_id=obs.track_id, observation=obs.to_dict(),
                             final=final.to_dict(), rules=rules.to_dict(),
                             laya=model.to_dict() if model else None, snapshot=snap,
                             status="filtered" if final.action == "ignore" else "open")
        log.info("%s %s -> %s%s", inc["code"], obs.kind, final.action,
                 " (filtered as false alarm)" if final.action == "ignore" else "")
        if final.action == "ignore":
            return inc                       # a false alarm is recorded, never alerted
        if self.s.auto_hold and obs.kind in (DROPPED_ITEM, SPILL):
            res = self.responder.hold()
            log.info("%s auto-hold: %s", inc["code"], res.get("detail"))
        self.notifier.notify(inc)
        return inc

    def _snapshot(self, frame: np.ndarray, obs: Observation) -> str | None:
        vis = frame.copy()
        if obs.bbox:
            x, y, w, h = obs.bbox
            cv2.rectangle(vis, (x - 4, y - 4), (x + w + 4, y + h + 4), RED, 2)
        name = f"{int(obs.ts * 1000)}_{obs.kind}.jpg"
        ok, buf = cv2.imencode(".jpg", vis, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
        if not ok:
            return None
        (self.snap_dir / name).write_bytes(buf.tobytes())
        return name
