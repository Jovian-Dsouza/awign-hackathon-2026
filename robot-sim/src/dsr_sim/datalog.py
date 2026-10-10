"""Recording runs into the project's `data/` folder (sibling of robot-sim/ in awign-hackathon-2026).

Every recorded run produces:
  data/runs.jsonl                         one JSON line per run: parameters, outcome, timings, key numbers
  data/episodes/<id>/meta.json            the same plus the log and a snapshot of the tuned constants
  data/episodes/<id>/trajectory.npz       time series at ~10 Hz (joint targets + measured, rover pose, item, ...)
  data/episodes/<id>/frames/wrist/*.jpg   camera frames at the same rate (optional)
  data/calibration/sim_parameters.json    everything about the simulated robot/scene a real build needs to match

See data/README.md for how these map to the real hardware.
"""

from __future__ import annotations

import contextlib
import hashlib
import io
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import mujoco
import numpy as np

from dsr_sim import pick as P
from dsr_sim import robot as R
from dsr_sim import scenarios as S
from dsr_sim import world as W
from dsr_sim.constants import DEFAULT_INVENTORY, DEFAULT_LAYOUT, PROJECT_ROOT

DATA_DIR = PROJECT_ROOT.parent / "data"
EPISODES_DIR = DATA_DIR / "episodes"
RUNS_FILE = DATA_DIR / "runs.jsonl"
CALIBRATION_FILE = DATA_DIR / "calibration" / "sim_parameters.json"
SAMPLE_HZ = 10.0
IMAGE_SIZE = (480, 640)  # height, width: what the real USB camera streams (firmware: 640x480)
ARM_JOINTS = R.ARM_JOINTS


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()[:12] if path.exists() else "missing"


class _Splitter(io.TextIOBase):
    """Forwards writes to the previous stdout and keeps a copy of the lines."""

    def __init__(self, previous, lines: list[str]) -> None:
        self.previous, self.lines, self._buf = previous, lines, ""

    def write(self, text: str) -> int:
        self.previous.write(text)
        self._buf += text
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            if line.strip():
                self.lines.append(line.rstrip())
        return len(text)

    def flush(self) -> None:
        self.previous.flush()


class Recorder:
    def __init__(self, task: P.Task, scenario: S.Scenario, params: W.Params, images: bool = True, hz: float = SAMPLE_HZ) -> None:
        self.task, self.scenario, self.params, self.images, self.hz = task, scenario, params, images, hz
        self.id = datetime.now().strftime("%Y%m%d-%H%M%S") + f"-{scenario.name}"
        self.dir = EPISODES_DIR / self.id
        self.rows: dict[str, list] = {k: [] for k in ("t", "stage", "base_pose", "base_cmd", "joints_deg", "joints_cmd_deg", "item_pos", "jaw_centre")}
        self.stages: list[str] = []
        self.stage_t0: dict[str, float] = {}
        self._last = -1e9
        self._renderer = None
        self.n_frames = 0
        m = task.model
        self.item = W.target_item(scenario.name, params)
        self._joint_adr = [m.jnt_qposadr[m.joint(R.ARM_PREFIX + j).id] for j in ARM_JOINTS]

    def attach(self) -> None:
        self.task.observers.append(self.on_step)

    def detach(self) -> None:
        if self.on_step in self.task.observers:
            self.task.observers.remove(self.on_step)
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None

    def on_step(self, task: P.Task) -> None:
        d = task.data
        if d.time - self._last < 1.0 / self.hz:
            return
        self._last = d.time
        stage = task.stage or "start"
        if stage not in self.stages:
            self.stages.append(stage)
            self.stage_t0[stage] = float(d.time)
        r = self.rows
        r["t"].append(float(d.time))
        r["stage"].append(self.stages.index(stage))
        r["base_pose"].append(task.robot.pose)
        r["base_cmd"].append(task.robot.last_twist)
        r["joints_deg"].append(np.rad2deg(d.qpos[self._joint_adr]))
        r["joints_cmd_deg"].append([task.arm_cmd[j] for j in ARM_JOINTS])
        r["item_pos"].append(task.item_pos(self.item))
        r["jaw_centre"].append(task.jaw_centre())
        if self.images:
            self._save_frames(task)

    def _save_frames(self, task: P.Task) -> None:
        from PIL import Image

        if self._renderer is None:
            self._renderer = mujoco.Renderer(task.model, *IMAGE_SIZE)
        out = self.dir / "frames" / "wrist"
        out.mkdir(parents=True, exist_ok=True)
        self._renderer.update_scene(task.data, camera="wrist_cam")
        Image.fromarray(self._renderer.render()).save(out / f"{self.n_frames:05d}.jpg", quality=85)
        self.n_frames += 1

    def finish(self, ok: bool, sim_seconds: float, wall_seconds: float, log: list[str]) -> dict:
        self.dir.mkdir(parents=True, exist_ok=True)
        arrays = {k: np.asarray(v, dtype=np.float32 if k != "stage" else np.int16) for k, v in self.rows.items()}
        np.savez_compressed(self.dir / "trajectory.npz", **arrays, stage_names=np.array(self.stages), joint_names=np.array(ARM_JOINTS))
        times = list(self.stage_t0.items()) + [("end", self.rows["t"][-1] if self.rows["t"] else 0.0)]
        stage_durations = {a: round(b[1] - a_t, 2) for (a, a_t), b in zip(times, times[1:])}
        meta = {
            "id": self.id,
            "created": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "scenario": self.scenario.name,
            "title": self.scenario.title,
            "params": self.params.as_dict(),
            "item": self.item,
            "success": bool(ok),
            "sim_seconds": round(sim_seconds, 2),
            "wall_seconds": round(wall_seconds, 2),
            "stages": stage_durations,
            "samples": len(self.rows["t"]),
            "sample_hz": self.hz,
            "frames": self.n_frames,
            "notes": self.task.notes,
            "constants": {
                "gripper_open_deg": P.GRIPPER_OPEN_DEG,
                "gripper_release_deg": P.GRIPPER_RELEASE_DEG,
                "gripper_closed_deg": P.GRIPPER_CLOSED_DEG,
                "pinch_z_m": P.PINCH_Z,
                "grasp_roll_deg": P.GRASP_ROLL,
                "grasp_z_off_m": S.GRASP_Z_OFF,
                "grasp_y_off_m": S.GRASP_Y_OFF,
                "mouth_gap_m": S.MOUTH_GAP,
                "advance_m": S.ADVANCE,
                "carry_reach_m": S.CARRY_REACH,
                "carry_height_m": S.CARRY_HEIGHT,
            },
            "versions": {"mujoco": mujoco.__version__, "robot_model": _sha(R.PARTS_JSON), "layout": _sha(DEFAULT_LAYOUT), "inventory": _sha(DEFAULT_INVENTORY)},
            "log": log,
        }
        (self.dir / "meta.json").write_text(json.dumps(meta, indent=1))
        line = {k: meta[k] for k in ("id", "created", "scenario", "item", "params", "success", "sim_seconds", "wall_seconds")}
        line["notes"] = meta["notes"]
        RUNS_FILE.parent.mkdir(parents=True, exist_ok=True)
        with RUNS_FILE.open("a") as fh:
            fh.write(json.dumps(line) + "\n")
        return meta


def run_recorded(task: P.Task, scenario: S.Scenario, params: W.Params, record_images: bool = True) -> bool:
    """Run `scenario` on `task` while recording it into the data folder. Returns the run's success."""
    rec = Recorder(task, scenario, params, images=record_images)
    rec.attach()
    log: list[str] = []
    wall0, sim0 = time.time(), task.data.time
    ok = False
    try:
        with contextlib.redirect_stdout(_Splitter(sys.stdout, log)):
            ok = scenario.run(task, params)
    finally:
        rec.detach()
        meta = rec.finish(ok, task.data.time - sim0, time.time() - wall0, log)
        print(f"[data] saved {meta['id']} ({'success' if ok else 'failed'}) -> {rec.dir}")
    return bool(ok)


# ---------------------------------------------------------------- calibration export
def export_calibration(model: mujoco.MjModel | None = None, path: Path = CALIBRATION_FILE) -> Path:
    """Write everything about the simulated robot and scene that a real build has to match."""
    if model is None:
        from dsr_sim.constants import DEFAULT_SCENE

        model = mujoco.MjModel.from_xml_path(str(DEFAULT_SCENE))
    joints = []
    for j in ARM_JOINTS:
        jid = model.joint(R.ARM_PREFIX + j).id
        aid = model.actuator(R.ARM_PREFIX + j).id
        lo, hi = np.rad2deg(model.jnt_range[jid])
        joints.append(
            {
                "name": j,
                "lerobot_key": f"{j}.pos",
                "range_deg": [round(float(lo), 1), round(float(hi), 1)],
                "actuator": {
                    "type": "position",
                    "kp": float(model.actuator_gainprm[aid, 0]),
                    "kv": float(-model.actuator_biasprm[aid, 2]),
                    "torque_limit_Nm": [float(v) for v in model.actuator_forcerange[aid]],
                },
            }
        )
    levels = [[W.rack_pos(lv, sl).round(4).tolist() for sl in range(W.SLOTS)] for lv in range(W.LEVELS)]
    cams = {}
    for name in ("wrist_cam",):
        cid = model.camera(name).id
        cams[name] = {"fovy_deg": float(model.cam_fovy[cid]), "pos_in_parent_m": [round(float(v), 4) for v in model.cam_pos[cid]], "parent_body": model.body(model.cam_bodyid[cid]).name}
    item_bodies = [i for i in range(model.nbody) if model.body(i).name == f"item_{W.ITEM}/object"]
    item_mass = float(model.body_mass[item_bodies[0]]) if item_bodies else None
    doc = {
        "generated": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "mujoco": mujoco.__version__,
        "notes": "Simulation values. Treat as the starting point for the real robot, to be verified on hardware.",
        "robot": {
            "rover": {
                "type": "4-wheel mecanum, holonomic planar drive (wheel slip not modelled)",
                "wheel_radius_m": R.WHEEL_RADIUS,
                "half_wheelbase_m": R.HALF_WHEELBASE,
                "half_track_m": R.HALF_TRACK,
                "deck_top_z_m": R.DECK_TOP,
                "arm_mount_xyz_m": list(R.ARM_MOUNT),
                "max_linear_mps": R.MAX_LINEAR,
                "max_angular_radps": R.MAX_ANGULAR,
                "footprint_half_extents_m": list(W.ROVER_HALF),
            },
            "arm": {"model": "SO-101 (SO-ARM100 MJCF, new calibration: zero = middle of range)", "joints": joints},
            "cameras": cams,
        },
        "control": {
            "cruise_speed_mps": 0.35,
            "acceleration_mps2": 0.8,
            "angular_acceleration_radps2": 3.0,
            "poses_deg": {k: dict(zip(ARM_JOINTS, v)) for k, v in R.POSES_DEG.items()},
            "gripper_deg": {"open": P.GRIPPER_OPEN_DEG, "release": P.GRIPPER_RELEASE_DEG, "rack_release": S.RACK_RELEASE_DEG, "closed": P.GRIPPER_CLOSED_DEG},
        },
        "manipulation": {
            "approach": "jaws level and pointing along the approach heading, wrist roll -90 deg (the real arm reads about -92 when picking) so the jaws open sideways",
            "wrist_roll_deg": P.GRASP_ROLL,
            "pinch_centre_offset_from_gripperframe_m": P.PINCH_Z,
            "grasp_height_above_item_centre_m": S.GRASP_Z_OFF,
            "grasp_depth_past_item_centre_m": S.GRASP_Y_OFF,
            "mouth_gap_m": S.MOUTH_GAP,
            "drive_in_distance_m": S.ADVANCE,
            "shoulder_forward_of_rover_centre_m": P.SHOULDER_FORWARD,
            "stand_off_candidates_m": list(P.REACH_CANDIDATES),
            "stand_off_note": "shoulder-to-jaws distance at the rack mouth; the longest one the arm can reach level",
            "carry": {"reach_m": S.CARRY_REACH, "height_m": S.CARRY_HEIGHT},
            "ik": {"heading_weight": P.IK_ROT_WEIGHT, "accept_position_error_m": P.IK_OK_ERR},
        },
        "scene": {
            "table_size_m": list(W.LAYOUT.table.size),
            "table_top_z_m": 0.0,
            "shelf": {
                "kind": "cardboard cubby unit, open front",
                "width_m": W.LAYOUT.shelf.width,
                "depth_m": W.LAYOUT.shelf.depth,
                "height_m": W.LAYOUT.shelf.height,
                "levels": W.LEVELS,
                "slots_per_level": W.SLOTS,
                "board_top_heights_m": list(W.LAYOUT.shelf.board_heights),
                "rack_slot_xyz_m": levels,
                "note": "rack_slot_xyz_m[level][slot] = (x, y, top of that rack's floor); level 0 is the bottom",
            },
            "tray": {"pos_xy_m": list(W.LAYOUT.tray.pos), "size_xyz_m": list(W.LAYOUT.tray.size)},
            "dock": {"pos_xy_m": list(W.DOCK[:2])},
        },
        "item": {"name": W.ITEM, "radius_m": W.ITEM_RADIUS, "half_height_m": S.ITEM_HALF_HEIGHT, "mass_kg": item_mass, "friction": "default 1.0 sliding"},
        "sim_only_assumptions": [
            "items are held by contact and friction only (no weld, no ghost jaws); grip quality still depends on rigid-body contact, not real material compliance",
            "mecanum wheels slip and roller dynamics are not modelled",
            "camera images are rendered, not real sensor images",
            "arm collision geometry is a convex decomposition of the SO-101 CAD meshes; 7 link pairs that overlap in the folded rest pose have contact disabled",
        ],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(doc, indent=1))
    return path
