"""Convert the recorded simulation episodes in data/episodes/ into a LeRobotDataset (v3 format) for ACT training.

Needs numpy, Pillow and lerobot (and ffmpeg for the videos); it does NOT need MuJoCo:

  uv run --with 'lerobot[dataset]' python data/tools/export_lerobot.py
  uv run --with 'lerobot[dataset]' python data/tools/export_lerobot.py --out data/lerobot/arm_only --arm-only
  uv run --with 'lerobot[dataset]' python data/tools/export_lerobot.py --assist off --include-failed

What goes in each frame (10 Hz, like the recording):
  observation.images.wrist    240x320 RGB video, the gripper camera
  observation.images.chassis  240x320 RGB video, the front-of-chassis camera
  observation.state           6 arm joints in degrees (shoulder_pan ... gripper), as measured
  action                      6 commanded arm joint targets in degrees  (+ 3 rover velocities vx, vy, wz unless --arm-only)
  task                        a sentence describing what this episode does

Only episodes that were recorded WITH camera frames are exported (the others have no images).
Ground-truth item position and the grasp-assist weld flag are deliberately left out: they do not exist on
the real robot. By default only successful episodes are included.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
IMAGE_SHAPE = (240, 320, 3)


def task_text(meta: dict) -> str:
    p, name = meta["params"], meta["scenario"]
    if name == "shelf_to_tray":
        lv, sl = p["rack"]
        return f"pick up the soup can from rack level {lv} slot {sl} and put it in the tray"
    x, y = p["item_xy"]
    dest = p.get("dest_rack") or [0, 2]
    return f"pick up the soup can from the table at x {x:.2f} y {y:.2f} and put it back in rack level {dest[0]} slot {dest[1]}"


def load_episode(ep: Path):
    meta = json.loads((ep / "meta.json").read_text())
    traj = np.load(ep / "trajectory.npz")
    return meta, traj


def main(argv: list[str] | None = None) -> int:
    here = Path(__file__).resolve().parent
    data_dir = here.parent
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data-dir", type=Path, default=data_dir)
    ap.add_argument("--out", type=Path, default=data_dir / "lerobot" / "dark_store_so101")
    ap.add_argument("--repo-id", default="local/dark_store_so101")
    ap.add_argument("--scenario", choices=["shelf_to_tray", "restock_misplaced"], help="only this simulation")
    ap.add_argument("--assist", choices=["on", "off", "any"], default="any", help="grasp-assist mode to include")
    ap.add_argument("--include-failed", action="store_true", help="also export unsuccessful episodes")
    ap.add_argument("--arm-only", action="store_true", help="action = 6 arm joints only (no rover velocities)")
    ap.add_argument("--no-videos", action="store_true", help="store frames as images instead of mp4")
    args = ap.parse_args(argv)

    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ImportError as exc:
        print(f"lerobot is not available: {exc}\nRun:  uv run --with 'lerobot[dataset]' python data/tools/export_lerobot.py", file=sys.stderr)
        return 1

    eps = sorted((args.data_dir / "episodes").glob("*/meta.json"))
    chosen, skipped = [], []
    for meta_path in eps:
        meta = json.loads(meta_path.read_text())
        why = None
        if not (meta_path.parent / "frames" / "wrist").exists() or meta.get("frames", 0) == 0:
            why = "no camera frames"
        elif args.scenario and meta["scenario"] != args.scenario:
            why = "other simulation"
        elif args.assist != "any" and meta.get("grasp_assist") != (args.assist == "on"):
            why = "other assist mode"
        elif not meta["success"] and not args.include_failed:
            why = "unsuccessful"
        (skipped if why else chosen).append((meta_path.parent, why))
    if not chosen:
        print("no episodes match; nothing to export.", file=sys.stderr)
        for ep, why in skipped:
            print(f"  skipped {ep.name}: {why}", file=sys.stderr)
        return 1

    fps = int(round(json.loads((chosen[0][0] / "meta.json").read_text())["sample_hz"]))
    action_names = [f"{j}.pos" for j in JOINTS] + ([] if args.arm_only else ["base_vx.vel", "base_vy.vel", "base_wz.vel"])
    features = {
        "observation.images.wrist": {"dtype": "image" if args.no_videos else "video", "shape": IMAGE_SHAPE, "names": ["height", "width", "channels"]},
        "observation.images.chassis": {"dtype": "image" if args.no_videos else "video", "shape": IMAGE_SHAPE, "names": ["height", "width", "channels"]},
        "observation.state": {"dtype": "float32", "shape": (6,), "names": [f"{j}.pos" for j in JOINTS]},
        "action": {"dtype": "float32", "shape": (len(action_names),), "names": action_names},
    }
    if args.out.exists():
        print(f"[error] {args.out} already exists; choose another --out or delete it first.", file=sys.stderr)
        return 1
    ds = LeRobotDataset.create(
        repo_id=args.repo_id, fps=fps, features=features, root=args.out, robot_type="so101_mecanum_sim", use_videos=not args.no_videos
    )

    report = {"fps": fps, "action_names": action_names, "episodes": [], "skipped": [{"id": e.name, "why": w} for e, w in skipped]}
    for ep_dir, _ in chosen:
        meta, traj = load_episode(ep_dir)
        n = int(len(traj["t"]))
        wrist = sorted((ep_dir / "frames" / "wrist").glob("*.jpg"))
        chassis = sorted((ep_dir / "frames" / "chassis").glob("*.jpg"))
        n = min(n, len(wrist), len(chassis))
        state = traj["joints_deg"][:n].astype(np.float32)
        action = traj["joints_cmd_deg"][:n].astype(np.float32)
        if not args.arm_only:
            action = np.concatenate([action, traj["base_cmd"][:n].astype(np.float32)], axis=1)
        task = task_text(meta)
        for i in range(n):
            ds.add_frame(
                {
                    "observation.images.wrist": np.asarray(Image.open(wrist[i]).convert("RGB")),
                    "observation.images.chassis": np.asarray(Image.open(chassis[i]).convert("RGB")),
                    "observation.state": state[i],
                    "action": action[i],
                    "task": task,
                }
            )
        ds.save_episode()
        report["episodes"].append({"id": meta["id"], "scenario": meta["scenario"], "frames": n, "success": meta["success"], "task": task})
        print(f"  exported {meta['id']}: {n} frames")
    ds.finalize()
    (args.out / "export_report.json").write_text(json.dumps(report, indent=1))
    print(f"\nLeRobotDataset written to {args.out}  ({len(chosen)} episodes, {sum(e['frames'] for e in report['episodes'])} frames, {fps} fps)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
