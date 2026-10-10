"""Write recorded simulation episodes as trajectory files the Pi's web panel can play back.

The panel (firmware/combined-web/web_combined.py) records and plays JSON lists of {"t": seconds, "pos": {joint: value}}
from ~/.so101_arm/trajs/<name>.json, in LeRobot units (body joints in degrees, gripper 0..100). This writes the same
format from data/episodes/*/trajectory.npz, so a simulated pick can be replayed slowly on the real arm to check limits
and clearances before any policy runs:

  python data/tools/export_pi_trajectories.py            # -> data/pi_trajectories/<episode id>.json
  scp data/pi_trajectories/<id>.json pi@raspberrypi.local:~/.so101_arm/trajs/

Only the arm is covered (the panel drives the base separately). Replay it with the arm clear of obstacles.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
GRIPPER_DEG_RANGE = (-10.0, 100.0)


def main() -> int:
    data_dir = Path(__file__).resolve().parent.parent
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data-dir", type=Path, default=data_dir)
    ap.add_argument("--out", type=Path, default=data_dir / "pi_trajectories")
    ap.add_argument("--include-failed", action="store_true")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    n = 0
    lo, hi = GRIPPER_DEG_RANGE
    for meta_path in sorted((args.data_dir / "episodes").glob("*/meta.json")):
        meta = json.loads(meta_path.read_text())
        if not meta["success"] and not args.include_failed:
            continue
        traj = np.load(meta_path.parent / "trajectory.npz")
        pos = traj["joints_deg"].astype(float)
        pos[:, 5] = (pos[:, 5] - lo) / (hi - lo) * 100.0
        frames = [{"t": round(float(t - traj["t"][0]), 3), "pos": {j: round(float(v), 1) for j, v in zip(JOINTS, row)}} for t, row in zip(traj["t"], pos)]
        (args.out / f"sim_{meta['id']}.json").write_text(json.dumps(frames))
        n += 1
    print(f"wrote {n} trajectories to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
