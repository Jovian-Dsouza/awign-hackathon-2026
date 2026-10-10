"""Split the SO-101 collision meshes into convex pieces so MuJoCo's contacts follow the real part shapes.

MuJoCo collides a mesh as ONE convex hull. The arm's links are not convex (the gripper has a gap between its jaws,
the folded arm nests its links inside each other), so with raw meshes the hull fills the jaw opening and the
folded arm sits inside itself. This script writes assets/so101/collision/<mesh>_<n>.stl: convex pieces (CoACD)
that together follow the true shape. Run it once; the sim loads the pieces (see robot._convex_collision).

Not a runtime dependency. Run with:  uv run --with coacd --with trimesh --with scipy python scripts/decompose_collision.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import coacd
import numpy as np
import trimesh

ASSETS = Path(__file__).resolve().parents[1] / "assets" / "so101"
OUT = ASSETS / "collision"

# mesh -> (CoACD concavity threshold, max pieces). The gripper parts get the tightest fit: their opening is the job.
PARTS = {
    "moving_jaw_so101_v1": (0.02, 24),
    "wrist_roll_follower_so101_v1": (0.02, 24),
    "wrist_roll_pitch_so101_v2": (0.04, 14),
    "rotation_pitch_so101_v1": (0.04, 12),
    "upper_arm_so101_v1": (0.05, 10),
    "under_arm_so101_v1": (0.05, 10),
    "motor_holder_so101_base_v1": (0.05, 8),
    "motor_holder_so101_wrist_v1": (0.05, 8),
    "sts3215_03a_v1": (0.08, 4),
    "sts3215_03a_no_horn_v1": (0.08, 4),
}


def main(names: list[str]) -> int:
    OUT.mkdir(exist_ok=True)
    for name, (threshold, max_pieces) in PARTS.items():
        if names and name not in names:
            continue
        mesh = trimesh.load(ASSETS / "assets" / f"{name}.stl")
        cmesh = coacd.Mesh(mesh.vertices, mesh.faces)
        pieces = coacd.run_coacd(cmesh, threshold=threshold, max_convex_hull=max_pieces, preprocess_mode="auto", seed=0)
        for old in OUT.glob(f"{name}_*.stl"):
            old.unlink()
        hull_vol = 0.0
        for i, (verts, faces) in enumerate(pieces):
            piece = trimesh.Trimesh(verts, faces)
            piece.export(OUT / f"{name}_{i}.stl")
            hull_vol += piece.volume
        print(f"{name:32s} {len(pieces):2d} pieces, volume {hull_vol / mesh.convex_hull.volume:.2f} of the single hull (part itself: {mesh.volume / mesh.convex_hull.volume:.2f})")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
