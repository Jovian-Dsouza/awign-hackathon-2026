"""Offline check of a trained ACT checkpoint on held-out simulated episodes (no simulator needed).

For every frame of the held-out episodes the policy gets the wrist image and joint state and predicts the next action;
we compare with the recorded action. Baselines: "hold the current pose" and "predict the training-set mean action".
This says whether the policy learned the motion from images + state. It does NOT say it can pick in closed loop.

  python data/tools/eval_act_offline.py --checkpoint outputs/act_dark_store/checkpoints/last/pretrained_model --first-episode 40
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import torch

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors

NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--checkpoint", type=Path, required=True)
    ap.add_argument("--dataset-root", type=Path, default=root / "lerobot" / "dark_store_so101")
    ap.add_argument("--first-episode", type=int, default=40, help="held-out episodes are first-episode ... end")
    ap.add_argument("--stride", type=int, default=5, help="evaluate every n-th frame")
    ap.add_argument("--device", default="mps" if torch.backends.mps.is_available() else "cpu")
    args = ap.parse_args()

    probe = LeRobotDataset("local/dark_store_so101", root=args.dataset_root)
    held = list(range(args.first_episode, probe.num_episodes))
    train_eps = list(range(args.first_episode))
    ds = LeRobotDataset("local/dark_store_so101", root=args.dataset_root, episodes=held)
    train_actions = np.asarray(probe.hf_dataset.with_format("numpy")["action"])
    ep_idx = np.asarray(probe.hf_dataset["episode_index"])
    mean_action = train_actions[np.isin(ep_idx, train_eps)].mean(0)

    policy = ACTPolicy.from_pretrained(args.checkpoint).to(args.device).eval()
    pre, post = make_pre_post_processors(policy.config, pretrained_path=str(args.checkpoint))

    errs, base_hold, base_mean = [], [], []
    last_ep = None
    for i in range(0, len(ds), args.stride):
        item = ds[i]
        if item["episode_index"] != last_ep:
            policy.reset()
            last_ep = item["episode_index"]
        batch = {k: v.unsqueeze(0) for k, v in item.items() if k.startswith("observation")}
        with torch.no_grad():
            pred = post(policy.select_action(pre(batch))).squeeze(0).cpu().numpy()
        gt = item["action"].numpy()
        errs.append(np.abs(pred - gt))
        base_hold.append(np.abs(item["observation.state"].numpy() - gt))
        base_mean.append(np.abs(mean_action - gt))
    f = lambda a: np.mean(a, axis=0)
    out = {
        "frames": len(errs),
        "held_out_episodes": held,
        "mae_per_joint": dict(zip(NAMES, map(float, f(errs)))),
        "mae_mean": float(np.mean(errs)),
        "baseline_hold_pose_mae": float(np.mean(base_hold)),
        "baseline_mean_action_mae": float(np.mean(base_mean)),
    }
    print(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
