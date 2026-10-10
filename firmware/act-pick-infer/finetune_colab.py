# -*- coding: utf-8 -*-
"""Fine-tune TakuyaHiraoka/act_so101_pick_diverse_objects on a private dataset.

Run this on Google Colab (GPU), not on the Pi. It installs LeRobot when needed,
logs in to Hugging Face, and launches ``lerobot-train`` from the base ACT
checkpoint onto your recorded wrist demos.

Colab cells (example)::

    !pip install -U "lerobot[feetech]"  # or: !pip install -e ".[feetech]" from a clone
    from huggingface_hub import login
    login()  # paste a write token

    # Upload this file, then:
    !python finetune_colab.py \\
      --dataset.repo_id YOUR_HF_USER/so101_pick_wrist_finetune \\
      --policy.repo_id YOUR_HF_USER/act_so101_pick_finetune

Or call with no args and set the env vars / edit DEFAULTS below. Extra CLI
flags are forwarded to ``lerobot-train``.

After training, run on the Pi::

    python act-pick-infer/infer.py --check --policy YOUR_HF_USER/act_so101_pick_finetune
"""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys


BASE_POLICY = "TakuyaHiraoka/act_so101_pick_diverse_objects"


def _which_train() -> str:
    path = shutil.which("lerobot-train")
    if path:
        return path
    # Fallback: module entry if the console script is missing.
    return None


def build_cmd(args: argparse.Namespace, passthrough: list[str]) -> list[str]:
    train = _which_train()
    if train:
        cmd = [train]
    else:
        cmd = [sys.executable, "-m", "lerobot.scripts.lerobot_train"]

    cmd += [
        f"--dataset.repo_id={args.dataset_repo_id}",
        f"--policy.path={args.policy_path}",
        f"--policy.repo_id={args.policy_repo_id}",
        f"--policy.device={args.device}",
        f"--policy.push_to_hub={'true' if args.push_to_hub else 'false'}",
        f"--policy.private={'true' if args.private else 'false'}",
        f"--batch_size={args.batch_size}",
        f"--steps={args.steps}",
        f"--save_freq={args.save_freq}",
        f"--num_workers={args.num_workers}",
        f"--wandb.enable={'true' if args.wandb else 'false'}",
        f"--output_dir={args.output_dir}",
        f"--job_name={args.job_name}",
    ]
    cmd += passthrough
    return cmd


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--dataset.repo_id",
        dest="dataset_repo_id",
        default=os.environ.get("FINETUNE_DATASET_REPO"),
        help="Private dataset with wrist demos (required)",
    )
    parser.add_argument(
        "--policy.path",
        dest="policy_path",
        default=BASE_POLICY,
        help="Base ACT checkpoint to fine-tune",
    )
    parser.add_argument(
        "--policy.repo_id",
        dest="policy_repo_id",
        default=os.environ.get("FINETUNE_POLICY_REPO"),
        help="Where to push the fine-tuned policy (required if push_to_hub)",
    )
    parser.add_argument("--policy.device", dest="device", default="cuda")
    parser.add_argument(
        "--policy.push_to_hub",
        dest="push_to_hub",
        type=lambda s: str(s).lower() in ("1", "true", "yes"),
        default=True,
    )
    parser.add_argument(
        "--policy.private",
        dest="private",
        type=lambda s: str(s).lower() in ("1", "true", "yes"),
        default=True,
    )
    parser.add_argument("--batch_size", type=int, default=8)
    parser.add_argument("--steps", type=int, default=10000)
    parser.add_argument("--save_freq", type=int, default=2000)
    parser.add_argument("--num_workers", type=int, default=2)
    parser.add_argument(
        "--wandb.enable",
        dest="wandb",
        type=lambda s: str(s).lower() in ("1", "true", "yes"),
        default=False,
    )
    parser.add_argument("--output_dir", default="outputs/train/act_so101_pick_finetune")
    parser.add_argument("--job_name", default="act_so101_pick_finetune")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the lerobot-train command and exit",
    )

    args, passthrough = parser.parse_known_args(argv)

    if not args.dataset_repo_id:
        parser.error("set --dataset.repo_id YOUR_HF_USER/so101_pick_wrist_finetune")
    if args.push_to_hub and not args.policy_repo_id:
        parser.error("set --policy.repo_id YOUR_HF_USER/act_so101_pick_finetune")

    cmd = build_cmd(args, passthrough)
    print("Fine-tune command:")
    print("  " + " \\\n    ".join(cmd))
    if args.dry_run:
        return 0

    # Hugging Face login is interactive; Colab users should call login() first.
    if os.environ.get("HF_TOKEN") or os.environ.get("HUGGING_FACE_HUB_TOKEN"):
        print("HF token found in environment")
    else:
        print("Tip: run `huggingface-cli login` or `from huggingface_hub import login; login()` first")

    return subprocess.call(cmd)


if __name__ == "__main__":
    raise SystemExit(main())
