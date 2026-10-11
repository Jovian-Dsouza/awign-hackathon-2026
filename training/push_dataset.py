"""Upload a locally recorded LeRobot dataset to the Hub so Colab can train on it.

The recording app saves datasets under ``~/.cache/huggingface/lerobot/<user>/<name>_<timestamp>``.
This script checks that the dataset matches the base ACT checkpoint's inputs (wrist camera,
6-D state/action), then pushes it with ``LeRobotDataset.push_to_hub`` so the ``v3.0`` version
tag that ``lerobot-train`` requires is created.

Run from the lerobot conda env on the Mac:

    conda activate lerobot
    python training/push_dataset.py \\
        --root ~/.cache/huggingface/lerobot/Jovian-Dsouza/so101_pick_test1_20261010_213115 \\
        --repo-id Jovian-Dsouza/so101_pick_test1

Add ``--dry-run`` to only validate.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from huggingface_hub import hf_hub_download

BASE_POLICY = "TakuyaHiraoka/act_so101_pick_diverse_objects"
MIN_EPISODES = 1


def load_info(root: Path) -> dict:
    info_path = root / "meta" / "info.json"
    if not info_path.is_file():
        raise FileNotFoundError(f"{info_path} not found - is --root a LeRobot dataset folder?")
    return json.loads(info_path.read_text())


def base_policy_features(policy_repo: str) -> dict[str, list[int]]:
    config = json.loads(Path(hf_hub_download(policy_repo, "config.json")).read_text())
    features = {**config["input_features"], **config["output_features"]}
    return {key: feat["shape"] for key, feat in features.items()}


def to_chw(shape: list[int]) -> list[int]:
    """Dataset stores video frames as HWC; the policy config uses CHW."""
    return [shape[2], shape[0], shape[1]] if len(shape) == 3 else shape


def check_compatibility(info: dict, expected: dict[str, list[int]]) -> list[str]:
    features = info["features"]
    errors = []
    if info["total_episodes"] < MIN_EPISODES:
        errors.append("dataset has no episodes")
    for key, shape in expected.items():
        if key not in features:
            errors.append(f"missing feature {key!r} (base policy expects it)")
            continue
        got = to_chw(features[key]["shape"]) if key.startswith("observation.images.") else features[key]["shape"]
        if list(got) != list(shape):
            errors.append(f"{key}: dataset shape {got} != base policy shape {shape}")
    return errors


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--root", type=Path, required=True, help="Local dataset folder")
    parser.add_argument("--repo-id", required=True, help="Hub dataset id, e.g. Jovian-Dsouza/so101_pick_test1")
    parser.add_argument("--base-policy", default=BASE_POLICY)
    parser.add_argument("--public", action="store_true", help="Create a public repo (default: private)")
    parser.add_argument("--dry-run", action="store_true", help="Validate only, do not upload")
    args = parser.parse_args(argv)

    root = args.root.expanduser().resolve()
    info = load_info(root)
    print(f"{root.name}: {info['total_episodes']} episodes, {info['total_frames']} frames @ {info['fps']} fps")

    errors = check_compatibility(info, base_policy_features(args.base_policy))
    if errors:
        print("Dataset is not compatible with the base policy:", *errors, sep="\n  - ", file=sys.stderr)
        return 1
    print(f"Features match {args.base_policy}")

    if args.dry_run:
        return 0

    # Imported late so --dry-run works without the full lerobot install.
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    dataset = LeRobotDataset(args.repo_id, root=root)
    dataset.push_to_hub(private=not args.public, tags=["so101", "act", "finetune"])
    print(f"Pushed: https://huggingface.co/datasets/{args.repo_id}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
