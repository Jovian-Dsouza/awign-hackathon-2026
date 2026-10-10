"""Generate many varied, recorded episodes into the data folder (a training/validation set for the real robot).

Usage:
  uv run dsr-collect --task restock_misplaced --episodes 20
  uv run dsr-collect --task shelf_to_tray --episodes 9 --sweep          # every rack once
  uv run dsr-collect --task restock_misplaced --episodes 10 --no-images # trajectories only (much faster)
  uv run dsr-collect --episodes 10 --no-assist                          # pure friction grasps
"""

from __future__ import annotations

import argparse
import itertools
import sys
from pathlib import Path

import mujoco
import numpy as np

from dsr_sim import datalog
from dsr_sim.constants import DEFAULT_SCENE
from dsr_sim.pick import Task
from dsr_sim.run import reset_scene
from dsr_sim.scenarios import SCENARIOS, resolve
from dsr_sim.world import LEVELS, SLOTS, TABLE, Params, home_rack, item_position_problem, occupied_racks, validate


def _random_item_xy(rng: np.random.Generator) -> tuple[float, float]:
    for _ in range(1000):
        x = float(rng.uniform(TABLE.x0, TABLE.x1))
        y = float(rng.uniform(TABLE.y0, TABLE.y1))
        if item_position_problem(x, y) is None:
            return round(x, 3), round(y, 3)
    raise RuntimeError("could not find a valid item position")


def make_params(task: str, i: int, rng: np.random.Generator, sweep: bool, assist: bool) -> Params:
    if task == "shelf_to_tray":
        racks = list(itertools.product(range(LEVELS), range(SLOTS)))
        rack = racks[i % len(racks)] if sweep else racks[int(rng.integers(len(racks)))]
        return Params(rack=rack, assist=assist)
    free = [r for r in itertools.product(range(LEVELS), range(SLOTS)) if r not in occupied_racks()]
    dest = home_rack() if rng.random() < 0.5 else free[int(rng.integers(len(free)))]
    return Params(item_xy=_random_item_xy(rng), dest_rack=None if dest == home_rack() else dest, assist=assist)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--task", default="restock_misplaced", help="simulation name or number")
    parser.add_argument("--episodes", type=int, default=10)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--sweep", action="store_true", help="shelf_to_tray: visit every rack in order instead of random racks")
    parser.add_argument("--no-images", action="store_true", help="skip camera frames (faster, much smaller)")
    parser.add_argument("--no-assist", action="store_true", help="pure friction grasps (grasp assist off)")
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    args = parser.parse_args(argv)

    scenario = resolve(args.task)
    model = mujoco.MjModel.from_xml_path(str(args.scene))
    data = mujoco.MjData(model)
    rng = np.random.default_rng(args.seed)
    datalog.export_calibration(model)
    ok_count = 0
    for i in range(args.episodes):
        params = make_params(scenario.name, i, rng, args.sweep, assist=not args.no_assist)
        try:
            validate(scenario.name, params)
        except Exception as exc:  # noqa: BLE001
            print(f"[{i + 1}/{args.episodes}] skipped: {exc}")
            continue
        reset_scene(model, data, scenario, params)
        task = Task(model, data)
        task.run(0.5)
        print(f"[{i + 1}/{args.episodes}] {scenario.name} {params.as_dict()}")
        ok = datalog.run_recorded(task, scenario, params, record_images=not args.no_images)
        ok_count += int(ok)
    print(f"done: {ok_count}/{args.episodes} successful. Data in {datalog.DATA_DIR}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
