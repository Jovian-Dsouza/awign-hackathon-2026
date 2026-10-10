"""Run one of the simulations, with a selector.

Usage:
  uv run mjpython -m dsr_sim.run                 # menu, then watch it in the viewer
  uv run mjpython -m dsr_sim.run --task 2        # by number or name (restock_misplaced)
  uv run dsr-run --list                          # what is available
  uv run dsr-run --task shelf_to_tray --headless # no window, prints the result

Viewer keys: Space pause, R/Backspace reset (then Space to play), N next simulation,
] [ speed, = - zoom, 1 2 3 4 camera views.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import mujoco

from dsr_sim.constants import DEFAULT_SCENE
from dsr_sim.pick import VIEWS, Closed, Restart, RunControl, Task, _viewer_keys
from dsr_sim.scenarios import SCENARIOS, Scenario, resolve
from dsr_sim.world import ParamError, Params, validate


def print_menu() -> None:
    print("\nSimulations:")
    for i, sc in enumerate(SCENARIOS.values(), start=1):
        print(f"  {i}) {sc.name:18s} {sc.title}\n     {sc.description}")
    print()


def choose_interactively() -> Scenario:
    print_menu()
    names = list(SCENARIOS)
    while True:
        try:
            raw = input(f"Which one? [1-{len(names)} or name, default 1]: ").strip() or "1"
        except EOFError:
            return SCENARIOS[names[0]]
        try:
            return resolve(raw)
        except KeyError as exc:
            print(exc)


def reset_scene(model: mujoco.MjModel, data: mujoco.MjData, scenario: Scenario, params: Params | None = None) -> None:
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_forward(model, data)
    if scenario.setup is not None:
        scenario.setup(model, data, params or Params())
        for _ in range(250):  # let the relocated item settle
            mujoco.mj_step(model, data)
        data.time = 0.0
        mujoco.mj_forward(model, data)


def run_headless(model, data, scenario: Scenario, frames_dir: Path | None, params: Params | None = None, record: bool = False) -> bool:
    params = params or Params()
    validate(scenario.name, params)
    reset_scene(model, data, scenario, params)
    task = Task(model, data, None, 1.0, frames_dir)
    task.run(0.5)
    print(f"== {scenario.title} ==")
    if record:
        from dsr_sim import datalog

        return datalog.run_recorded(task, scenario, params, record_images=True)
    return scenario.run(task, params)


def run_viewer(model, data, scenario: Scenario, args) -> bool:
    from mujoco import viewer as mj_viewer

    names = list(SCENARIOS)
    control = RunControl(paused=args.start_paused, speed=args.speed)
    holder: dict = {}
    ok = False
    print(__doc__.split("Viewer keys:", 1)[1].strip())
    with mj_viewer.launch_passive(model, data, key_callback=_viewer_keys(holder, control)) as viewer:
        holder["viewer"] = viewer
        while viewer.is_running():
            if control.next_requested:
                scenario = SCENARIOS[names[(names.index(scenario.name) + 1) % len(names)]]
                control.next_requested = False
                print(f"[simulation] {scenario.title}")
            control.reset_requested = False
            reset_scene(model, data, scenario, args.params)
            viewer.cam.azimuth, viewer.cam.elevation, viewer.cam.distance = VIEWS["1"][:3]
            viewer.cam.lookat[:] = VIEWS["1"][3]
            viewer.sync()
            task = Task(model, data, viewer, control.speed, args.frames_dir, control)
            try:
                task.run(0.5)
                print(f"== {scenario.title} ==")
                ok = scenario.run(task, args.params)
                print("finished. R or Backspace to reset (then Space to play), N for the next simulation, close the window to exit.")
                while viewer.is_running() and not control.reset_requested:
                    viewer.sync()
                    time.sleep(0.03)
            except Restart:
                pass
            except Closed:
                break
            if control.reset_requested:
                control.paused = True
                print("[reset] press Space to play")
    return ok


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--task", help="simulation name or number (see --list); omit for a menu")
    parser.add_argument("--list", action="store_true", help="list the simulations and exit")
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--headless", action="store_true", help="no window; print the result")
    parser.add_argument("--speed", type=float, default=1.0, help="viewer playback speed vs real time")
    parser.add_argument("--start-paused", action="store_true", help="open the viewer paused; press Space to start")
    parser.add_argument("--frames-dir", type=Path, default=None, help="save overview/wrist snapshots per stage")
    parser.add_argument("--rack", nargs=2, type=int, metavar=("LEVEL", "SLOT"), help="shelf_to_tray: rack to pick from (level 0 = bottom)")
    parser.add_argument("--item-xy", nargs=2, type=float, metavar=("X", "Y"), help="restock_misplaced: where the item is left on the table (metres)")
    parser.add_argument("--dest-rack", nargs=2, type=int, metavar=("LEVEL", "SLOT"), help="restock_misplaced: rack to put it in (default: its home rack)")
    parser.add_argument("--record", action="store_true", help="headless only: save the run into the data folder")
    args = parser.parse_args(argv)
    args.params = Params.from_dict({"rack": args.rack, "item_xy": args.item_xy, "dest_rack": args.dest_rack})

    if args.list:
        print_menu()
        return 0
    scenario = resolve(args.task) if args.task else (SCENARIOS["shelf_to_tray"] if args.headless else choose_interactively())

    model = mujoco.MjModel.from_xml_path(str(args.scene))
    data = mujoco.MjData(model)
    if args.headless:
        try:
            ok = run_headless(model, data, scenario, args.frames_dir, args.params, args.record)
        except ParamError as exc:
            print(f"[error] {exc}", file=sys.stderr)
            return 2
        return 0 if ok else 1
    return 0 if run_viewer(model, data, scenario, args) else 1


if __name__ == "__main__":
    sys.exit(main())
