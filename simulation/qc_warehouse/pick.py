"""Run Stretch 3 through the customer orders: navigate, identify, pick, and pack.

The robot localizes and maps with SLAM (slam_toolbox in Docker) by default:
  uv run qcw-slam up                                        # once: start slam_toolbox + rosbridge
  uv run qcw-pick [--orders config/orders.yaml]             # headless, prints a report
  uv run mjpython -m qc_warehouse.pick --viewer [--realtime] # watch it live (macOS needs mjpython)
  uv run qcw-pick --localization ground-truth               # baseline: simulator pose + scene map
"""

from __future__ import annotations

import argparse
import contextlib
import json
import logging
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import mujoco

from qc_warehouse.autonomy.fulfill import OrderPicker, OrderResult
from qc_warehouse.autonomy.mapping import MapProvider, StaticMap
from qc_warehouse.autonomy.navigation import build_occupancy_grid
from qc_warehouse.autonomy.orders import load_orders
from qc_warehouse.autonomy.perception import HeadCamera
from qc_warehouse.autonomy.runner import SimRunner
from qc_warehouse.autonomy.slam_bridge import DEFAULT_ROSBRIDGE_URL, RosbridgeSlamBackend
from qc_warehouse.autonomy.slam_stack import SlamStack, attach_slam, room_bounds
from qc_warehouse.autonomy.stretch import Stretch
from qc_warehouse.autonomy.warehouse import Warehouse
from qc_warehouse.config import ConfigError, Layout, load_inventory, load_layout
from qc_warehouse.constants import CONFIG_DIR, DEFAULT_INVENTORY, DEFAULT_LAYOUT, DEFAULT_SCENE, PROJECT_ROOT
from qc_warehouse.map_image import map_image, write_png
from qc_warehouse.render import render_cameras
from qc_warehouse.slam_cli import compose

NAV_INFLATION = 0.34  # robot footprint radius incl. a tucked gripper/item


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--orders", type=Path, default=CONFIG_DIR / "orders.yaml")
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--layout", type=Path, default=DEFAULT_LAYOUT)
    parser.add_argument("--inventory", type=Path, default=DEFAULT_INVENTORY)
    parser.add_argument("--viewer", action="store_true", help="show the passive MuJoCo viewer (mjpython on macOS)")
    parser.add_argument("--realtime", action="store_true", help="pace the simulation at wall-clock speed")
    parser.add_argument("--report", type=Path, default=PROJECT_ROOT / "renders" / "pick_report.json")
    parser.add_argument("--snapshot-dir", type=Path, default=PROJECT_ROOT / "renders", help="final camera snapshots")
    parser.add_argument("--localization", choices=("slam", "ground-truth"), default="slam")
    parser.add_argument("--rosbridge", default=DEFAULT_ROSBRIDGE_URL, help="slam_toolbox rosbridge websocket")
    parser.add_argument("--no-slam-restart", action="store_true", help="reuse the running SLAM session instead of starting fresh")
    parser.add_argument("--map-image", type=Path, default=PROJECT_ROOT / "renders" / "slam_map.png")
    parser.add_argument("--seed", type=int, default=0, help="sensor-noise seed")
    return parser.parse_args(argv)


@dataclass
class Localization:
    maps: MapProvider
    stack: SlamStack | None = None

    def report(self) -> dict[str, Any]:
        return self.stack.report() if self.stack else {"mode": "ground-truth"}

    def close(self) -> None:
        if self.stack:
            self.stack.close()


def _setup_localization(args: argparse.Namespace, robot: Stretch, runner: SimRunner, layout: Layout) -> Localization:
    if args.localization == "ground-truth":
        return Localization(StaticMap(build_occupancy_grid(robot.model, robot.data, room_bounds(layout), NAV_INFLATION)))
    if not args.no_slam_restart:
        print("starting a fresh SLAM session (docker compose)...")
        compose("restart")
    backend = RosbridgeSlamBackend(args.rosbridge)
    try:
        stack = attach_slam(robot, runner, layout, backend, NAV_INFLATION, seed=args.seed)
    except Exception:
        backend.close()
        raise
    return Localization(stack.maps, stack)


def _save_map_image(loc: Localization, path: Path) -> None:
    if loc.stack is None:
        return
    grid = loc.stack.maps.current()
    try:
        write_png(path, map_image(loc.stack.maps.values, grid.origin, grid.resolution, loc.stack.monitor.trajectory))
    except OSError as exc:
        print(f"[warn] could not write the SLAM map image: {exc}", file=sys.stderr)
        return
    print(f"SLAM map: {path} (red = estimated path, green = true path)")


@contextlib.contextmanager
def _maybe_viewer(model: mujoco.MjModel, data: mujoco.MjData, enabled: bool):
    if not enabled:
        yield None
        return
    import mujoco.viewer

    try:
        viewer = mujoco.viewer.launch_passive(model, data)
    except RuntimeError as exc:
        raise RuntimeError(f"{exc}\nOn macOS run:  uv run mjpython -m qc_warehouse.pick --viewer") from exc
    with viewer:
        viewer.cam.lookat[:] = [0.0, 1.0, 0.6]
        viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 8.5, -90.0, -35.0
        yield viewer


def _print_report(results: tuple[OrderResult, ...], wall: float, sim: float) -> None:
    print("\n=== Fulfilment report ===")
    for order in results:
        print(f"{order.order_id} -> tote_{order.tote}: {'PACKED' if order.success else 'INCOMPLETE'}")
        for r in order.items:
            mark = "ok " if r.success else "ERR"
            print(f"   [{mark}] {r.item:16s} {r.grasp or '-':5s} {r.sim_seconds:6.1f}s  {r.detail}")
    picked = sum(r.success for o in results for r in o.items)
    total = sum(len(o.items) for o in results)
    print(f"{picked}/{total} items packed | sim time {sim:.0f}s | wall time {wall:.0f}s")


def _print_localization(report: dict[str, Any]) -> None:
    if report.get("mode") != "slam":
        print("localization: simulator ground truth")
        return
    err = report["error_vs_ground_truth"]
    print(
        f"SLAM localization error vs ground truth: mean {err['mean_xy_error'] * 100:.1f} cm, max {err['max_xy_error'] * 100:.1f} cm, "
        f"heading mean {err['mean_yaw_error_deg']:.2f} deg | scans {report['scans']['scans']}, processed {report['scans']['acks']}"
    )


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    sys.stdout.reconfigure(line_buffering=True)  # progress lines show up promptly when piped or logged
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    try:
        layout = load_layout(args.layout)
        inventory = load_inventory(args.inventory, layout)
        orders = load_orders(args.orders, inventory, max_orders=layout.station.num_totes)
        if not args.scene.exists():
            raise ConfigError(f"scene not found: {args.scene}. Run `uv run qcw-build`.")
        model = mujoco.MjModel.from_xml_path(str(args.scene))
    except (ConfigError, ValueError) as exc:
        print(f"[error] {exc}", file=sys.stderr)
        return 1

    data = mujoco.MjData(model)
    try:
        mujoco.mj_resetDataKeyframe(model, data, model.key("stow").id)
        mujoco.mj_forward(model, data)
        warehouse = Warehouse(model, data, layout, inventory)
        robot = Stretch(model, data)
    except (KeyError, ValueError) as exc:
        print(f"[error] scene does not match the configs ({exc}); rebuild with `uv run qcw-build`", file=sys.stderr)
        return 1

    started = time.perf_counter()
    camera = loc = None
    try:
        camera = HeadCamera(model, robot.head_camera)
        runner = SimRunner(model, data, realtime=args.realtime)
        loc = _setup_localization(args, robot, runner, layout)  # SLAM ready before the window opens
        with _maybe_viewer(model, data, args.viewer) as viewer:
            runner.viewer = viewer
            picker = OrderPicker(robot, runner, warehouse, loc.maps, camera, on_event=print, survey=loc.stack is not None)
            results = picker.fulfil(orders)
            report_loc = loc.report()
            _save_map_image(loc, args.map_image)
    except RuntimeError as exc:  # includes SLAM service / connection failures
        print(f"[error] {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\n[stopped] viewer closed or interrupted", file=sys.stderr)
        return 130
    finally:
        if camera is not None:
            camera.close()
        if loc is not None:
            loc.close()

    _print_report(results, time.perf_counter() - started, data.time)
    _print_localization(report_loc)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps({"orders": [asdict(r) for r in results], "localization": report_loc}, indent=2))
    render_cameras(model, data, ("packing_station_cam", "overview"), args.snapshot_dir)
    print(f"report: {args.report}  snapshots: {args.snapshot_dir}/packing_station_cam.png")
    return 0 if all(r.success for r in results) else 2


if __name__ == "__main__":
    sys.exit(main())
