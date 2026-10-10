"""Compose the full warehouse scene and write it to an MJCF file.

Usage:  uv run python -m qc_warehouse.build_scene [--layout L] [--inventory I] [--out OUT]
"""

from __future__ import annotations

import argparse
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco

from qc_warehouse.config import ConfigError, Item, Layout, load_inventory, load_layout
from qc_warehouse.constants import (
    DEFAULT_INVENTORY,
    DEFAULT_LAYOUT,
    DEFAULT_SCENE,
    IMPRATIO,
    NOSLIP_ITERATIONS,
    THIRD_PARTY_DIR,
    TIMESTEP,
)
from qc_warehouse.items import AssetError, add_items
from qc_warehouse.materials import add_materials
from qc_warehouse.robot import add_stow_keyframe, add_stretch
from qc_warehouse.room import add_room
from qc_warehouse.shelf import add_shelf_unit
from qc_warehouse.station import add_packing_station, add_robot_dock


def build_spec(
    layout: Layout, inventory: tuple[Item, ...], asset_root: Path = THIRD_PARTY_DIR, with_robot: bool = True
) -> mujoco.MjSpec:
    """Build the in-memory spec. Third-party assets are loaded with absolute paths."""
    spec = mujoco.MjSpec()
    spec.modelname = "quickcommerce_warehouse"
    spec.option.timestep = TIMESTEP
    spec.option.impratio = IMPRATIO
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.option.noslip_iterations = NOSLIP_ITERATIONS  # keeps grasped items from slipping

    add_materials(spec)
    add_room(spec, layout)
    for placement in layout.shelves:
        add_shelf_unit(spec, layout.shelf, placement)
    add_packing_station(spec, layout.station)
    add_robot_dock(spec, layout.robot_dock)
    add_items(spec, inventory, layout, asset_root)
    if with_robot:
        add_stretch(spec, layout.robot_dock, asset_root / "stretch3" / "stretch.xml")
        add_stow_keyframe(spec, spec.compile())
    return spec


def _relativize_file_paths(xml: str, out_dir: Path) -> str:
    """Rewrite absolute asset `file=` paths relative to the scene file so it stays portable."""
    root = ET.fromstring(xml)
    for element in root.iter():
        path = element.get("file")
        if path and os.path.isabs(path):
            element.set("file", os.path.relpath(path, out_dir))
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode") + "\n"


def build_scene_xml(
    layout: Layout, inventory: tuple[Item, ...], out_path: Path, asset_root: Path = THIRD_PARTY_DIR
) -> Path:
    """Build, validate (compile) and write the scene. Returns the output path."""
    out_path = Path(out_path).resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    spec = build_spec(layout, inventory, asset_root)
    xml = _relativize_file_paths(spec.to_xml(), out_path.parent)
    # Validate a temp copy (same dir so relative paths resolve) before replacing the old scene.
    tmp_path = out_path.with_suffix(".tmp.xml")
    tmp_path.write_text(xml)
    try:
        mujoco.MjModel.from_xml_path(str(tmp_path))
        os.replace(tmp_path, out_path)
    finally:
        tmp_path.unlink(missing_ok=True)
    return out_path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--layout", type=Path, default=DEFAULT_LAYOUT)
    parser.add_argument("--inventory", type=Path, default=DEFAULT_INVENTORY)
    parser.add_argument("--out", type=Path, default=DEFAULT_SCENE)
    args = parser.parse_args(argv)

    try:
        layout = load_layout(args.layout)
        inventory = load_inventory(args.inventory, layout)
        out = build_scene_xml(layout, inventory, args.out)
    except (ConfigError, AssetError, ValueError) as exc:
        print(f"[error] {exc}", file=sys.stderr)
        return 1
    print(f"[ok] wrote {out} ({len(layout.shelves)} shelves, {len(inventory)} items)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
