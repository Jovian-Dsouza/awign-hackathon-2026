"""Load and validate the layout / inventory YAML files into frozen dataclasses."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from dsr_sim.constants import ITEM_SCALE_RANGE, ITEM_SOURCES, SOURCE_DEFAULT_SCALE


class ConfigError(ValueError):
    """Raised when a config file is missing or invalid."""


@dataclass(frozen=True)
class TableConfig:
    """The flat demo surface (a table top); its top face is the world z = 0 plane."""

    size: tuple[float, float]
    thickness: float


@dataclass(frozen=True)
class ShelfConfig:
    width: float
    depth: float
    height: float
    board_heights: tuple[float, ...]
    board_thickness: float
    post_size: float
    lip_height: float
    slots_per_level: int
    pickable_levels: tuple[int, ...]


@dataclass(frozen=True)
class Placement:
    name: str
    pos: tuple[float, float]
    yaw_deg: float = 0.0


@dataclass(frozen=True)
class TrayConfig:
    """Open cardboard drop tray that delivered items are placed into."""

    pos: tuple[float, float]
    yaw_deg: float
    size: tuple[float, float, float]  # outer x, y, wall height


@dataclass(frozen=True)
class DockConfig:
    pos: tuple[float, float]
    pad_size: tuple[float, float]
    yaw_deg: float = 0.0  # robot heading at spawn (0 = facing +x)


@dataclass(frozen=True)
class Layout:
    table: TableConfig
    shelf: ShelfConfig
    shelves: tuple[Placement, ...]
    tray: TrayConfig
    robot_dock: DockConfig


@dataclass(frozen=True)
class Item:
    name: str
    source: str
    asset: str
    shelf: str
    level: int
    slot: int
    euler_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    yaw_deg: float = 0.0
    scale: float | None = None  # None -> SOURCE_DEFAULT_SCALE[source]
    mass_kg: float | None = None  # total mass of the (scaled) item; None -> whatever the asset's densities give

    @property
    def effective_scale(self) -> float:
        return self.scale if self.scale is not None else SOURCE_DEFAULT_SCALE[self.source]


def _read_yaml(path: Path) -> dict[str, Any]:
    path = Path(path)
    if not path.exists():
        raise ConfigError(f"config file not found: {path}")
    try:
        data = yaml.safe_load(path.read_text())
    except yaml.YAMLError as exc:
        raise ConfigError(f"invalid YAML in {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise ConfigError(f"{path}: expected a mapping at top level")
    return data


def _vec(value: Any, n: int, field: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != n:
        raise ConfigError(f"{field}: expected a list of {n} numbers, got {value!r}")
    try:
        return tuple(float(v) for v in value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"{field}: non-numeric value in {value!r}") from exc


def _build(cls: type, data: dict[str, Any], section: str) -> Any:
    try:
        return cls(**data)
    except TypeError as exc:
        raise ConfigError(f"{section}: {exc}") from exc


def _check_right_angle(yaw_deg: float, field: str) -> None:
    """Grasp planning works on axis-aligned bounding boxes, so racks/stations must be axis-aligned."""
    if float(yaw_deg) % 90 != 0:
        raise ConfigError(f"{field}: {yaw_deg} deg not supported; use a multiple of 90")


def _parse_shelf(raw: dict[str, Any]) -> ShelfConfig:
    boards = tuple(float(h) for h in raw.get("board_heights", ()))
    if not boards or list(boards) != sorted(boards):
        raise ConfigError("shelf.board_heights must be a non-empty ascending list")
    if boards[-1] >= float(raw.get("height", 0)):
        raise ConfigError("shelf.height must exceed the top board height")
    data = {**raw, "board_heights": boards, "pickable_levels": tuple(raw.get("pickable_levels", ()))}
    shelf = _build(ShelfConfig, data, "shelf")
    if any(not 0 <= lvl < len(boards) for lvl in shelf.pickable_levels):
        raise ConfigError("shelf.pickable_levels references a non-existent level")
    if shelf.slots_per_level < 1:
        raise ConfigError("shelf.slots_per_level must be >= 1")
    if min(shelf.width, shelf.depth, shelf.board_thickness, shelf.post_size) <= 0:
        raise ConfigError("shelf dimensions must be positive")
    gaps = [b - a for a, b in zip(boards, boards[1:])] + [shelf.height - boards[-1]]
    if min(gaps) <= shelf.board_thickness + 0.1:
        raise ConfigError("shelf levels are too close together (need > board_thickness + 0.1 m)")
    return shelf


def load_layout(path: Path) -> Layout:
    try:
        return _parse_layout(_read_yaml(path), path)
    except (AttributeError, TypeError, ValueError) as exc:
        if isinstance(exc, ConfigError):
            raise
        raise ConfigError(f"{path}: malformed layout ({exc})") from exc


def _parse_layout(raw: dict[str, Any], path: Path) -> Layout:
    try:
        table_raw, shelf_raw, shelves_raw = raw["table"], raw["shelf"], raw["shelves"]
        tray_raw, dock_raw = raw["tray"], raw["robot_dock"]
    except KeyError as exc:
        raise ConfigError(f"{path}: missing section {exc}") from exc

    table = _build(TableConfig, {**table_raw, "size": _vec(table_raw.get("size"), 2, "table.size")}, "table")
    shelves = tuple(
        _build(Placement, {**s, "pos": _vec(s.get("pos"), 2, f"shelves[{i}].pos")}, f"shelves[{i}]")
        for i, s in enumerate(shelves_raw)
    )
    names = [s.name for s in shelves]
    if len(set(names)) != len(names):
        raise ConfigError(f"duplicate shelf names: {names}")
    for placement in shelves:
        _check_right_angle(placement.yaw_deg, f"shelves[{placement.name}].yaw_deg")

    tray = _build(
        TrayConfig,
        {**tray_raw, "pos": _vec(tray_raw.get("pos"), 2, "tray.pos"), "size": _vec(tray_raw.get("size"), 3, "tray.size")},
        "tray",
    )
    _check_right_angle(tray.yaw_deg, "tray.yaw_deg")
    dock = _build(
        DockConfig,
        {
            "pos": _vec(dock_raw.get("pos"), 2, "robot_dock.pos"),
            "pad_size": _vec(dock_raw.get("pad_size"), 2, "robot_dock.pad_size"),
            "yaw_deg": float(dock_raw.get("yaw_deg", 0.0)),
        },
        "robot_dock",
    )
    return Layout(table=table, shelf=_parse_shelf(shelf_raw), shelves=shelves, tray=tray, robot_dock=dock)


def _parse_item(raw: dict[str, Any], layout: Layout, idx: int) -> Item:
    if not isinstance(raw, dict):
        raise ConfigError(f"items[{idx}]: expected a mapping, got {raw!r}")
    where = f"items[{idx}] ({raw.get('name', '?')})"
    data = dict(raw)
    if "euler_deg" in data:
        data["euler_deg"] = _vec(data["euler_deg"], 3, f"{where}.euler_deg")
    item = _build(Item, data, where)

    if item.source not in ITEM_SOURCES:
        raise ConfigError(f"{where}: unknown source {item.source!r}, expected one of {sorted(ITEM_SOURCES)}")
    if item.shelf not in {s.name for s in layout.shelves}:
        raise ConfigError(f"{where}: unknown shelf {item.shelf!r}")
    if not 0 <= item.level < len(layout.shelf.board_heights):
        raise ConfigError(f"{where}: level {item.level} out of range")
    if item.level not in layout.shelf.pickable_levels:
        raise ConfigError(f"{where}: level {item.level} is not pickable (allowed: {list(layout.shelf.pickable_levels)})")
    if isinstance(item.yaw_deg, bool) or not isinstance(item.yaw_deg, (int, float)):
        raise ConfigError(f"{where}: yaw_deg must be a number")
    if item.scale is not None and (isinstance(item.scale, bool) or not isinstance(item.scale, (int, float))):
        raise ConfigError(f"{where}: scale must be a number")
    if not 0 <= item.slot < layout.shelf.slots_per_level:
        raise ConfigError(f"{where}: slot {item.slot} out of range")
    if item.mass_kg is not None and (isinstance(item.mass_kg, bool) or not isinstance(item.mass_kg, (int, float)) or not 0.001 <= item.mass_kg <= 1.0):
        raise ConfigError(f"{where}: mass_kg must be a number between 0.001 and 1.0")
    lo, hi = ITEM_SCALE_RANGE
    if item.scale is not None and not lo <= item.scale <= hi:
        raise ConfigError(f"{where}: scale {item.scale} outside [{lo}, {hi}]")
    return item


def load_inventory(path: Path, layout: Layout) -> tuple[Item, ...]:
    raw = _read_yaml(path)
    entries = raw.get("items")
    if not isinstance(entries, list) or not entries:
        raise ConfigError(f"{path}: 'items' must be a non-empty list")

    items = tuple(_parse_item(e, layout, i) for i, e in enumerate(entries))

    names = [i.name for i in items]
    dupes = sorted({n for n in names if names.count(n) > 1})
    if dupes:
        raise ConfigError(f"duplicate item names: {dupes}")

    seen: dict[tuple[str, int, int], str] = {}
    for item in items:
        key = (item.shelf, item.level, item.slot)
        if key in seen:
            raise ConfigError(f"slot {key} occupied by both {seen[key]!r} and {item.name!r}")
        seen[key] = item.name
    return items
