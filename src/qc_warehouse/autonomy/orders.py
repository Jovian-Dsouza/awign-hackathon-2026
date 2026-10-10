"""Load and validate customer orders from YAML."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

from qc_warehouse.config import ConfigError, Item


@dataclass(frozen=True)
class Order:
    order_id: str
    items: tuple[str, ...]


def load_orders(path: Path, inventory: tuple[Item, ...], max_orders: int) -> tuple[Order, ...]:
    """Parse `path` and check every item is stocked and ordered at most once."""
    path = Path(path)
    if not path.exists():
        raise ConfigError(f"orders file not found: {path}")
    try:
        raw = yaml.safe_load(path.read_text())
    except yaml.YAMLError as exc:
        raise ConfigError(f"invalid YAML in {path}: {exc}") from exc

    entries = raw.get("orders") if isinstance(raw, dict) else None
    if not isinstance(entries, list) or not entries:
        raise ConfigError(f"{path}: 'orders' must be a non-empty list")
    if len(entries) > max_orders:
        raise ConfigError(f"{path}: {len(entries)} orders but only {max_orders} delivery totes")

    stocked = {item.name for item in inventory}
    orders: list[Order] = []
    claimed: dict[str, str] = {}
    for idx, entry in enumerate(entries):
        if not isinstance(entry, dict) or not isinstance(entry.get("id"), str):
            raise ConfigError(f"orders[{idx}]: expected a mapping with a string 'id'")
        items = entry.get("items")
        if not isinstance(items, list) or not items or not all(isinstance(i, str) for i in items):
            raise ConfigError(f"order {entry['id']}: 'items' must be a non-empty list of item names")
        for name in items:
            if name not in stocked:
                raise ConfigError(f"order {entry['id']}: unknown item {name!r} (not in inventory)")
            if name in claimed:
                raise ConfigError(f"order {entry['id']}: {name!r} already claimed by order {claimed[name]} (one unit in stock)")
            claimed[name] = entry["id"]
        orders.append(Order(order_id=entry["id"], items=tuple(items)))

    ids = [o.order_id for o in orders]
    if len(set(ids)) != len(ids):
        raise ConfigError(f"{path}: duplicate order ids {ids}")
    return tuple(orders)
