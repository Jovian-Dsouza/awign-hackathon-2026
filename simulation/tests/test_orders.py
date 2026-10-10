from pathlib import Path

import pytest
import yaml

from qc_warehouse.autonomy.orders import Order, load_orders
from qc_warehouse.config import ConfigError, load_inventory, load_layout
from qc_warehouse.constants import CONFIG_DIR, DEFAULT_INVENTORY, DEFAULT_LAYOUT


@pytest.fixture(scope="module")
def inventory():
    return load_inventory(DEFAULT_INVENTORY, load_layout(DEFAULT_LAYOUT))


def _write(tmp_path: Path, orders) -> Path:
    path = tmp_path / "orders.yaml"
    path.write_text(yaml.safe_dump({"orders": orders}))
    return path


def test_default_orders_load(inventory):
    orders = load_orders(CONFIG_DIR / "orders.yaml", inventory, max_orders=2)
    assert orders[0] == Order("ORD-1001", ("ketchup", "cookies", "milk"))


@pytest.mark.parametrize(
    ("orders", "message"),
    [
        ([], "non-empty"),
        ([{"id": "A", "items": ["caviar"]}], "unknown item"),
        ([{"id": "A", "items": []}], "non-empty list"),
        ([{"id": "A", "items": ["milk"]}, {"id": "B", "items": ["milk"]}], "already claimed"),
        ([{"id": "A", "items": ["milk"]}, {"id": "A", "items": ["tuna"]}], "duplicate"),
        ([{"items": ["milk"]}], "string 'id'"),
        ([{"id": str(i), "items": [n]} for i, n in enumerate(["milk", "tuna", "sugar"])], "totes"),
    ],
)
def test_invalid_orders_rejected(tmp_path, inventory, orders, message):
    with pytest.raises(ConfigError, match=message):
        load_orders(_write(tmp_path, orders), inventory, max_orders=2)


def test_missing_orders_file(tmp_path, inventory):
    with pytest.raises(ConfigError, match="not found"):
        load_orders(tmp_path / "nope.yaml", inventory, max_orders=2)
