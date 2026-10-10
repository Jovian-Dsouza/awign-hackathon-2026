from pathlib import Path

import pytest
import yaml

from qc_warehouse.config import ConfigError, load_inventory, load_layout
from qc_warehouse.constants import DEFAULT_INVENTORY, DEFAULT_LAYOUT


def _write(tmp_path: Path, name: str, data: dict) -> Path:
    path = tmp_path / name
    path.write_text(yaml.safe_dump(data))
    return path


def _inventory(*items: dict) -> dict:
    return {"items": list(items)}


def _item(**overrides) -> dict:
    base = {"name": "ketchup", "source": "hope", "asset": "ketchup", "shelf": "A", "level": 1, "slot": 0}
    return {**base, **overrides}


@pytest.fixture
def layout():
    return load_layout(DEFAULT_LAYOUT)


def test_default_configs_load(layout):
    inventory = load_inventory(DEFAULT_INVENTORY, layout)
    assert [s.name for s in layout.shelves] == ["A", "B", "C"]
    assert len(inventory) >= 12
    assert {i.shelf for i in inventory} == {"A", "B", "C"}


def test_layout_is_immutable(layout):
    with pytest.raises(AttributeError):
        layout.shelf.width = 2.0  # type: ignore[misc]


def test_missing_file_raises(tmp_path):
    with pytest.raises(ConfigError, match="not found"):
        load_layout(tmp_path / "nope.yaml")


def test_layout_rejects_unsorted_boards(tmp_path):
    data = yaml.safe_load(DEFAULT_LAYOUT.read_text())
    data["shelf"]["board_heights"] = [0.5, 0.2]
    with pytest.raises(ConfigError, match="ascending"):
        load_layout(_write(tmp_path, "l.yaml", data))


def test_layout_rejects_non_axis_aligned_shelf(tmp_path):
    data = yaml.safe_load(DEFAULT_LAYOUT.read_text())
    data["shelves"][0]["yaw_deg"] = 45
    with pytest.raises(ConfigError, match="multiple of 90"):
        load_layout(_write(tmp_path, "l.yaml", data))


def test_layout_rejects_duplicate_shelf_names(tmp_path):
    data = yaml.safe_load(DEFAULT_LAYOUT.read_text())
    data["shelves"][1]["name"] = data["shelves"][0]["name"]
    with pytest.raises(ConfigError, match="duplicate"):
        load_layout(_write(tmp_path, "l.yaml", data))


@pytest.mark.parametrize(
    ("override", "message"),
    [
        ({"shelf": "Z"}, "unknown shelf"),
        ({"level": 9}, "level"),
        ({"level": 0}, "pickable"),
        ({"scale": True}, "scale"),
        ({"yaw_deg": "left"}, "yaw_deg"),
        ({"slot": 99}, "slot"),
        ({"source": "ikea"}, "source"),
        ({"euler_deg": [90, 0]}, "euler_deg"),
        ({"scale": 50}, "scale"),
    ],
)
def test_inventory_rejects_bad_item(tmp_path, layout, override, message):
    path = _write(tmp_path, "i.yaml", _inventory(_item(**override)))
    with pytest.raises(ConfigError, match=message):
        load_inventory(path, layout)


def test_item_scale_defaults_per_source(tmp_path, layout):
    path = _write(tmp_path, "i.yaml", _inventory(_item(), _item(name="can", source="ycb", slot=1), _item(name="big", slot=2, scale=2.0)))
    hope, ycb, explicit = load_inventory(path, layout)
    assert hope.effective_scale == 1.5
    assert ycb.effective_scale == 1.0
    assert explicit.effective_scale == 2.0


def test_inventory_rejects_slot_collision(tmp_path, layout):
    path = _write(tmp_path, "i.yaml", _inventory(_item(name="a"), _item(name="b")))
    with pytest.raises(ConfigError, match="occupied"):
        load_inventory(path, layout)


def test_inventory_rejects_duplicate_names(tmp_path, layout):
    path = _write(tmp_path, "i.yaml", _inventory(_item(slot=0), _item(slot=1)))
    with pytest.raises(ConfigError, match="duplicate"):
        load_inventory(path, layout)
