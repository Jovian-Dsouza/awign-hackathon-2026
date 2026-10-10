import mujoco
import pytest

from qc_warehouse.config import Item, load_layout
from qc_warehouse.constants import DEFAULT_LAYOUT, THIRD_PARTY_DIR
from qc_warehouse.items import AssetError, add_item

pytestmark = pytest.mark.skipif(not (THIRD_PARTY_DIR / "hope").exists(), reason="third-party assets missing")


def _item(**overrides) -> Item:
    base = {"name": "milk", "source": "hope", "asset": "milk", "shelf": "A", "level": 1, "slot": 0, "euler_deg": (90.0, 0.0, 0.0)}
    return Item(**{**base, **overrides})


def test_item_too_wide_for_slot_is_rejected():
    layout = load_layout(DEFAULT_LAYOUT)
    with pytest.raises(AssetError, match="too wide"):
        # Flat mac & cheese box at 4x: ~0.34 m wide (> 0.3 m slot) but only ~0.1 m tall.
        add_item(mujoco.MjSpec(), _item(asset="macaroni_and_cheese", euler_deg=(0.0, 0.0, 0.0), scale=4.0), layout)


def test_item_too_tall_for_level_gap_is_rejected():
    layout = load_layout(DEFAULT_LAYOUT)
    # Upright milk at 2.5x is ~0.33 m tall; level 1 has 0.475 m clearance -> fits.
    add_item(mujoco.MjSpec(), _item(scale=2.5), layout)
    with pytest.raises(AssetError, match="too tall"):
        add_item(mujoco.MjSpec(), _item(scale=3.8), layout)


def test_item_body_gets_single_free_joint():
    layout = load_layout(DEFAULT_LAYOUT)
    spec = mujoco.MjSpec()
    add_item(spec, _item(), layout)
    model = spec.compile()
    assert model.njnt == 1 and model.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE
