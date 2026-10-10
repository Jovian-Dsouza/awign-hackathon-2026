"""Project paths, physics settings and colour palette."""

from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
CONFIG_DIR = PROJECT_ROOT / "config"
ASSETS_DIR = PROJECT_ROOT / "assets"
THIRD_PARTY_DIR = ASSETS_DIR / "third_party"

DEFAULT_LAYOUT = CONFIG_DIR / "layout.yaml"
DEFAULT_INVENTORY = CONFIG_DIR / "inventory.yaml"
DEFAULT_SCENE = ASSETS_DIR / "scene.xml"

ITEM_SOURCES = {
    "hope": "hope/{asset}/{asset}.xml",
    "ycb": "ycb/YCB_sim/qc_{asset}.xml",
}
# HOPE objects are toy-sized groceries; scale them up to retail size by default.
SOURCE_DEFAULT_SCALE = {"hope": 1.5, "ycb": 1.0}
ITEM_SCALE_RANGE = (0.2, 5.0)

# Physics
TIMESTEP = 0.002
IMPRATIO = 10.0
NOSLIP_ITERATIONS = 3
ITEM_DROP_CLEARANCE = 0.003  # gap between item bottom and board at spawn
SHELF_FRICTION = (1.0, 0.005, 0.0001)

# Visual-only geoms (no collision) are placed in this group.
DECOR_GROUP = 2

# RGBA palette
COLORS = {
    "rack_steel": (0.15, 0.32, 0.62, 1.0),   # blue pallet-rack uprights
    "board": (0.78, 0.62, 0.42, 1.0),        # wood/MDF decking
    "lip": (0.95, 0.55, 0.10, 1.0),          # orange beam/lip
    "back_panel": (0.82, 0.84, 0.86, 1.0),
    "cardboard": (0.72, 0.55, 0.36, 1.0),
    "cardboard_dark": (0.60, 0.45, 0.28, 1.0),
    "wall": (0.90, 0.90, 0.88, 1.0),
    "tape_yellow": (0.98, 0.80, 0.10, 1.0),
    "tape_green": (0.15, 0.70, 0.35, 1.0),
    "table_top": (0.85, 0.86, 0.88, 1.0),
    "table_leg": (0.25, 0.25, 0.28, 1.0),
    "tote": (0.10, 0.55, 0.30, 1.0),          # green delivery totes
    "sign": (0.90, 0.20, 0.25, 1.0),
    "dock": (0.20, 0.45, 0.85, 0.6),
    "drop_zone": (0.1, 0.9, 0.3, 0.4),
}
