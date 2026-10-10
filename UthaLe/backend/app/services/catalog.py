"""Product catalog.

SKUs and shelf slots come from the MuJoCo warehouse's `config/inventory.yaml`, so
the dashboard and the simulation describe the same dark store. Nominal weights
are illustrative values used by the simulated load cell; they are not measured.
"""

from __future__ import annotations

import logging
from pathlib import Path

import yaml

from app.models.schemas import CatalogItem

log = logging.getLogger(__name__)

# Illustrative nominal pack weights (grams) for the simulated load cell.
NOMINAL_WEIGHT_G = {
    "ketchup": 400, "bbq_sauce": 450, "mustard": 300, "salad_dressing": 350,
    "tomato_sauce": 420, "cookies": 200, "mac_and_cheese": 210, "pudding_mix": 110,
    "popcorn": 280, "crackers": 410, "sugar": 500, "tomato_soup": 350, "tuna": 170,
    "butter": 250, "cream_cheese": 225, "alphabet_soup": 350, "milk": 1030,
    "orange_juice": 1050,
}

_FALLBACK = [
    ("ketchup", "A", 1, 0), ("cookies", "B", 1, 0), ("crackers", "B", 2, 1),
    ("tomato_soup", "C", 1, 0), ("milk", "C", 2, 1),
]


def _display_name(sku: str) -> str:
    return sku.replace("_", " ").title().replace("Bbq", "BBQ").replace("And", "&")


def load_catalog(path: Path) -> dict[str, CatalogItem]:
    rows: list[tuple[str, str, int, int]]
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        rows = [(i["name"], i["shelf"], int(i["level"]), int(i["slot"])) for i in data["items"]]
        log.info("Loaded %d catalog items from %s", len(rows), path)
    except (OSError, KeyError, TypeError, yaml.YAMLError) as exc:
        log.warning("Could not read inventory %s (%s); using built-in fallback catalog", path, exc)
        rows = _FALLBACK
    return {
        sku: CatalogItem(
            sku=sku,
            name=_display_name(sku),
            location=f"{shelf}-L{level}-S{slot}",
            shelf=shelf,
            nominal_weight_g=NOMINAL_WEIGHT_G.get(sku, 300),
        )
        for sku, shelf, level, slot in rows
    }
