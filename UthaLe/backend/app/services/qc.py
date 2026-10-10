"""Deterministic placement verification.

Principle: a robot reporting "placed" only means its motion finished. A record is
PASS only when independent evidence (tag identity + weight) agrees with the
order. Missing evidence -> REVIEW, contradicting evidence -> FAIL.
"""

from __future__ import annotations

from dataclasses import dataclass

WEIGHT_TOLERANCE_FRACTION = 0.10
WEIGHT_TOLERANCE_MIN_G = 25.0
EMPTY_FRACTION = 0.25  # below this share of the expected weight, nothing was placed


@dataclass(frozen=True)
class Expected:
    sku: str
    quantity: int
    destination: str
    nominal_weight_g: int


def evaluate(expected: Expected, robot_outcome: str, robot_destination: str | None,
             sensors: dict) -> tuple[str, list[str]]:
    fails: list[str] = []
    reviews: list[str] = []

    if robot_outcome != "placed":
        fails.append(f"Robot did not complete placement (reported '{robot_outcome}')")
    elif robot_destination and robot_destination != expected.destination:
        fails.append(f"Robot placed at {robot_destination}, order expects {expected.destination}")

    nfc = sensors.get("nfc")
    load = sensors.get("load_cell")

    if nfc is not None:
        if not nfc.get("tag_read"):
            reviews.append("NFC: no tag read at the drop point")
        elif nfc.get("tag_sku") != expected.sku:
            fails.append(f"NFC: tag reads '{nfc.get('tag_sku')}', order expects '{expected.sku}'")

    if load is not None:
        exp_g = expected.nominal_weight_g * expected.quantity
        delta = float(load.get("delta_g", 0.0))
        tol = max(WEIGHT_TOLERANCE_MIN_G, WEIGHT_TOLERANCE_FRACTION * exp_g)
        if delta < EMPTY_FRACTION * exp_g:
            fails.append(f"Load cell: +{delta:.0f} g, expected ≈{exp_g} g (nothing placed?)")
        elif abs(delta - exp_g) > tol:
            reviews.append(f"Load cell: +{delta:.0f} g outside {exp_g}±{tol:.0f} g")

    if nfc is None and load is None and robot_outcome == "placed":
        reviews.append("No independent evidence; robot self-report only")

    if fails:
        return "FAIL", fails + reviews
    if reviews:
        return "REVIEW", reviews
    return "PASS", []
