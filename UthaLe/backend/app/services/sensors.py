"""Placement sensors (NFC tag reader + load cell under the tote).

No physical sensors are connected yet. `SimulatedSensorHub` produces readings
that are explicitly tagged `source: "simulated"`; `NullSensorHub` returns nothing,
which makes QC fall back to REVIEW (robot self-report alone is never a PASS).
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from app.services.robot import SimulationScenario


class SensorHub(ABC):
    source: str

    @abstractmethod
    def read_placement(self, line_key: str, sku: str, quantity: int, nominal_weight_g: int,
                       catalog_weights: dict[str, int]) -> dict:
        """Return {"nfc": {...} | None, "load_cell": {...} | None}."""


class NullSensorHub(SensorHub):
    source = "none"

    def read_placement(self, line_key, sku, quantity, nominal_weight_g, catalog_weights) -> dict:
        return {"nfc": None, "load_cell": None}


class SimulatedSensorHub(SensorHub):
    source = "simulated"

    def __init__(self, scenario: SimulationScenario):
        self.scenario = scenario

    def read_placement(self, line_key, sku, quantity, nominal_weight_g, catalog_weights) -> dict:
        rng = self.scenario.rng
        anomaly = self.scenario.anomaly_for(line_key)
        if anomaly == "grasp_failure" and self.scenario.is_injected(line_key):
            return {
                "nfc": {"tag_read": False, "tag_sku": None, "source": "simulated"},
                "load_cell": {"delta_g": round(rng.gauss(0, 2), 1),
                              "expected_g": nominal_weight_g * quantity, "source": "simulated"},
            }

        tag_sku: str | None = sku
        unit_weight = nominal_weight_g
        if anomaly == "sku_mismatch":
            others = [s for s in catalog_weights if s != sku] or [sku]
            tag_sku = rng.choice(others)
            unit_weight = catalog_weights.get(tag_sku, nominal_weight_g)
        elif anomaly == "nfc_missing":
            tag_sku = None

        delta = unit_weight * quantity * rng.gauss(1.0, 0.012)
        if anomaly == "weight_mismatch":
            delta *= rng.choice((0.62, 1.45))
        return {
            "nfc": {"tag_read": tag_sku is not None, "tag_sku": tag_sku, "source": "simulated"},
            "load_cell": {"delta_g": round(delta, 1), "expected_g": nominal_weight_g * quantity,
                          "source": "simulated"},
        }
