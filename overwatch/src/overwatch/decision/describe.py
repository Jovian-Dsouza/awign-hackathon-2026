"""Turn a detector measurement into the plain sentence a text decision model reads.

Laya is a language model: given raw numbers it answered ~0.42 for everything (a real drop,
a shadow and a hand alike; AUC 0.32). Given the same measurements as a sentence, it ranked
real drops above false alarms (AUC 0.90 on cases built from the team's sim frames).

The mapping is fixed thresholds, identical for every case, so it never "knows" the answer;
it only translates numbers into words. Inference and training export both use it.
"""

from __future__ import annotations

from overwatch.events import CAMERA_FAULT, DROPPED_ITEM, SPILL


def describe(kind: str, features: dict, summary: str = "") -> str:
    f = features
    if kind == DROPPED_ITEM:
        area = float(f.get("area_pct", 0.0))
        size = "tiny" if area < 0.15 else "small" if area < 0.5 else "fairly large"
        aspect, solidity = float(f.get("aspect", 1.0)), float(f.get("solidity", 1.0))
        shape = ("compact and solid, like a single object" if solidity >= 0.85 and aspect <= 2
                 else "irregular and elongated" if aspect > 2 else "irregular")
        edges = "sharp, well defined edges" if float(f.get("edge_density", 0.0)) >= 0.05 else "soft, blurry edges"
        c = float(f.get("contrast", 0.0))
        contrast = ("strongly different from the table surface" if c >= 80
                    else "moderately different from the table" if c >= 45
                    else "only faintly different from the table")
        secs = float(f.get("stationary_s", 0.0))
        return (f"During a robot pickup, a {size} object appeared on the table and then stopped moving for "
                f"{secs:.0f} seconds. It is {shape}, with {edges}, and {contrast}.")
    if kind == SPILL:
        area = float(f.get("area_pct", 0.0))
        c = float(f.get("contrast", 0.0))
        edges = "soft, spreading edges" if float(f.get("edge_density", 0.0)) < 0.05 else "ragged edges"
        tone = "clearly different from" if c >= 60 else "only slightly different from"
        return (f"During a robot pickup, a large irregular patch covering {area:.1f} percent of the view appeared "
                f"on the table and stayed there for {float(f.get('stationary_s', 0.0)):.0f} seconds. It is spread out "
                f"and shapeless, with {edges}, {tone} the table surface. It looks like something was spilled.")
    if kind == CAMERA_FAULT:
        return f"The monitoring camera has a problem ({f.get('fault', 'unknown')}): {summary}"
    return summary or kind
