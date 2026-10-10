"""Smoke tests for the command-line entry points."""

import pytest

from qc_warehouse import build_scene, render
from qc_warehouse.constants import THIRD_PARTY_DIR

needs_assets = pytest.mark.skipif(
    not (THIRD_PARTY_DIR / "hope").exists() or not (THIRD_PARTY_DIR / "ycb").exists(),
    reason="third-party assets missing: run `uv run python scripts/fetch_assets.py`",
)


def test_build_reports_config_errors(tmp_path, capsys):
    bad = tmp_path / "inventory.yaml"
    bad.write_text("items: []\n")
    assert build_scene.main(["--inventory", str(bad), "--out", str(tmp_path / "s.xml")]) == 1
    assert "non-empty" in capsys.readouterr().err


def test_render_missing_scene(tmp_path, capsys):
    assert render.main(["--scene", str(tmp_path / "missing.xml")]) == 1
    assert "not found" in capsys.readouterr().err


def test_pick_rejects_bad_orders(tmp_path, capsys):
    from qc_warehouse import pick

    bad = tmp_path / "orders.yaml"
    bad.write_text("orders:\n  - id: X\n    items: [caviar]\n")
    assert pick.main(["--orders", str(bad)]) == 1
    assert "unknown item" in capsys.readouterr().err


def test_pick_missing_scene(tmp_path, capsys):
    from qc_warehouse import pick

    assert pick.main(["--scene", str(tmp_path / "nope.xml")]) == 1
    assert "scene not found" in capsys.readouterr().err


@needs_assets
def test_build_then_render_png(tmp_path):
    scene = tmp_path / "scene.xml"
    assert build_scene.main(["--out", str(scene)]) == 0
    out_dir = tmp_path / "renders"
    assert render.main(["--scene", str(scene), "--out-dir", str(out_dir), "--settle", "0.1", "--cameras", "overview"]) == 0
    png = (out_dir / "overview.png").read_bytes()
    assert png.startswith(b"\x89PNG") and len(png) > 10_000


@needs_assets
def test_render_unknown_camera(tmp_path, capsys):
    scene = tmp_path / "scene.xml"
    build_scene.main(["--out", str(scene)])
    assert render.main(["--scene", str(scene), "--out-dir", str(tmp_path), "--cameras", "nope"]) == 1
    assert "nope" in capsys.readouterr().err
