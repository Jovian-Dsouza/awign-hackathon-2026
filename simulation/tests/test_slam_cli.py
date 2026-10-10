import subprocess
import types

import numpy as np
import pytest

from qc_warehouse import slam_cli
from qc_warehouse.autonomy.frames import BasePose
from qc_warehouse.map_image import SCALE, map_image, write_png


def _fake_run(returncode=0, exc=None):
    calls = []

    def run(cmd, **kwargs):
        calls.append(cmd)
        if exc is not None:
            raise exc
        return types.SimpleNamespace(returncode=returncode)

    return run, calls


def test_compose_builds_docker_command(monkeypatch):
    run, calls = _fake_run()
    monkeypatch.setattr(subprocess, "run", run)
    assert slam_cli.main(["restart"]) == 0
    assert calls[0][:4] == ["docker", "compose", "-f", str(slam_cli.COMPOSE_FILE)]
    assert "--force-recreate" in calls[0]


@pytest.mark.parametrize(
    ("run_kwargs", "message"),
    [({"exc": FileNotFoundError()}, "docker not found"), ({"returncode": 3}, "exit 3"), ({"exc": subprocess.TimeoutExpired("docker", 1)}, "timed out")],
)
def test_compose_failures_are_reported(monkeypatch, capsys, run_kwargs, message):
    run, _ = _fake_run(**run_kwargs)
    monkeypatch.setattr(subprocess, "run", run)
    assert slam_cli.main(["up"]) == 1
    assert message in capsys.readouterr().err


def test_unknown_command_rejected():
    with pytest.raises(ValueError, match="unknown command"):
        slam_cli.compose("explode")


def test_map_image_and_png(tmp_path):
    values = np.full((10, 6), -1, dtype=np.int8)
    values[2, 3] = 100
    values[5, 5] = 0
    img = map_image(values, (0.0, 0.0), 0.1, [(BasePose(0.4, 0.1, 0), BasePose(0.4, 0.2, 0))])
    assert img.shape == (6 * SCALE, 10 * SCALE, 3)
    # +y is up: cell (2, 3) lands in picture row 6-1-3 = 2.
    assert tuple(img[2 * SCALE, 2 * SCALE]) == (30, 30, 30)
    path = tmp_path / "map.png"
    write_png(path, img)
    assert path.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    with pytest.raises(ValueError):
        write_png(path, np.zeros((2, 2), dtype=np.uint8))
