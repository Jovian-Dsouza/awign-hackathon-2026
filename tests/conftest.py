"""Shared test helpers."""

import os
import socket
from urllib.parse import urlparse

import pytest

from qc_warehouse.autonomy.slam_bridge import DEFAULT_ROSBRIDGE_URL
from qc_warehouse.constants import DEFAULT_SCENE

ROSBRIDGE_URL = os.environ.get("QCW_ROSBRIDGE", DEFAULT_ROSBRIDGE_URL)


def _reachable(url: str) -> bool:
    parsed = urlparse(url)
    try:
        with socket.create_connection((parsed.hostname, parsed.port or 9090), timeout=1.0):
            return True
    except OSError:
        return False


# Tests that talk to the real slam_toolbox container (`uv run qcw-slam up`).
ROS_MARKS = [
    pytest.mark.ros,
    pytest.mark.skipif(
        not _reachable(ROSBRIDGE_URL) or not DEFAULT_SCENE.exists(), reason=f"slam_toolbox not reachable at {ROSBRIDGE_URL}"
    ),
]
