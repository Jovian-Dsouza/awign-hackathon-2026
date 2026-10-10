"""Manage the dockerised slam_toolbox + rosbridge service.

Usage:
  uv run qcw-slam up        # build (first time) and start; waits until healthy
  uv run qcw-slam restart   # fresh SLAM session (clears the map)
  uv run qcw-slam status | logs | down
"""

from __future__ import annotations

import argparse
import subprocess
import sys

from qc_warehouse.constants import PROJECT_ROOT

COMPOSE_FILE = PROJECT_ROOT / "docker" / "compose.yaml"
COMMANDS = {
    "up": ("up", "-d", "--build", "--wait"),
    # slam_toolbox keeps some state across /reset, so a new session needs a new container.
    "restart": ("up", "-d", "--force-recreate", "--wait"),
    "down": ("down",),
    "status": ("ps",),
    "logs": ("logs", "--tail", "100"),
}
COMPOSE_TIMEOUT = 900  # first build pulls ROS 2 images


class SlamServiceError(RuntimeError):
    """docker compose failed or is not available."""


def compose(command: str) -> None:
    if command not in COMMANDS:
        raise ValueError(f"unknown command {command!r}; expected one of {sorted(COMMANDS)}")
    try:
        result = subprocess.run(["docker", "compose", "-f", str(COMPOSE_FILE), *COMMANDS[command]], timeout=COMPOSE_TIMEOUT, check=False)
    except FileNotFoundError as exc:
        raise SlamServiceError("docker not found; install Docker Desktop to run SLAM (or use --localization ground-truth)") from exc
    except subprocess.TimeoutExpired as exc:
        raise SlamServiceError(f"`docker compose {command}` timed out after {COMPOSE_TIMEOUT}s") from exc
    if result.returncode != 0:
        raise SlamServiceError(f"`docker compose {command}` failed (exit {result.returncode})")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=sorted(COMMANDS))
    args = parser.parse_args(argv)
    try:
        compose(args.command)
    except SlamServiceError as exc:
        print(f"[error] {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
