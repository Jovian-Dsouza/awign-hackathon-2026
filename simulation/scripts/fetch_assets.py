"""Download third-party grocery item assets into assets/third_party/.

Sources (see README for licenses):
  - LIBERO HOPE objects (MIT)      -> assets/third_party/hope/<name>/<name>.xml
  - Hello Robot Stretch 3 from MuJoCo Menagerie (Apache-2.0) -> assets/third_party/stretch3/
  - YCB_sim by Vikash Kumar (Apache-2.0) -> assets/third_party/ycb/YCB_sim/...

YCB_sim ships its objects as <mujocoinclude> fragments, so this script also
writes one small standalone wrapper MJCF per YCB object (qc_<id>.xml) that the
scene builder can load like any other model.

Usage:  uv run python scripts/fetch_assets.py [--force]
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
THIRD_PARTY = ROOT / "assets" / "third_party"

LIBERO_REPO = "https://github.com/Lifelong-Robot-Learning/LIBERO.git"
LIBERO_SHA = "8f1084e3132a39270c3a13ebe37270a43ece2a01"
LIBERO_HOPE_DIR = "libero/libero/assets/stable_hope_objects"
YCB_REPO = "https://github.com/vikashplus/YCB_sim.git"
YCB_SHA = "57546b87f4724c947eadd4241a7892473febb88d"
MENAGERIE_REPO = "https://github.com/google-deepmind/mujoco_menagerie.git"
MENAGERIE_SHA = "f054586a8e90465d49ee5be15335c4a0c7f57caf"
STRETCH_DIR = "hello_robot_stretch_3"
GIT_TIMEOUT_S = 300

YCB_IDS = (
    "003_cracker_box",
    "004_sugar_box",
    "005_tomato_soup_can",
    "006_mustard_bottle",
    "007_tuna_fish_can",
    "008_pudding_box",
    "009_gelatin_box",
    "010_potted_meat_can",
)

YCB_WRAPPER = """<mujoco model="{oid}">
  <include file="includes/defaults_ycb.xml"/>
  <include file="includes/assets_{oid}.xml"/>
  <worldbody>
    <body name="object">
      <include file="includes/body_{oid}.xml"/>
    </body>
  </worldbody>
</mujoco>
"""


def _run(cmd: list[str], cwd: Path | None = None) -> str:
    try:
        result = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, timeout=GIT_TIMEOUT_S)
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError(f"Command timed out after {GIT_TIMEOUT_S}s: {' '.join(cmd)}") from exc
    if result.returncode != 0:
        raise RuntimeError(f"Command failed: {' '.join(cmd)}\n{result.stderr.strip()}")
    return result.stdout.strip()


def _checkout_pinned(repo: str, sha: str, clone: Path, sparse_dir: str | None = None) -> None:
    """Shallow-fetch exactly `sha` and verify HEAD matches it."""
    clone.mkdir(parents=True)
    _run(["git", "init", "-q"], cwd=clone)
    _run(["git", "remote", "add", "origin", repo], cwd=clone)
    if sparse_dir:
        _run(["git", "sparse-checkout", "set", "--no-cone", sparse_dir, "/LICENSE"], cwd=clone)
    _run(["git", "fetch", "-q", "--depth", "1", "--filter=blob:none", "origin", sha], cwd=clone)
    _run(["git", "checkout", "-q", "FETCH_HEAD"], cwd=clone)
    head = _run(["git", "rev-parse", "HEAD"], cwd=clone)
    if head != sha:
        raise RuntimeError(f"{repo}: expected commit {sha}, got {head}")


def _copy_tree(src: Path, dest: Path) -> None:
    """Copy without following symlinks, so links can't pull in files from outside the clone."""
    shutil.copytree(src, dest, symlinks=True, ignore=shutil.ignore_patterns(".git"))


def fetch_hope(dest: Path, workdir: Path) -> None:
    clone = workdir / "LIBERO"
    _checkout_pinned(LIBERO_REPO, LIBERO_SHA, clone, sparse_dir=LIBERO_HOPE_DIR)
    _copy_tree(clone / LIBERO_HOPE_DIR, dest)
    shutil.copy(clone / "LICENSE", dest / "LICENSE")


def fetch_ycb(dest: Path, workdir: Path) -> None:
    clone = workdir / "YCB_sim"
    _checkout_pinned(YCB_REPO, YCB_SHA, clone)
    _copy_tree(clone, dest / "YCB_sim")
    for oid in YCB_IDS:
        (dest / "YCB_sim" / f"qc_{oid}.xml").write_text(YCB_WRAPPER.format(oid=oid))


def fetch_stretch(dest: Path, workdir: Path) -> None:
    clone = workdir / "menagerie"
    _checkout_pinned(MENAGERIE_REPO, MENAGERIE_SHA, clone, sparse_dir=STRETCH_DIR)
    _copy_tree(clone / STRETCH_DIR, dest)


def verify(third_party: Path) -> list[str]:
    """Return a list of missing required files (empty when everything is present)."""
    required = [third_party / "hope" / "ketchup" / "ketchup.xml", third_party / "stretch3" / "stretch.xml"]
    required += [third_party / "ycb" / "YCB_sim" / f"qc_{oid}.xml" for oid in YCB_IDS]
    required += [third_party / "ycb" / "YCB_sim" / "meshes" / f"{oid}.msh" for oid in YCB_IDS]
    return [str(p.relative_to(ROOT)) for p in required if not p.exists()]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--force", action="store_true", help="re-download even if assets exist")
    args = parser.parse_args()

    sources = {"hope": fetch_hope, "ycb": fetch_ycb, "stretch3": fetch_stretch}
    THIRD_PARTY.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        for name, fetch in sources.items():
            dest = THIRD_PARTY / name
            if dest.exists() and not args.force:
                print(f"[skip] {name}: already present")
                continue
            print(f"[fetch] {name} ...")
            staging = Path(tmp) / f"staged_{name}"
            try:
                fetch(staging, Path(tmp))
            except (RuntimeError, OSError) as exc:
                print(f"[error] {name}: {exc} (existing assets left untouched)", file=sys.stderr)
                return 1
            # Only replace the existing copy once the new one is complete.
            shutil.rmtree(dest, ignore_errors=True)
            shutil.move(str(staging), dest)

    missing = verify(THIRD_PARTY)
    if missing:
        print("[error] missing files after fetch:\n  " + "\n  ".join(missing), file=sys.stderr)
        print("Re-run with --force.", file=sys.stderr)
        return 1
    print("[ok] all assets present in", THIRD_PARTY.relative_to(ROOT))
    return 0


if __name__ == "__main__":
    sys.exit(main())
