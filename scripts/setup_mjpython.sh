#!/usr/bin/env bash
# Make `mjpython` (needed for MuJoCo's passive viewer on macOS) work inside a uv venv.
#
# uv's standalone CPython links libpython via @executable_path/../lib, which from
# .venv/bin resolves to .venv/lib/ -- where the dylib doesn't exist. Symlink it in.
# Safe to re-run.
set -euo pipefail
cd "$(dirname "$0")/.."

libdir=$(uv run python -c "import sysconfig; print(sysconfig.get_config_var('LIBDIR'))")
ldlib=$(uv run python -c "import sysconfig; print(sysconfig.get_config_var('LDLIBRARY'))")
src="$libdir/$ldlib"
if [[ ! -f "$src" ]]; then
  echo "[error] $src not found; this Python has no shared libpython (mjpython needs one)" >&2
  exit 1
fi
mkdir -p .venv/lib
ln -sf "$src" ".venv/lib/$ldlib"
echo "[ok] linked .venv/lib/$ldlib -> $src"
echo "Now run: uv run mjpython -m qc_warehouse.pick --viewer"
