import os
import sys
from pathlib import Path

# Never grab the real camera or touch the real database from tests.
os.environ["CAMERA_AUTOSTART"] = "0"
os.environ.setdefault("DB_PATH", str(Path(__file__).parent / ".test-default.db"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
