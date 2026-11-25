import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / "src"
for package in ("go2_follower_perception", "go2_follower_control"):
    sys.path.insert(0, str(ROOT / package))
