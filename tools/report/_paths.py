# Tier: D
"""Puts tools/parts on sys.path (its modules import each other by flat name) and gives the repo root."""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
PARTS = REPO / "tools" / "parts"
if str(PARTS) not in sys.path:
    sys.path.insert(0, str(PARTS))
