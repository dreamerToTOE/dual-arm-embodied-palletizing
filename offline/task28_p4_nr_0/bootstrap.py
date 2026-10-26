"""Read-only import paths for the four accepted offline layers."""
from pathlib import Path
import sys
REPO=Path(__file__).resolve().parents[2]
for name in ('task28_g1_1','task28_g1_2a','task28_g1_2b_0','task28_g1_2b_1'):
    path=str(REPO/'offline'/name)
    if path not in sys.path:sys.path.insert(0,path)
