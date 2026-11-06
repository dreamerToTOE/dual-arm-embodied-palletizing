#!/usr/bin/env python3
"""One bounded static Isaac probe for a physically feasible larger top marker."""
from __future__ import annotations

import argparse
from pathlib import Path

from research.stage_b_vision_contact_r2b.scripts import r2b_cal24_transfer_probe as probe


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--marker-size-m", type=float, default=0.09)
    parser.add_argument("--quiet-size-m", type=float, default=0.108)
    parser.add_argument("--run-id", default="r2b_cube_marker_90mm_probe01")
    parser.add_argument("--analyze-only", type=Path)
    args = parser.parse_args()
    if not 0.0 < args.marker_size_m < args.quiet_size_m <= probe.CUBE_SIZE_M:
        raise ValueError("marker and quiet backing must fit inside the 120 mm Cube face")
    probe.MARKER_SIZE_M = float(args.marker_size_m)
    probe.QUIET_SIZE_M = float(args.quiet_size_m)
    probe.MARKER_FRONT_OFFSET_M = probe.CUBE_SIZE_M / 2.0 + probe.CELL_THICKNESS_M
    output = args.analyze_only or probe._capture_run(args.run_id)
    probe._analyze(output)


if __name__ == "__main__":
    main()
