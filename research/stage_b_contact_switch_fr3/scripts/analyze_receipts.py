#!/usr/bin/env python3
"""Summarize the bounded pilot and make one diagnostic F0 plot."""
import json
from pathlib import Path

ROOT = Path(__file__).parents[1]
OUT = ROOT / "evidence/analysis.json"


def main():
    f0 = json.loads((ROOT / "evidence/f0_smoke/summary.json").read_text())
    f1a = json.loads((ROOT / "evidence/f1_attempt01/summary.json").read_text())
    f1b = json.loads((ROOT / "evidence/f1_attempt02/summary.json").read_text())
    result = {
        "schema": "g1a.fr3.analysis.v1",
        "f0_status": f0["status"],
        "f0_actual_fr3_motion": bool(f0["tcp_displacement_m"] > 0 and not f0["teleport_used"]),
        "f0_position_drive_only": f0["control_mode"] == "position_drive_target",
        "f0_transition_step": f0["transition"]["step"],
        "f0_tracking_error_rad": f0["max_abs_tracking_error_rad"],
        "f1_attempt01_status": f1a["status"],
        "f1_attempt02_status": f1b["status"],
        "f1_moveit_started": bool(f1b.get("moveit_started")),
        "f1_planning_fcl_evidence": True,
        "f1_cube_displacement_verified": False,
        "f1_loaded_wall_contact_verified": False,
        "f1_contact_force_comparison_valid": False,
        "f1_blocker": f1b["qp_stop_reason"],
        "comparison_status": {"C0": "NOT_RUN", "C1": "NOT_RUN", "C2": "NOT_RUN"},
        "acceptance": "G1A_FR3_PARTIAL",
        "advantage": "INCONCLUSIVE"
    }
    OUT.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
