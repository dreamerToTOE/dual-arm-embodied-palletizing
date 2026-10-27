"""Read-only integrity audit for the QP-3 handoff evidence."""
from __future__ import annotations
import argparse, hashlib, json
from pathlib import Path


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--evidence", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    root = args.evidence
    records = json.loads((root / "handoff_records.json").read_text())
    summary = json.loads((root / "summary.json").read_text())
    static = json.loads((root / "static_interface_contract.json").read_text())
    checks = {}
    checks["execution_disabled"] = summary.get("execution_authorized") is False and all(
        r.get("execution_authorized") is False for r in records.values())
    checks["positive_target_count"] = len(records["positive"]["position_targets"]) == 5
    checks["positive_contract_and_geometry"] = (
        records["positive"]["status"] == "COMPLETED_OFFLINE_SAMPLED_UNCERTIFIED"
        and all(t["contract_check"]["valid"] and t["predicted_state_valid"]
                and t["sampled_edge_status"] == "sampled_edge_valid"
                for t in records["positive"]["position_targets"]))
    checks["negative_initial_gate_stop"] = (
        records["negative_payload_table"]["status"] == "STOPPED_AT_FIRST_REJECTION"
        and not records["negative_payload_table"]["position_targets"]
        and any("INITIAL_STATE_GEOMETRY_REJECTED" in x["reasons"]
                for x in records["negative_payload_table"]["rejections"]))
    checks["no_ros_or_publish"] = summary.get("no_commands_published") is True
    checks["timing_boundary"] = summary.get("real_time_verified") is False and summary.get("physical_feedback_measured") is False
    checks["source_hashes"] = all(Path(path).is_file() and sha(Path(path)) == digest
                                   for path, digest in static["hashes"].items())
    checks["fixed_joint_contract"] = static["interface_facts"]["joint_order"] == [
        f"left_fr3_joint{i}" for i in range(1, 8)] + [f"right_fr3_joint{i}" for i in range(1, 8)]
    result = {"schema": "task28.g2_p5_qp_3.evidence_audit.v1",
              "status": "PASS" if all(checks.values()) else "FAIL",
              "execution_authorized": False, "checks": checks,
              "positive_identity": records["positive"]["identity"],
              "negative_identity": records["negative_payload_table"]["identity"],
              "continuous_safety": summary["continuous_safety"]}
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps(result, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
