"""Run a small, registered offline QP-2 to position-target handoff batch."""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
from pathlib import Path

from .handoff import run_offline_handoff, HandoffContract

ROOT = Path(__file__).resolve().parents[2]
QP2_REG = ROOT / "offline/task28_p5_qp_2/evidence/preregistration.json"


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    out = args.output
    if out.exists():
        raise SystemExit("REFUSE_EXISTING_OUTPUT_DIRECTORY")
    out.mkdir(parents=True)
    reg = json.loads(QP2_REG.read_text())
    node = next(x for x in reg["nodes"] if x["identity"] == "seed28:transport:midpoint40")
    negative = reg["penetration_control"]
    neg_path = ROOT / negative["source_path"]
    if sha(neg_path) != negative["source_sha256"]:
        raise SystemExit("SAVED_NEGATIVE_SOURCE_HASH_MISMATCH")
    saved_negative = next(x for x in json.loads(gzip.decompress(neg_path.read_bytes()))
                          if x["case"] == negative["source_case"])
    contract = HandoffContract()
    positive = run_offline_handoff(
        node["q14"], node["object_pose"], [0.005, -0.003, 0.002, 0.01, -0.005, 0.02],
        dt_s=0.02, max_steps=5, identity="task28_qp3_seed28_transport40_5step",
        contract=contract)
    negative_result = run_offline_handoff(
        negative["q14"], negative["object_pose"], negative["desired_object_twist"],
        dt_s=negative["dt_s"], max_steps=negative["max_steps"],
        identity=negative["identity"], contract=contract)
    records = {"positive": positive, "negative_payload_table": negative_result}
    (out / "handoff_records.json").write_text(json.dumps(records, indent=2, sort_keys=True))
    static = {
        "schema": "task28.g2_p5_qp_3.static_interface_contract.v1",
        "execution_authorized": False,
        "fixture": "fixtures/task28_g1_0_new_single_cube_v0",
        "source_files": [
            "docs/tasks/TASK28_G0A_INTERFACE_AUDIT.md",
            "docs/tasks/TASK28_PAPER_TO_MAIN_RESEARCH_ROADMAP.md",
            "isaac/scripts/task24_side_suction_tight_bridge.py",
            "ros_ws/src/fr3_dual_palletize/src/task25_batched_side_suction.cpp",
            "fixtures/task28_g1_0_new_single_cube_v0/bridge/task28_single_cube_bridge.py",
            "fixtures/task28_g1_0_new_single_cube_v0/bridge/task28_g1_0_bridge_contract.json",
            "offline/task28_p5_qp_2/feedback.py",
            "offline/task28_p5_qp_2/driver.py",
            "offline/task28_p4_nr_0/evidence/experiments_v1/seed_28/result.json.gz",
        ],
        "interface_facts": {
            "object_path_source": "saved NR-0 seed28 transport midpoint40; no new NR or waypoint replacement",
            "object_time_parameterization": "not present in NR-0; QP3 uses explicitly registered synthetic dt=0.02 s",
            "position_adapter": "q_next=q+dt*qdot; split 14-vector into left/right 7-vector position targets in memory",
            "joint_order": list(contract.left_joint_names + contract.right_joint_names),
            "command_topic_shape": "/left/joint_command and /right/joint_command sensor_msgs/JointState.position in historical sink",
            "dual_sync_contract": "same stamp and frame_id task28_g1_0_dual_sync; target pair is atomically represented but never published",
            "source_feedback": "QP2 predicted q_next and G1/B0 synthetic state diagnostics; no actuator feedback or R1 observation",
            "failure_policy": "first rejected/inconclusive step retained and no later candidate is generated",
            "nominal_period": "historical code contains 10 ms sleep/period constants; measured servo/physics period remains UNKNOWN",
            "position_only": True,
            "velocity_field_connected": False,
            "collision_avoidance_in_qp": False,
        },
        "hashes": {p: sha(ROOT / p) for p in [
            "fixtures/task28_g1_0_new_single_cube_v0/bridge/task28_single_cube_bridge.py",
            "fixtures/task28_g1_0_new_single_cube_v0/bridge/task28_g1_0_bridge_contract.json",
            "offline/task28_p5_qp_2/feedback.py", "offline/task28_p5_qp_2/driver.py",
            "offline/task28_p4_nr_0/evidence/experiments_v1/seed_28/result.json.gz",
        ]},
    }
    (out / "static_interface_contract.json").write_text(json.dumps(static, indent=2, sort_keys=True))
    summary = {
        "schema": "task28.g2_p5_qp_3.handoff_summary.v1",
        "execution_authorized": False,
        "positive_status": positive["status"],
        "positive_target_pairs": len(positive["position_targets"]),
        "negative_status": negative_result["status"],
        "negative_target_pairs": len(negative_result["position_targets"]),
        "negative_reasons": negative_result["rejections"],
        "positive_all_contract_checks": all(t["contract_check"]["valid"] for t in positive["position_targets"]),
        "positive_all_predicted_states_valid": all(t["predicted_state_valid"] for t in positive["position_targets"]),
        "positive_all_sampled_edges_valid": all(t["sampled_edge_status"] == "sampled_edge_valid" for t in positive["position_targets"]),
        "no_commands_published": True,
        "physical_feedback_measured": False,
        "real_time_verified": False,
        "continuous_safety": "NOT_CERTIFIED_FINITE_SAMPLING_ONLY",
        "stop_after": "G2-P5-QP-3",
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True))
    print(json.dumps(summary, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
