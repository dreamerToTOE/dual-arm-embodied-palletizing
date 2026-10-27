"""Audit the sealed QP-2 offline evidence without rerunning a controller."""
from __future__ import annotations
import argparse, gzip, hashlib, json, platform, sys
from pathlib import Path


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--evidence", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    root = args.evidence
    summary = json.loads((root / "summary.json").read_text())
    config = json.loads((root / "registered_config.json").read_text())
    cases = list(root.glob("case_*.json.gz"))
    cases.sort()
    checks: dict[str, bool] = {}
    details: dict[str, object] = {}
    checks["summary_schema"] = summary.get("schema") == "task28.g2_p5_qp_2.experiment_summary.v1"
    checks["execution_disabled"] = summary.get("execution_authorized") is False and all(
        c.get("execution_authorized") is False for c in summary.get("cases", [])
    )
    checks["finite_sampling_boundary"] = summary.get("continuous_safety") == "NOT_CERTIFIED_FINITE_SAMPLING_ONLY"
    checks["expected_artifact_count"] = len(cases) == len(summary.get("artifacts", [])) == len(summary.get("cases", [])) == 32
    checks["pair_count"] = len(summary.get("pairs", [])) == 15
    checks["registered_config_matches"] = config.get("schema") == "task28.g2_p5_qp_2.preregistration.v1"
    checks["no_new_calls"] = summary.get("new_NR_IK_or_reprojection_calls") == 0 and summary.get("new_R1_observations") == 0
    artifact_hashes = []
    identity_index = {(x["identity"], x["artifact"]): x for x in summary.get("cases", [])}
    artifact_index = {(x["identity"], x["path"]): x for x in summary.get("artifacts", [])}
    mismatches = []
    for path in cases:
        raw = path.read_bytes()
        try:
            record = json.loads(gzip.decompress(raw))
        except Exception as exc:
            mismatches.append({"file": path.name, "reason": f"gzip/json:{exc}"})
            continue
        req = record.get("request", {})
        # The controller label (baseline/feedback) is in the summary; the
        # saved bounded record intentionally uses one common record method.
        key = (req.get("identity"), path.name)
        expected = identity_index.get(key)
        registered = artifact_index.get(key)
        artifact_hashes.append({"path": path.name, "sha256": sha256_bytes(raw), "bytes": len(raw), "identity": req.get("identity"), "method": record.get("method")})
        if expected is None or registered is None:
            mismatches.append({"file": path.name, "reason": "missing summary/artifact identity", "key": key})
            continue
        if expected.get("status") != record.get("status") or expected.get("accepted_steps") != record.get("accepted_step_count") or expected.get("attempted_steps") != record.get("attempted_step_count"):
            mismatches.append({"file": path.name, "reason": "summary_record_mismatch"})
        if registered.get("path") != path.name or registered.get("sha256") != sha256_bytes(raw):
            mismatches.append({"file": path.name, "reason": "artifact_hash_mismatch"})
        if record.get("execution_authorized") is not False or record.get("continuous_safety") != "NOT_CERTIFIED_FINITE_SAMPLING_ONLY":
            mismatches.append({"file": path.name, "reason": "safety_or_execution_flag_mismatch"})
        if record.get("nr_projection_performed") is not False or record.get("phase_confirmation_observations_generated") != 0:
            mismatches.append({"file": path.name, "reason": "forbidden_runtime_path_flag"})
        steps = record.get("steps", [])
        if sum(1 for step in steps if step.get("accepted") is True) != record.get("accepted_step_count"):
            mismatches.append({"file": path.name, "reason": "accepted_step_count_mismatch"})
        for step in steps:
            if step.get("execution_authorized") is not False:
                mismatches.append({"file": path.name, "reason": "step_execution_flag_mismatch", "step": step.get("step_index")})
        rejected = [step for step in steps if step.get("accepted") is False]
        if len(rejected) > 1 or (rejected and rejected[-1] is not steps[-1]):
            mismatches.append({"file": path.name, "reason": "rejection_continuation_or_multiple_failures"})
    checks["record_identity_and_hashes"] = not mismatches
    details["mismatches"] = mismatches
    details["artifact_hashes"] = artifact_hashes
    neg = [c for c in summary.get("cases", []) if c.get("purpose") == "current_state_FCL_negative_not_reachable_path"]
    checks["payload_negative_both_rejected"] = len(neg) == 2 and all(
        c.get("status") == "STOPPED_AT_FIRST_REJECTION" and c.get("attempted_steps") == 0 and c.get("accepted_steps") == 0
        for c in neg
    )
    checks["primary_requirements"] = all(summary.get("experiment_requirements", {}).values())
    details["negative_controls"] = [{"identity": c.get("identity"), "method": c.get("method"), "status": c.get("status"), "failure": c.get("failure")} for c in neg]
    details["summary_status"] = summary.get("status")
    details["environment"] = summary.get("environment")
    details["audit_command"] = "PYTHONDONTWRITEBYTECODE=1 python3 offline/task28_p5_qp_2/audit_saved_evidence.py --evidence offline/task28_p5_qp_2/evidence/experiments_v1 --output offline/task28_p5_qp_2/evidence/evidence_audit.json"
    details["python"] = sys.version
    details["platform"] = platform.platform()
    result = {"schema": "task28.g2_p5_qp_2.evidence_audit.v1", "status": "PASS" if all(checks.values()) else "FAIL", "execution_authorized": False, "checks": checks, "details": details}
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"status": result["status"], "checks": checks, "artifact_count": len(artifact_hashes), "mismatch_count": len(mismatches)}))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
