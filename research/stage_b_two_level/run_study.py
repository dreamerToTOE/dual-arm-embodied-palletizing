#!/usr/bin/env python3
"""CPU-only two-level candidate study for Stage B.

Every candidate is an offline MoveIt/FCL validity check.  No controller, simulator,
force observation, or physical contact is involved.
"""
from __future__ import annotations

import hashlib
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import re
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[2]
PRIOR = ROOT / "research/stage_b_selector/evidence/pilot_v1"
BASE_FIXTURE = PRIOR / "S2_aligned_control/fixture.json"
YAW_CASES = {
    "yaw_0": "S2_aligned_control",
    "yaw_m2": "S2_yaw_-2deg",
    "yaw_p2": "S2_yaw_+2deg",
    "yaw_m4": "S2_yaw_-4deg",
    "yaw_p4": "S2_yaw_+4deg",
    "yaw_m6": "S2_yaw_-6deg",
    "yaw_p6": "S2_yaw_+6deg",
}
WALL_CASES = {
    "wall_m235": "S3_y_-0.235",
    "wall_m220": "S3_y_-0.220",
    "wall_p220": "S3_y_+0.220",
    "wall_p235": "S3_y_+0.235",
}
RULE_THRESHOLD_DEG = math.degrees(math.asin((1 + .003 / .06) / math.sqrt(2)) - math.pi / 4)


def workers() -> int:
    cpu = os.cpu_count() or 1
    available_kib = 0
    try:
        for line in Path("/proc/meminfo").read_text().splitlines():
            if line.startswith("MemAvailable:"):
                available_kib = int(line.split()[1])
                break
    except OSError:
        pass
    # Each MoveIt process is small, but a bounded half-CPU pool avoids timing noise.
    return max(2, min(8, cpu // 2, max(2, available_kib // (2 * 1024 * 1024))))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def fixture_for(case_id: str, base: dict) -> tuple[dict, list[dict], dict]:
    fixture = json.loads(json.dumps(base))
    occupancy: list[dict] = []
    metadata = {"case_id": case_id, "target_row": "rear", "rear_loaded": 0}
    if case_id in YAW_CASES:
        source = json.loads((PRIOR / YAW_CASES[case_id] / "fixture.json").read_text())
        fixture["cube_xyz_m"] = source["cube_xyz_m"]
        fixture["cube_quaternion_wxyz"] = source["cube_quaternion_wxyz"]
        metadata["source"] = YAW_CASES[case_id]
    elif case_id in WALL_CASES:
        source = json.loads((PRIOR / WALL_CASES[case_id] / "fixture.json").read_text())
        fixture["cube_xyz_m"] = source["cube_xyz_m"]
        fixture["cube_quaternion_wxyz"] = source["cube_quaternion_wxyz"]
        metadata["source"] = WALL_CASES[case_id]
    elif case_id.startswith("combined_"):
        parts = case_id.split("_")
        fixture["cube_xyz_m"][1] = -.220 if parts[1] == "wall" else 0.0
        angle = -4 if parts[2] == "m4" else 4
        fixture["cube_quaternion_wxyz"] = [math.cos(math.radians(angle)/2), 0, 0, math.sin(math.radians(angle)/2)]
        if parts[1] == "occupied":
            metadata["rear_loaded"] = 1
            occupancy.append({"id": "occupied_cube_0", "xyz_m": [.98, -.13, fixture["cube_xyz_m"][2]], "size_m": [.12, .12, .12]})
    elif case_id == "front_after_rear":
        fixture["cube_xyz_m"][0] = .94
        fixture["cube_xyz_m"][1] = 0.0
        metadata.update({"target_row": "front", "rear_loaded": 3, "predecessor": ["rear_minus_y", "rear_center", "rear_plus_y"]})
        for i, y in enumerate((-.13, 0, .13)):
            occupancy.append({"id": f"occupied_cube_{i}", "xyz_m": [.98, y, fixture["cube_xyz_m"][2]], "size_m": [.12, .12, .12]})
    elif case_id.startswith("rear_loaded_"):
        # Three lateral rear slots; target remains in the middle slot.
        fixture["cube_xyz_m"][1] = 0.0
        count = int(case_id.rsplit("_", 1)[1])
        metadata["rear_loaded"] = count
        metadata["predecessor"] = ["rear_minus_y", "rear_plus_y"][:count]
        for i, y in enumerate((-.13, .13)[:count]):
            # Fixture coordinates include the +0.1 m rail/world shift; planner occupancy is in planning frame.
            occupancy.append({"id": f"occupied_cube_{i}", "xyz_m": [.98, y, fixture["cube_xyz_m"][2]],
                              "size_m": [.12, .12, .12]})
    elif case_id == "front_before_rear":
        fixture["cube_xyz_m"][1] = 0.0
        metadata.update({"target_row": "front", "rear_loaded": 0, "order_gate": "rear_must_be_complete"})
    else:
        raise ValueError(case_id)
    return fixture, occupancy, metadata


def yaw_deg(fixture: dict) -> float:
    q = fixture["cube_quaternion_wxyz"]
    return math.degrees(2 * math.atan2(q[3], q[0]))


def run_process(cmd: list[str], env: dict[str, str], log_path: Path, timeout: int = 90) -> dict:
    started = time.monotonic()
    try:
        proc = subprocess.run(cmd, env=env, text=True, capture_output=True, timeout=timeout)
        output, return_code = proc.stdout + proc.stderr, proc.returncode
    except subprocess.TimeoutExpired as exc:
        output = (exc.stdout or "") if isinstance(exc.stdout, str) else (exc.stdout or b"").decode(errors="replace")
        output += "\nTIMEOUT\n"
        return_code = None
    elapsed = round(time.monotonic() - started, 4)
    log_path.write_text(output)
    if "Error retrieving file" in output or "has visual geometry but no collision geometry" in output:
        return {"return_code": return_code, "result": "ERROR", "phase": "model_load",
                "detail": "robot collision mesh not loaded", "elapsed_s": elapsed}
    match = re.search(r"(?:PRECONTACT|YAW_CANDIDATE)_(PASS|REJECT)(?:\s+[^\n]*)?", output)
    result = match.group(1) if match else "ERROR"
    phase = None
    detail = None
    reject = re.search(r"YAW_CANDIDATE_REJECT phase=([^ ]+) reason=(.*?)(?: samples=|$)", output)
    if reject:
        phase, detail = reject.group(1), reject.group(2)
    metrics = {}
    for name in ("samples", "ik_attempt", "ik_attempts", "joint_length_rad", "path_length_rad", "seconds"):
        value = re.search(r"\b" + name + r"=([0-9.eE+-]+)", output)
        if value:
            metrics[name] = float(value.group(1))
    return {"return_code": return_code, "result": result, "phase": phase,
            "detail": detail, "elapsed_s": elapsed, "metrics": metrics}


def precontact_task(task: tuple) -> dict:
    (build, urdf, srdf, fixture_path, out_path, arm, env) = task
    executable = build / ("precontact_left" if arm == "left" else "precontact_right")
    result = run_process([str(executable), str(urdf), str(srdf), str(fixture_path), str(out_path)], env, out_path.with_suffix(".log"))
    result.update({"arm": arm, "path": str(out_path)})
    return result


def candidate_task(task: tuple) -> dict:
    (build, urdf, srdf, fixture_path, precontact_path, occupancy_path, out_path,
     arm, sequence, gap_mm, env) = task
    executable = build / ("yaw_planner_left" if arm == "left" else "yaw_planner_right")
    result = run_process([
        str(executable), str(urdf), str(srdf), str(fixture_path), str(precontact_path),
        str(out_path), "0", sequence, str(occupancy_path), str(gap_mm)
    ], env, out_path.with_suffix(".log"))
    result.update({"arm": arm, "sequence": sequence, "gap_mm": gap_mm, "trajectory": str(out_path)})
    return result


def main() -> None:
    if len(sys.argv) not in (5, 6):
        raise SystemExit("usage: run_study.py BUILD_DIR URDF SRDF OUTPUT_DIR [SEED]")
    build, urdf, srdf, output = map(Path, sys.argv[1:5])
    seed = int(sys.argv[5]) if len(sys.argv) == 6 else 28
    if not 1 <= seed <= 2**32 - 1:
        raise ValueError("seed must be a positive 32-bit integer")
    output.mkdir(parents=True, exist_ok=False)
    env = os.environ.copy()
    env.update({"ROS_DOMAIN_ID": "182", "ROS_LOCALHOST_ONLY": "1", "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp", "STAGE_B_SEED": str(seed)})
    worker_count = workers()
    base = json.loads(BASE_FIXTURE.read_text())
    case_ids = list(YAW_CASES) + list(WALL_CASES) + ["rear_loaded_0", "rear_loaded_1", "rear_loaded_2", "front_after_rear", "front_before_rear", "combined_wall_m4", "combined_wall_p4", "combined_occupied_m4", "combined_occupied_p4"]
    cases: dict[str, dict] = {}
    precontact_jobs = []
    for case_id in case_ids:
        fixture, occupancy, metadata = fixture_for(case_id, base)
        fixture["occupied_boxes"] = occupancy
        case_dir = output / "cases" / case_id
        case_dir.mkdir(parents=True)
        fixture_path = case_dir / "fixture.json"
        occupancy_path = case_dir / "occupancy.json"
        fixture_path.write_text(json.dumps(fixture, indent=2) + "\n")
        occupancy_path.write_text(json.dumps(occupancy, indent=2) + "\n")
        cases[case_id] = {"fixture": fixture, "fixture_path": fixture_path, "occupancy_path": occupancy_path,
                          "metadata": metadata, "yaw_deg": yaw_deg(fixture), "precontact": {}}
        if metadata.get("order_gate") != "rear_must_be_complete":
            for arm in ("left", "right"):
                path = case_dir / f"precontact_{arm}.csv"
                precontact_jobs.append((build, urdf, srdf, fixture_path, path, arm, env))

    with mp.Pool(worker_count) as pool:
        for result in pool.imap_unordered(precontact_task, precontact_jobs):
            case_id = Path(result["path"]).parent.name
            cases[case_id]["precontact"][result["arm"]] = result
    if any(r["result"] == "ERROR" for c in cases.values() for r in c["precontact"].values()):
        raise RuntimeError("precontact process error: inspect logs, especially robot collision mesh loading")

    candidates = []
    for case_id, data in cases.items():
        if data["metadata"].get("order_gate") == "rear_must_be_complete":
            continue
        for arm in ("left", "right"):
            if data["precontact"].get(arm, {}).get("result") != "PASS":
                continue
            pre = Path(data["precontact"][arm]["path"])
            for sequence in ("direct", "correct"):
                for gap in (1, 2):
                    path = output / "cases" / case_id / f"{arm}_{sequence}_gap{gap}.csv"
                    candidates.append((build, urdf, srdf, data["fixture_path"], pre, data["occupancy_path"],
                                       path, arm, sequence, gap, env))
    candidate_results: dict[str, list[dict]] = {case_id: [] for case_id in cases}
    with mp.Pool(worker_count) as pool:
        for result in pool.imap_unordered(candidate_task, candidates):
            case_id = Path(result["trajectory"]).parent.name
            candidate_results[case_id].append(result)
    if any(r["result"] == "ERROR" for rows in candidate_results.values() for r in rows):
        raise RuntimeError("candidate process error: inspect logs, especially robot collision mesh loading")

    def selected(case_id: str, method: str) -> dict:
        data = cases[case_id]
        if data["metadata"].get("order_gate") == "rear_must_be_complete":
            return {"result": "INVALID_ORDER", "reason": "rear row not complete", "elapsed_s": 0.0}
        yaw = abs(data["yaw_deg"])
        sequence = "direct" if method == "fixed" or (method == "rule" and yaw <= RULE_THRESHOLD_DEG) else "correct"
        # A transparent baseline uses the opposite arm at the negative wall or with a minus-Y neighbor.
        arm = "right" if method == "rule" and (data["fixture"]["cube_xyz_m"][1] <= -.20 or data["metadata"].get("rear_loaded", 0) > 0) else "left"
        rows = [r for r in candidate_results[case_id] if r["arm"] == arm and r["sequence"] == sequence and r["gap_mm"] == 1]
        if not rows:
            pre = data["precontact"].get(arm, {})
            return {"result": "REJECT" if pre.get("result") == "REJECT" else "INCONCLUSIVE", "reason": f"{arm} pre-contact: {pre.get('result', 'unavailable')}", "elapsed_s": pre.get("elapsed_s", 0.0), "arm": arm}
        row = rows[0].copy()
        row["method_sequence"] = sequence
        return row

    def two_level(case_id: str) -> dict:
        data = cases[case_id]
        if data["metadata"].get("order_gate") == "rear_must_be_complete":
            return {"result": "INVALID_ORDER", "reason": "rear row not complete", "elapsed_s": 0.0}
        rows = candidate_results[case_id]
        if not rows:
            return {"result": "INCONCLUSIVE", "reason": "no complete candidate was executable", "elapsed_s": 0.0}
        # The low-level evaluator supplies complete paths; rank by correction, arm switch, then gap.
        passing = [r for r in rows if r["result"] == "PASS"]
        passing.sort(key=lambda r: (r["sequence"] == "correct", r["arm"] == "right", r["gap_mm"], r["elapsed_s"]))
        chosen = (passing[0] if passing else min(rows, key=lambda r: (r["sequence"] == "correct", r["arm"] == "right", r["gap_mm"]))).copy()
        chosen["candidate_count"] = len(rows)
        chosen["candidate_pass_count"] = len(passing)
        chosen["search_elapsed_s"] = round(sum(r["elapsed_s"] for r in rows), 4)
        chosen["result"] = "PASS" if passing else "REJECT"
        return chosen

    summary = []
    for case_id in case_ids:
        fixed = selected(case_id, "fixed")
        rule = selected(case_id, "rule")
        level2 = two_level(case_id)
        level1 = "OBSERVED_SEQUENCE_SWITCH" if level2["result"] == "PASS" and level2.get("sequence") == "correct" else "NO_SEQUENCE_SWITCH_OBSERVED"
        level2_decision = "ADVANTAGE_OVER_RULE" if level2["result"] == "PASS" and rule["result"] != "PASS" else "NO_ADVANTAGE_OVER_RULE"
        summary.append({"case": case_id, "metadata": cases[case_id]["metadata"], "yaw_deg": cases[case_id]["yaw_deg"],
                        "fixed_m0": fixed, "rule_m1": rule, "two_level_m2": level2,
                        "level1_decision": level1, "level2_decision": level2_decision,
                        "physical_execution": "DEFERRED"})

    results = {
        "study": "stage_b_two_level_selector_feasibility",
        "scope": "CPU-only geometric/kinematic MoveIt/FCL candidate planning; no Isaac Sim, RL, force, or physical contact",
        "workers": worker_count,
        "cpu_count": os.cpu_count(),
        "mem_available_kib_at_start": next((int(x.split()[1]) for x in Path("/proc/meminfo").read_text().splitlines() if x.startswith("MemAvailable:")), None),
        "ompl_seed": seed,
        "rule_threshold_deg": RULE_THRESHOLD_DEG,
        "rule_arm": "right if target y <= -0.20 m or rear_loaded > 0; otherwise left",
        "reference_sha256": {"urdf": sha256(urdf), "srdf": sha256(srdf)},
        "precontact_results": {k: v["precontact"] for k, v in cases.items()},
        "candidate_results": candidate_results,
        "results": summary,
    }
    (output / "results.json").write_text(json.dumps(results, indent=2) + "\n")
    counts = {method: sum(row[f"{method}_m{0 if method == 'fixed' else 1 if method == 'rule' else 2}"]["result"] == "PASS" for row in summary)
              for method in ("fixed", "rule", "two_level")}
    print(json.dumps({"output": str(output), "workers": worker_count, "scenario_count": len(summary), "pass_counts": counts}, indent=2))


if __name__ == "__main__":
    main()
