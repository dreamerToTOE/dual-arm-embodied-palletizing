#!/usr/bin/env python3
"""Measure separate fixed, rule, and exhaustive policies with the same scenario pool."""
import json
import multiprocessing as mp
import os
from pathlib import Path
import statistics
import sys
import time

from run_study import RULE_THRESHOLD_DEG, candidate_task, precontact_task, workers, yaw_deg


def run_case(args):
    build, urdf, srdf, input_dir, output, method, seed, env = args
    fixture_path = input_dir / "fixture.json"
    occupancy_path = input_dir / "occupancy.json"
    fixture = json.loads(fixture_path.read_text())
    occupancy = json.loads(occupancy_path.read_text())
    yaw = abs(yaw_deg(fixture))
    rule_arm = "right" if fixture["cube_xyz_m"][1] <= -.20 or occupancy else "left"
    # The rule uses rear occupancy, which is equivalent to nonempty occupancy in this grid.
    arms = ("left", "right") if method == "two_level" else (("left",) if method == "fixed" else (rule_arm,))
    sequences = ("direct", "correct") if method == "two_level" else (("correct",) if method == "rule" and yaw > RULE_THRESHOLD_DEG else ("direct",))
    gaps = (1, 2) if method == "two_level" else (1,)
    output.mkdir(parents=True)
    start = time.monotonic()
    pres = []
    candidates = []
    for arm in arms:
        pre = output / f"precontact_{arm}.csv"
        p = precontact_task((build, urdf, srdf, fixture_path, pre, arm, env))
        pres.append(p)
        if p["result"] == "ERROR":
            raise RuntimeError(f"precontact error: {p}")
        if p["result"] != "PASS":
            continue
        for seq in sequences:
            for gap in gaps:
                path = output / f"{arm}_{seq}_gap{gap}.csv"
                c = candidate_task((build, urdf, srdf, fixture_path, pre, occupancy_path, path, arm, seq, gap, env))
                if c["result"] == "ERROR":
                    raise RuntimeError(f"candidate error: {c}")
                candidates.append(c)
    passed = [c for c in candidates if c["result"] == "PASS"]
    return {"case": input_dir.name, "method": method, "seed": seed,
            "result": "PASS" if passed else ("INCONCLUSIVE" if method == "two_level" and not candidates else "REJECT"),
            "elapsed_s": round(time.monotonic() - start, 4), "precontact_count": len(pres),
            "candidate_count": len(candidates), "candidate_pass_count": len(passed)}


def main():
    if len(sys.argv) != 5:
        raise SystemExit("usage: time_methods.py BUILD_DIR URDF SRDF NEW_OUTPUT_DIR")
    build, urdf, srdf, output = map(Path, sys.argv[1:])
    output.mkdir(parents=True, exist_ok=False)
    inputs = Path(__file__).resolve().parent / "evidence/occupancy_seed_28/cases"
    case_dirs = sorted(p for p in inputs.iterdir() if p.is_dir() and p.name != "front_before_rear")
    env = os.environ.copy()
    env.update({"ROS_DOMAIN_ID": "182", "ROS_LOCALHOST_ONLY": "1", "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp"})
    nworkers = workers()
    results = []
    for seed in (28, 29, 30):
        run_env = dict(env, STAGE_B_SEED=str(seed))
        # Rotate method order to limit systematic warm-cache bias.
        methods = ("fixed", "rule", "two_level")
        offset = seed - 28
        for method in methods[offset:] + methods[:offset]:
            jobs = [(build, urdf, srdf, case, output / f"seed_{seed}" / method / case.name,
                     method, seed, run_env) for case in case_dirs]
            with mp.Pool(nworkers) as pool:
                rows = list(pool.imap_unordered(run_case, jobs))
            results.extend(rows)
            times = sorted(row["elapsed_s"] for row in rows)
            print(json.dumps({"seed": seed, "method": method, "passed": sum(r["result"] == "PASS" for r in rows),
                              "median_s": statistics.median(times), "max_s": max(times)}, ensure_ascii=False), flush=True)
    (output / "timings.json").write_text(json.dumps({"workers": nworkers, "cases": len(case_dirs),
                                                      "results": results}, indent=2) + "\n")


if __name__ == "__main__":
    main()
