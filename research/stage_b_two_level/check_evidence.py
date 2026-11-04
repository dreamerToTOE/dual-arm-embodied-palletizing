#!/usr/bin/env python3
"""Check study evidence integrity and documented comparisons."""
import json
from pathlib import Path


root = Path(__file__).resolve().parent / "evidence"
signature = None
recorded = {}
for seed in (28, 29, 30):
    folder = root / f"occupancy_seed_{seed}"
    data = json.loads((folder / "results.json").read_text())
    assert data["ompl_seed"] == seed
    rows = data["results"]
    recorded[seed] = {row["case"]: row for row in rows}
    assert len(rows) == 20 and len({row["case"] for row in rows}) == 20
    invalid = [row for row in rows if row["case"] == "front_before_rear"]
    assert len(invalid) == 1 and all(invalid[0][k]["result"] == "INVALID_ORDER" for k in ("fixed_m0", "rule_m1", "two_level_m2"))
    valid = [row for row in rows if row not in invalid]
    assert [sum(row[k]["result"] == "PASS" for row in valid) for k in ("fixed_m0", "rule_m1", "two_level_m2")] == [7, 18, 18]
    current = [(r["case"], *(r[k]["result"] for k in ("fixed_m0", "rule_m1", "two_level_m2"))) for r in rows]
    assert signature is None or current == signature
    signature = current
    assert sum(len(v) for v in data["candidate_results"].values()) == 80
    for case, candidates in data["candidate_results"].items():
        for candidate in candidates:
            log = folder / "cases" / case / f"{candidate['arm']}_{candidate['sequence']}_gap{candidate['gap_mm']}.log"
            content = log.read_text()
            assert "Error retrieving file" not in content
            assert "has visual geometry but no collision geometry" not in content
            assert f"YAW_CANDIDATE_{candidate['result']}" in content
    assert not list(folder.rglob("STAGE_B_TWO_LEVEL_ADVISOR_REPORT_ZH.docx"))
timings = json.loads((root / "timing_methods/timings.json").read_text())
assert timings["workers"] == 8 and len(timings["results"]) == 171
for row in timings["results"]:
    key = {"fixed": "fixed_m0", "rule": "rule_m1", "two_level": "two_level_m2"}[row["method"]]
    assert row["result"] == recorded[row["seed"]][row["case"]][key]["result"]
assert "reason=initial Cube/world collision" in (root / "negative_control/planner.log").read_text()
assert "reason=unexpected occupancy id" in (root / "negative_control/invalid_id.log").read_text()
print("Evidence checks passed for three seeds; invalid-order gate, candidate logs, and model-loading errors checked.")
