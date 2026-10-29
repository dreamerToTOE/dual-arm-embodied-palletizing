"""Re-run the frozen pre-R9 suites into R9 evidence without touching archives."""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent / "evidence" / "regressions"
sys.path.insert(0, str(ROOT))
from offline.task28_p5_qp_6_r8 import run_regressions as frozen


def run(argv, env, name, expected):
    start = time.monotonic()
    p = subprocess.run(argv, cwd=ROOT, env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    output = p.stdout
    (OUT / f"{name}.log").write_text(output)
    counts = re.findall(r"(?:^|\s)(\d+) passed", output)
    passed = int(counts[-1]) if counts else 0
    row = {"name": name, "argv": argv, "cwd": str(ROOT), "PYTHONPATH": env.get("PYTHONPATH"),
           "PYTHONDONTWRITEBYTECODE": "1", "exit_code": p.returncode,
           "elapsed_s": time.monotonic() - start, "passed": passed, "expected": expected,
           "log_sha256": hashlib.sha256(output.encode()).hexdigest(),
           "pass": p.returncode == 0 and passed == expected}
    (OUT / f"{name}.json").write_text(json.dumps(row, indent=2) + "\n")
    print(f"{name}: exit={p.returncode} passed={passed}/{expected}", flush=True)
    return row


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    # The R8 runner has the frozen, de-duplicated 1015 + separate R7=4 recipe.
    # Redirect its output only; it reads archived metadata and never mutates it.
    frozen.OUT = OUT / "pre_r8"
    frozen.main()
    pre = json.loads((frozen.OUT / "summary.json").read_text())
    prior_env = json.loads((ROOT / "offline/task28_p5_qp_6_r4/evidence/regressions/historical.json").read_text())["PYTHONPATH"]
    env = os.environ.copy()
    env.update(PYTHONPATH=prior_env, PYTHONDONTWRITEBYTECODE="1")
    r8 = run(["/usr/bin/python3", "-m", "pytest", "offline/task28_p5_qp_6_r8/tests", "-q", "-p", "no:cacheprovider", "--import-mode=importlib"], env, "r8_focused", 15)
    total = pre["total_passed"] + pre["r7_focused_separate"]["passed"] + r8["passed"]
    summary = {"schema": "TASK28_R9_PRIOR_REGRESSION_SUMMARY_V1", "status": "PASS" if pre["all_requested_historical_pass"] and r8["pass"] else "FAIL",
               "historical_1015": pre["total_passed"], "r7_focused": pre["r7_focused_separate"]["passed"], "r8_focused_passed": r8["passed"],
               "total_passed": total, "expected_total": 1034, "physics_launched": False,
               "source": "R8 frozen de-duplicated runner redirected to R9 evidence; R8 focused suite rerun separately",
               "pre_r8_summary": str(frozen.OUT / "summary.json"), "r8_focused_result": r8}
    (OUT / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({"status": summary["status"], "total_passed": total}, ensure_ascii=False))
    return 0 if summary["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
