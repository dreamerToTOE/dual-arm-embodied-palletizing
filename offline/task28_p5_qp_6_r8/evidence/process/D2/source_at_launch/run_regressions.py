"""R8: rerun the 1015 pre-R7 tests and record the four R7 tests separately."""

import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time


ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent / "evidence" / "regressions"
PREVIOUS = ROOT / "offline/task28_p5_qp_6_r4/evidence/regressions"
SPECS = [
    ("historical", "historical", 789),
    ("qp4", "qp4", 53),
    ("qp5", "qp5", 5),
    ("qp6_r1", "qp6_r1_retry", 30),
    ("qp6_r2", "qp6_r2", 12),
    ("qp6_r3", "qp6_r3", 7),
    ("qp6", "qp6_retry", 54),
    ("p5_r1", "r1", 6),
    ("p5_r2", "r2", 22),
    ("r4_focused", None, 12),
    ("r5_focused", None, 13),
    ("r6_focused", None, 12),
]


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    prior_env = json.loads((PREVIOUS / "historical.json").read_text())["PYTHONPATH"]
    env_script = Path("/tmp/r6_pytest_env.sh")
    if env_script.exists():
        recorded = env_script.read_text().strip().removeprefix("export PYTHONPATH=")
        if recorded != prior_env:
            raise RuntimeError("R6 environment script differs from archived environment")
    seen_paths = set()
    rows = []
    for name, metadata_name, expected in SPECS:
        if metadata_name:
            old = json.loads((PREVIOUS / f"{metadata_name}.json").read_text())
            argv = old["argv"]
            pythonpath = old.get("PYTHONPATH", prior_env)
        else:
            revision = name.split("_")[0]
            argv = ["/usr/bin/python3", "-m", "pytest",
                    f"offline/task28_p5_qp_6_{revision}/tests", "-q", "-p",
                    "no:cacheprovider", "--import-mode=importlib"]
            pythonpath = prior_env
        paths = [p for p in argv if p.startswith("offline/") and p.endswith("/tests")]
        if seen_paths.intersection(paths):
            raise RuntimeError("Duplicate test directory in regression suites")
        seen_paths.update(paths)
        env = os.environ.copy()
        env.update(PYTHONPATH=pythonpath, PYTHONDONTWRITEBYTECODE="1")
        start = time.monotonic()
        process = subprocess.run(argv, cwd=ROOT, env=env, text=True,
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        elapsed = time.monotonic() - start
        output = process.stdout
        (OUT / f"{name}.log").write_text(output)
        counts = re.findall(r"(?:^|\s)(\d+) passed", output)
        passed = int(counts[-1]) if counts else 0
        row = {"name": name, "argv": argv, "cwd": str(ROOT),
               "PYTHONPATH": pythonpath, "PYTHONDONTWRITEBYTECODE": "1",
               "exit_code": process.returncode, "elapsed_s": elapsed,
               "passed": passed, "expected": expected,
               "test_directories": paths,
               "log_sha256": hashlib.sha256(output.encode()).hexdigest(),
               "pass": process.returncode == 0 and passed == expected}
        (OUT / f"{name}.json").write_text(json.dumps(row, indent=2) + "\n")
        rows.append(row)
        print(f"{name}: exit={process.returncode} passed={passed}/{expected}", flush=True)
    total = sum(row["passed"] for row in rows)
    ok = all(row["pass"] for row in rows) and total == 1015
    summary = {"status": "PASS" if ok else "FAIL", "total_passed": total,
               "expected_total": 1015, "no_duplicate_test_directories": True,
               "r7_tests_included": False, "physics_launched": False,
               "suites": rows}
    argv = ["/usr/bin/python3", "-m", "pytest", "offline/task28_p5_qp_6_r7/tests", "-q", "-p", "no:cacheprovider", "--import-mode=importlib"]
    env = os.environ.copy()
    env.update(PYTHONPATH=str(ROOT) + os.pathsep + prior_env, PYTHONDONTWRITEBYTECODE="1")
    start = time.monotonic()
    process = subprocess.run(argv, cwd=ROOT, env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    output = process.stdout
    (OUT / "r7_focused.log").write_text(output)
    counts = re.findall(r"(?:^|\s)(\d+) passed", output)
    passed = int(counts[-1]) if counts else 0
    r7 = {"name": "r7_focused", "argv": argv, "cwd": str(ROOT), "PYTHONPATH": env["PYTHONPATH"], "exit_code": process.returncode, "elapsed_s": time.monotonic()-start, "passed": passed, "expected": 4, "included_in_previous_1015": False, "log_sha256": hashlib.sha256(output.encode()).hexdigest(), "pass": process.returncode == 0 and passed == 4}
    (OUT / "r7_focused.json").write_text(json.dumps(r7, indent=2) + "\n")
    print(f"r7_focused (separate): exit={process.returncode} passed={passed}/4", flush=True)
    summary["r7_focused_separate"] = r7
    summary["all_requested_historical_pass"] = ok and r7["pass"]
    summary["new_r8_tests_included"] = False
    (OUT / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    return 0 if summary["all_requested_historical_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
