#!/usr/bin/env python3
"""Small receipt checks for the bounded G1-A-FR3 pilot."""
import json
from pathlib import Path

ROOT = Path(__file__).parents[1]


def test_f0_receipt():
    s = json.loads((ROOT / "evidence/f0_smoke/summary.json").read_text())
    assert s["status"] == "PASS"
    assert s["teleport_used"] is False
    assert s["contact_forces_used"] is False
    assert s["frames"] == 150
    assert s["transition"]["step"] == 90
    assert s["tcp_displacement_m"] > 0.0


def test_f1_receipt_is_truthful():
    s = json.loads((ROOT / "evidence/f1_attempt02/summary.json").read_text())
    assert s["status"] == "BLOCKED_RUNTIME_EXCEPTION"
    assert s["moveit_exit"] == 0
    assert s["qp_stop_reason"] == "LOCAL_SWEEP_LIMIT"
    assert s["timeout_reached"] is True


if __name__ == "__main__":
    test_f0_receipt()
    test_f1_receipt_is_truthful()
    print("receipt checks: PASS")
