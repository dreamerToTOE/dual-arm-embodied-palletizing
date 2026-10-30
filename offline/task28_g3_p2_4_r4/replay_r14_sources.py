#!/usr/bin/env python3
"""Replay R14 no-contact source rows through the strict adapter.

This is a post-collection, offline provenance check.  It does not rerun
Isaac, contact, suction, ROS, or a command sink.  Command, applied readback,
and projected readback are adapted through separate descriptors so their
semantics remain visible in the resulting evidence.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from .source_adapter import AdapterContract, AdapterRole, SourceAdapter, SourceKind, SourceSample, canonical_descriptor

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
DEFAULT_INPUT = ROOT / "offline/task28_r14/evidence/no_contact/attempt_01/raw/zero_load_hold_observations.json"


def sha(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def replay(input_path: Path) -> dict[str, object]:
    data = json.loads(input_path.read_text())
    rows = data.get("samples")
    if not isinstance(rows, list) or not rows:
        raise ValueError("MISSING_SAMPLES")
    first = rows[0]
    n = len(first["commanded_effort"])
    names = tuple(f"fr3_joint{i}" for i in range(1, 8)) + ("fr3_finger_joint1", "fr3_finger_joint2")
    if n != len(names):
        raise ValueError("UNEXPECTED_DOF_COUNT")
    dt = 1.0 / 120.0
    contract = AdapterContract(names, nominal_dt_s=dt, dt_tolerance_s=1e-6,
                               max_age_s=0.002, max_abs_effort_Nm=60., max_step_Nm=10.)
    evidence = sha(input_path)
    descriptors = {
        "simulation_drive_effort": canonical_descriptor(SourceKind.SIMULATION_DRIVE_EFFORT, names,
                                                          evidence_sha256=evidence, sample_period_s=dt),
        "simulation_applied_effort": canonical_descriptor(SourceKind.SIMULATION_APPLIED_EFFORT, names,
                                                           evidence_sha256=evidence, sample_period_s=dt),
        "simulation_projected_effort": canonical_descriptor(SourceKind.SIMULATION_PROJECTED_EFFORT, names,
                                                             evidence_sha256=evidence, sample_period_s=dt),
    }
    fields = {
        "simulation_drive_effort": "commanded_effort",
        "simulation_applied_effort": "applied_effort_readback",
        "simulation_projected_effort": "measured_projected_effort",
    }
    adapters = {name: SourceAdapter(contract) for name in descriptors}
    accepted = {name: 0 for name in descriptors}
    max_latency = 0.0
    timestamps = []
    for row in rows:
        t = float(row["sim_time_s"])
        seq = int(row["physics_step_index"])
        timestamps.append(t)
        max_latency = max(max_latency, float(row.get("source_read_duration_s", 0.0)))
        for name, adapter in adapters.items():
            adapter.accept(descriptors[name], SourceSample(row[fields[name]], t, seq),
                           role=AdapterRole.DYNAMICS_EFFORT_INPUT)
            accepted[name] += 1
    intervals = [b - a for a, b in zip(timestamps, timestamps[1:])]
    summary = {
        "schema": "task28.g3_p2_4.r4.r14_source_replay.v1",
        "status": "R14_SOURCES_STRICTLY_ADAPTED",
        "input": str(input_path),
        "input_sha256": evidence,
        "sample_count": len(rows),
        "dof_count": n,
        "joint_order": list(names),
        "simulation_physics_dt_s": dt,
        "timestamp_first_s": timestamps[0],
        "timestamp_last_s": timestamps[-1],
        "timestamp_min_dt_s": min(intervals),
        "timestamp_max_dt_s": max(intervals),
        "max_source_read_duration_s": max_latency,
        "accepted_counts": accepted,
        "source_validation": "SIMULATION_CANDIDATE",
        "simulation_tau_J_equivalence": False,
        "candidate_external_torque_validated": False,
        "contact_oracle_present": False,
        "hardware_or_ros": False,
        "interpretation": "Offline source provenance/quality replay only; R14 command/applied/projected simulation signals are not real FR3 tau_J and do not validate contact force.",
    }
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = replay(args.input)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps({k: result[k] for k in ("status", "sample_count", "accepted_counts")}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
