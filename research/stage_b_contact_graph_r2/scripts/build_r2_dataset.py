#!/usr/bin/env python3
"""Build a compact, time-aligned R2 replay dataset from the R1 trace.

PhysX contacts are copied into ``oracle_reference`` only.  Estimator inputs
contain geometry/motion and explicit UNKNOWN fields; no contact oracle is read.
"""
from __future__ import annotations
import argparse, gzip, hashlib, json, math
from pathlib import Path
from collections import defaultdict

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TRACE = ROOT / "evidence/r1_event_supervisor_physical_push01/physical_trace"


def gz_lines(path):
    with gzip.open(path, "rt", encoding="utf-8") as f:
        for line in f:
            if line.strip():
                yield json.loads(line)


def pair_category(colliders):
    a, b = colliders
    if "/World/Table" in colliders:
        return "CUBE_TABLE"
    if "/World/Task27/TruckBox/WallDeep" in colliders:
        return "CUBE_DEEP_WALL"
    if any("left_fr3" in x and "side_suction_tool" in x for x in colliders):
        return "LEFT_TOOL_CUBE"
    if any("right_fr3" in x and "side_suction_tool" in x for x in colliders):
        return "RIGHT_TOOL_CUBE"
    return None


def sha256(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trace-dir", type=Path, default=DEFAULT_TRACE)
    ap.add_argument("--out", type=Path, default=ROOT / "dataset/r2_timeline.jsonl.gz")
    args = ap.parse_args()
    trace_path = args.trace_dir / "physical_trace.jsonl.gz"
    contact_path = args.trace_dir / "contact_events.jsonl.gz"
    contacts = defaultdict(lambda: {"edges": set(), "count": 0, "impulse_norm_sum_Ns": 0.0, "impulse_norm_max_Ns": 0.0})
    for event in gz_lines(contact_path):
        cat = pair_category(event.get("colliders", []))
        if not cat:
            continue
        step = int(event["physics_step"])
        imp = event.get("impulse_world_Ns") or [0.0, 0.0, 0.0]
        norm = math.sqrt(sum(float(v) ** 2 for v in imp))
        c = contacts[step]
        c["edges"].add(cat)
        c["count"] += 1
        c["impulse_norm_sum_Ns"] += norm
        c["impulse_norm_max_Ns"] = max(c["impulse_norm_max_Ns"], norm)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    previous_tcp = None
    previous_time = None
    n = 0
    with gzip.open(args.out, "wt", encoding="utf-8") as out, gzip.open(trace_path, "rt", encoding="utf-8") as src:
        for line in src:
            if not line.strip():
                continue
            raw = json.loads(line)
            step = int(raw["physics_step"])
            t = float(raw["time_s"])
            cube = tuple(float(v) for v in raw["cube_world_m"])
            left = tuple(float(v) for v in raw["tcp_world_m"]["left"])
            right = tuple(float(v) for v in raw["tcp_world_m"]["right"])
            ds = [math.dist(cube, left), math.dist(cube, right)]
            tool_speed = None
            if previous_tcp is not None and previous_time is not None and t > previous_time:
                tool_speed = max(math.dist(left, previous_tcp[0]), math.dist(right, previous_tcp[1])) / (t - previous_time)
            c = contacts.get(step, {"edges": set(), "count": 0, "impulse_norm_sum_Ns": 0.0, "impulse_norm_max_Ns": 0.0})
            edge_oracle = {name: ("OBSERVED_ACTIVE" if name in c["edges"] else "OBSERVED_INACTIVE") for name in ("LEFT_TOOL_CUBE", "RIGHT_TOOL_CUBE", "CUBE_TABLE", "CUBE_DEEP_WALL")}
            edge_oracle["CUBE_SIDE_WALL"] = "UNKNOWN"
            estimator_input = {
                "observation_state": "OBSERVED_ACTIVE",
                "edge_observations": {"CUBE_SIDE_WALL": "UNKNOWN"},
                "suction_command": None,
                "suction_closed_feedback": None,
                "command_advance_m": None,
                "features": {
                    "tool_cube_distance_m": min(ds),
                    "left_tool_cube_distance_m": ds[0],
                    "right_tool_cube_distance_m": ds[1],
                    # Signed nominal geometric feature; this is not a contact gap.
                    "deep_wall_gap_m": 1.10 - cube[0],
                    "cube_bottom_gap_m": cube[2] - 0.26,
                    "tool_speed_m_s": tool_speed,
                    "cube_speed_m_s": math.sqrt(sum(float(v) ** 2 for v in (raw.get("cube_velocity_m_s") or [0.0] * 3))),
                    "tracking_error_m": None,
                },
            }
            row = {
                "schema": "task28.r2.timeline.v1",
                "physics_step": step, "time_s": t, "dt_s": raw.get("dt_s"),
                "q_actual_rad": raw.get("q_actual_rad"), "dq_actual_rad_s": raw.get("dq_actual_rad_s"),
                "tcp_world_m": raw.get("tcp_world_m"), "cube_world_m": list(cube),
                "cube_quaternion_wxyz": raw.get("cube_quaternion_wxyz"), "cube_velocity_m_s": raw.get("cube_velocity_m_s"),
                "control_owner": raw.get("control_owner"), "control_mode": raw.get("control_mode"),
                "estimator_input": estimator_input,
                "oracle_reference": {"edge_states": edge_oracle, "contact_event_count": c["count"],
                                     "impulse_norm_sum_Ns": c["impulse_norm_sum_Ns"], "impulse_norm_max_Ns": c["impulse_norm_max_Ns"]},
                "availability": {"side_wall": "UNKNOWN", "suction": "UNKNOWN", "commanded_advancement": "UNKNOWN", "tracking_error": "UNKNOWN"},
            }
            out.write(json.dumps(row, separators=(",", ":")) + "\n")
            previous_tcp, previous_time = (left, right), t
            n += 1
    summary = {"schema": "task28.r2.timeline.summary.v1", "frames": n, "episode_count": 1,
               "trace_sha256": sha256(trace_path), "contact_sha256": sha256(contact_path),
               "input_policy": "geometry_motion_only; PhysX contacts retained as oracle_reference",
               "side_wall_observation": "UNKNOWN", "suction_observation": "UNKNOWN",
               "commanded_advancement": "UNKNOWN", "new_physical_episodes": 0}
    (args.out.parent / "r2_timeline_summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))

if __name__ == "__main__":
    main()
