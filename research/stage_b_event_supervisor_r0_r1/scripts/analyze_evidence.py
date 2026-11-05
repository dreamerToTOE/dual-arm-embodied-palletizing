#!/usr/bin/env python3
"""Summarize the preserved FR3 contact run and make a small audit plot."""
import gzip
import json
from pathlib import Path
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = ROOT / "research/stage_b_event_supervisor_r0_r1/evidence/r1_event_supervisor_physical_push01"
OUT = ROOT / "research/stage_b_event_supervisor_r0_r1"


def main():
    trace = EVIDENCE / "physical_trace/physical_trace.jsonl.gz"
    contacts = EVIDENCE / "physical_trace/contact_events.jsonl.gz"
    points = []
    first_tool = first_wall = None
    max_impulse = 0.0
    with gzip.open(trace, "rt", encoding="utf-8") as stream:
        for index, line in enumerate(stream):
            row = json.loads(line)
            cube = row.get("cube_world_m")
            if cube and (index % 20 == 0):
                points.append((float(row["time_s"]), float(cube[0]), float(cube[1])))
    with gzip.open(contacts, "rt", encoding="utf-8") as stream:
        for line in stream:
            row = json.loads(line)
            paths = row.get("colliders", [])
            impulse = row.get("impulse_world_Ns", [0, 0, 0])
            max_impulse = max(max_impulse, sum(float(v) ** 2 for v in impulse) ** 0.5)
            if first_tool is None and any("side_suction_tool" in p for p in paths):
                first_tool = {"physics_step": row["physics_step"], "time_s": row["time_s"]}
            if first_wall is None and any("WallDeep" in p for p in paths):
                first_wall = {"physics_step": row["physics_step"], "time_s": row["time_s"]}
    summary = json.loads((EVIDENCE / "physical_trace/physical_trace_summary.json").read_text())
    executor = (EVIDENCE / "task27_executor.log").read_text(errors="replace")
    final_match = re.search(r"cell_error=([0-9.+-]+) mm, ([^=]+)_gap=([0-9.+-]+) mm, ([^=]+)_gap=([0-9.+-]+) mm", executor)
    result = {
        "schema": "stage_b.event_supervisor.r1.evidence_analysis.v1",
        "trace_summary": summary,
        "first_tool_cube_contact": first_tool,
        "first_deep_wall_contact": first_wall,
        "max_contact_impulse_norm_Ns": max_impulse,
        "final_executor_geometry": final_match.group(0) if final_match else None,
    }
    out_json = OUT / "evidence_analysis.json"
    out_json.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    if points:
        t, x, y = zip(*points)
        fig, axis = plt.subplots(figsize=(7, 3.5))
        axis.plot(t, x, label="Cube x")
        axis.plot(t, y, label="Cube y")
        if first_tool:
            axis.axvline(first_tool["time_s"], color="tab:green", linestyle="--", label="tool/Cube")
        if first_wall:
            axis.axvline(first_wall["time_s"], color="tab:red", linestyle="--", label="deep wall")
        axis.set(xlabel="simulation time (s)", ylabel="position (m)")
        axis.grid(True, alpha=0.25)
        axis.legend(loc="best")
        fig.tight_layout()
        fig.savefig(OUT / "figures/r1_cube_motion.png", dpi=150)
        plt.close(fig)
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
