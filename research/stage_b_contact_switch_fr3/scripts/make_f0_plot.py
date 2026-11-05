#!/usr/bin/env python3
"""Make a compact static receipt plot from the recorded F0 JSONL."""
import json
from pathlib import Path

import matplotlib.pyplot as plt

ROOT = Path(__file__).parents[1]


def main():
    rows = [json.loads(line) for line in (ROOT / "evidence/f0_smoke/timeseries.jsonl").read_text().splitlines()]
    t = [r["time_s"] for r in rows]
    q = [r["q_rad"][0] for r in rows]
    target = [r["target_q_rad"][0] for r in rows]
    tcp_x = [r["tcp_world_xyz_quat_wxyz"][0] for r in rows]
    fig, axes = plt.subplots(2, 1, figsize=(7.2, 4.8), sharex=True)
    axes[0].plot(t, q, label="measured J1")
    axes[0].plot(t, target, "--", label="target J1")
    axes[0].axvline(rows[90]["time_s"], color="k", alpha=0.35, label="hold transition")
    axes[0].set_ylabel("J1 [rad]")
    axes[0].legend(loc="best", fontsize=8)
    axes[0].grid(alpha=0.25)
    axes[1].plot(t, tcp_x, color="tab:orange")
    axes[1].axvline(rows[90]["time_s"], color="k", alpha=0.35)
    axes[1].set_xlabel("simulation time [s]")
    axes[1].set_ylabel("TCP x [m]")
    axes[1].grid(alpha=0.25)
    fig.suptitle("G1-A-FR3 F0: actual articulation tracking and target hold")
    fig.tight_layout()
    out = ROOT / "figures/f0_tracking.png"
    fig.savefig(out, dpi=160)
    print(out)


if __name__ == "__main__":
    main()
