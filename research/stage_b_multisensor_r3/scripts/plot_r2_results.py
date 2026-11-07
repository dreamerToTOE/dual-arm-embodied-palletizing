#!/usr/bin/env python3
"""Plot committed holdout uncertainty, including historical pose aliases."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

R3=Path(__file__).resolve().parents[1];out=R3/'evidence/r2_holdout01_analysis'
summary=json.loads((out/'summary.json').read_text())
frames=[json.loads(s) for s in (out/'time_binding_interface.jsonl').read_text().splitlines()]
poses=[json.loads(s) for s in (out/'pixel_predictions.jsonl').read_text().splitlines()]
assert len(frames)==summary['unique_dynamic_frames']
fig,axes=plt.subplots(1,3,figsize=(12,3.7),constrained_layout=True)
start=frames[0]['metadata_hypotheses'][0]['time_s'];x=[];width=[]
for r in frames:
    if r['candidates']:
        x.append(r['metadata_hypotheses'][0]['time_s']-start)
        ts=[c['time_s'] for c in r['candidates']];width.append((max(ts)-min(ts))*1000)
axes[0].step(x,width,where='post',label='Complete causal aliases',color='#1765a2')
axes[0].axhline(1000/60,color='#b13428',ls='--',label='One physics tick')
axes[0].set_yscale('log');axes[0].set_ylabel('Candidate time span (ms)');axes[0].legend(fontsize=8)
valid=[r for r in poses if r['clock_valid']]
axes[1].step([r['sdk_time_s']-start for r in valid],[r['velocity_m_s'] for r in valid],where='post',color='#9b4c15')
axes[1].set_ylabel('Command velocity (m/s)')
for axis in axes[:2]:axis.set_xlabel('Simulation time relative to first dynamic frame (s)')
labels=['V0 SDK','V1 R1 reference','V2 full aliases'];values=[summary['comparisons'][v]['xy_error_max_mm']['p95'] for v in ('V0','V1','V2')]
axes[2].bar(labels,values,color=['#777777','#b77a1c','#1765a2']);axes[2].set_yscale('log');axes[2].set_ylabel('P95 candidate-worst XY error (mm)')
for axis in axes:axis.grid(alpha=.2)
fig.suptitle(f"R2 holdout: {summary['cube_timing_observable_frames']} independently time-observable / {summary['cube_timing_unresolved_frames']} unresolved; no midpoint timestamp")
fig.savefig(R3/'evidence/r2_scene_binding_comparison.png',dpi=150)
