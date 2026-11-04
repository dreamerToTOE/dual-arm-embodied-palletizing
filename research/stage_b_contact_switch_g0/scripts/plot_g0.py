#!/usr/bin/env python3
"""Make compact, reproducible G0 diagnostic figures from retained JSONL evidence."""
from __future__ import annotations
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / 'evidence'
FIGURES = ROOT / 'figures'
FIGURES.mkdir(exist_ok=True)

def rows(run):
    return [json.loads(x) for x in (EVIDENCE / run / 'timeseries.jsonl').read_text().splitlines()]

def c1():
    rs = rows('g0_c1_table_04'); t=np.array([r['sim_time_s'] for r in rs]); f=np.array([r['matrix_force_world_N'][2] for r in rs]); v=np.array([r['velocity_world_m_s'][2] for r in rs]); e=np.array([r['unexplained_impulse_Ns'][2] for r in rs]); awake=np.array([not r['sleeping'] for r in rs])
    fig,ax=plt.subplots(3,1,figsize=(9,7),sharex=True,constrained_layout=True)
    ax[0].plot(t,f); ax[0].axhline(.8*9.81,color='k',ls='--',label='m|g|'); ax[0].set_ylabel('support Fz (N)'); ax[0].legend()
    ax[1].plot(t,v*1e3); ax[1].set_ylabel('Cube vz (mm/s)')
    ax[2].plot(t,e); ax[2].set_ylabel('residual (N s)'); ax[2].set_xlabel('simulation time (s)')
    for a in ax: a.grid(alpha=.25); a.axvline(t[np.flatnonzero(~awake)[0]],color='gray',ls=':',label='sleep transition' if a is ax[0] else None)
    fig.suptitle('G0-C1 minimal Cube/Table (run 04; gray line = first sleep)'); fig.savefig(FIGURES/'g0_c1_awake_sleep.png',dpi=160); plt.close(fig)

def c2():
    rs=rows('g0_c2_wall_02'); t=np.array([r['simulation_time_s'] for r in rs]); gap=np.array([r['wall_gap_m'] for r in rs]); f=np.array([r['cube_wall_force_world_N'][0] for r in rs]); phase=[r['phase'] for r in rs]
    fig,ax=plt.subplots(2,1,figsize=(9,6),sharex=True,constrained_layout=True)
    ax[0].plot(t,gap); ax[0].axhline(1e-4,color='k',ls='--'); ax[0].set_ylabel('wall gap (m)')
    ax[1].plot(t,f); ax[1].axhline(.02,color='k',ls='--'); ax[1].set_ylabel('wall normal F (N)'); ax[1].set_xlabel('simulation time (s)')
    for a in ax: a.grid(alpha=.25)
    fig.suptitle('G0-C2 Cube/DeepWall (run 02; withdrawal did not unload)'); fig.savefig(FIGURES/'g0_c2_wall_loaded.png',dpi=160); plt.close(fig)

def c3():
    rs=rows('g0_c3_tool_01'); t=np.array([r['simulation_time_s'] for r in rs]); f=np.array([r['tool_cube_force_world_N'][1] for r in rs]); y=np.array([r['cube_xyz_m'][1] for r in rs]);
    fig,ax=plt.subplots(2,1,figsize=(9,6),sharex=True,constrained_layout=True)
    ax[0].plot(t,f); ax[0].set_ylabel('tool/Cube Fy (N)')
    ax[1].plot(t,(y-y[0])*1e3); ax[1].set_ylabel('Cube Δy (mm)'); ax[1].set_xlabel('simulation time (s)')
    for a in ax: a.grid(alpha=.25)
    fig.suptitle('G0-C3 isolated rigid tool/Cube (run 01)'); fig.savefig(FIGURES/'g0_c3_tool_contact.png',dpi=160); plt.close(fig)

if __name__ == '__main__':
    c1(); c2(); c3(); print('wrote', sorted(str(p) for p in FIGURES.glob('*.png')))
