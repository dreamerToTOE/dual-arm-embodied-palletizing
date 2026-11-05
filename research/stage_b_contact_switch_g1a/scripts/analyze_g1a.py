#!/usr/bin/env python3
"""Aggregate fixed-protocol G0-R/G1-A evidence and make compact plots."""
from __future__ import annotations
import json, hashlib
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
ROOT=Path(__file__).resolve().parents[1]; E=ROOT/'evidence'; F=ROOT/'figures'; F.mkdir(exist_ok=True)

def read_summary(run): return json.loads((E/run/'summary.json').read_text())
def read_rows(run): return [json.loads(x) for x in (E/run/'timeseries.jsonl').read_text().splitlines()]

def g0r():
    runs=[f'g0r_c2_final_{i:02d}' for i in (1,2,3)]
    ss=[read_summary(r) for r in runs]
    assert all(s['status']=='RECORDED' and s['loaded_contact_acceptance'] and s['unloaded_contact_acceptance'] for s in ss)
    assert all(s['event_step_alignment_ok'] for s in ss)
    result={'protocol_id':ss[0]['protocol_id'],'protocol_sha256':ss[0]['protocol_sha256'],'runs':runs,'repeat_count':len(runs),'loaded_repeat_count':sum(s['loaded_contact_acceptance'] for s in ss),'unloaded_repeat_count':sum(s['unloaded_contact_acceptance'] for s in ss),'corrected_residual_linf_Ns':[s['corrected_residual_linf_Ns'] for s in ss],'uncorrected_residual_linf_Ns':[s['uncorrected_residual_linf_Ns'] for s in ss],'peak_wall_normal_force_N':[s['peak_wall_normal_force_N'] for s in ss],'first_loaded_step':[s['first_loaded_step'] for s in ss],'first_separated_step':[s['first_separated_step'] for s in ss],'raw_wall_pair_steps_after_separation':[s['raw_wall_pair_steps_after_separation'] for s in ss],'post_separation_loaded_impulse_events':[s['contact_events_after_separation'] for s in ss]}
    (ROOT/'g0r_summary.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    rs=read_rows(runs[0]); t=np.array([r['simulation_time_s'] for r in rs]); gap=np.array([r['wall_gap_m'] for r in rs]); wf=np.array([r['cube_wall_force_world_N'][0] for r in rs]); corr=np.array([max(abs(x) for x in r['unexplained_impulse_Ns']) for r in rs]); unc=np.array([max(abs(x) for x in r['uncorrected_unexplained_impulse_Ns']) for r in rs]);
    fig,ax=plt.subplots(3,1,figsize=(9,7),sharex=True,constrained_layout=True); ax[0].plot(t,gap); ax[0].axhline(1e-4,ls='--',color='k'); ax[0].set_ylabel('wall gap (m)'); ax[1].plot(t,wf,label='signed wall F'); ax[1].set_ylabel('wall Fx (N)'); ax[1].legend(); ax[2].semilogy(t,np.maximum(corr,1e-12),label='corrected'); ax[2].semilogy(t,np.maximum(unc,1e-12),label='uncorrected'); ax[2].set_ylabel('ledger residual (N s)'); ax[2].set_xlabel('simulation time (s)'); ax[2].legend();
    for a in ax: a.grid(alpha=.25)
    fig.suptitle('G0-R DeepWall: active-command ledger and unload'); fig.savefig(F/'g0r_wall_ledger.png',dpi=160); plt.close(fig)
    return result

def g1a():
    runs={c:[f'g1a_{c}_final_{i:02d}' for i in (3,4,5)] for c in ('C0','C1','C2')}
    result={'protocol_sha256':read_summary(runs['C0'][0])['protocol_sha256'],'controllers':{}}
    for c,rr in runs.items():
        ss=[read_summary(r) for r in rr]; assert all(s['status']=='RECORDED' for s in ss)
        result['controllers'][c]={'runs':rr,'repeat_count':len(ss),'transition_times_s':[s['transition_time_s'] for s in ss],'final_gaps_m':[s['final_cube_wall_gap_m'] for s in ss],'geometric_acceptance':[s['geometric_acceptance'] for s in ss],'loaded_wall_contact':[s['loaded_wall_contact'] for s in ss],'wall_contact_steps':[s['wall_contact_steps'] for s in ss],'max_wall_force_N':[s['max_wall_force_N'] for s in ss],'transition_command_discontinuity_N':[s['transition_command_discontinuity_max_N'] for s in ss],'corrected_residual_linf_Ns':[s['max_corrected_residual_linf_Ns'] for s in ss],'mode_sequences':[s['mode_sequence'] for s in ss]}
    (ROOT/'g1a_comparison.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    fig,ax=plt.subplots(3,1,figsize=(10,8),sharex=False,constrained_layout=True)
    colors={'C0':'tab:blue','C1':'tab:orange','C2':'tab:green'}
    for c in ('C0','C1','C2'):
        rs=read_rows(runs[c][0]); t=np.array([r['simulation_time_s'] for r in rs]); gap=np.array([r['cube_wall_gap_m'] for r in rs]); cmd=np.array([r['commanded_force_next_world_N'][0] for r in rs]); wf=np.array([r['wall_force_world_N'][0] for r in rs]); ax[0].plot(t,gap,label=c,color=colors[c]); ax[1].plot(t,cmd,label=c,color=colors[c]); ax[2].plot(t,wf,label=c,color=colors[c])
    ax[0].axhline(.003,color='k',ls='--',label='3 mm'); ax[0].set_ylabel('Cube-wall gap (m)'); ax[1].set_ylabel('next command Fx (N)'); ax[2].set_ylabel('wall Fx (N)'); ax[2].set_xlabel('simulation time (s)')
    for a in ax: a.grid(alpha=.25); a.legend()
    fig.suptitle('G1-A geometry/motion-only physical proxy comparison (representative repeat)'); fig.savefig(F/'g1a_proxy_comparison.png',dpi=160); plt.close(fig)
    return result

if __name__=='__main__':
    print(json.dumps({'g0r':g0r(),'g1a':g1a()},indent=2,ensure_ascii=False))
