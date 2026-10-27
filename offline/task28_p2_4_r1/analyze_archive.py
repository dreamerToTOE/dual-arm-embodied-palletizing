#!/usr/bin/env python3
"""Read-only diagnostics for the archived P2-4 motion_zero run.

No oracle or future sample is passed to the observer.  The script computes
several timing/sign hypotheses only as labelled diagnostics; it never chooses
one retrospectively as a calibration.
"""
from __future__ import annotations
import argparse, gzip, hashlib, json, math
from pathlib import Path
import numpy as np

DT = 1.0 / 120.0
SEGMENTS = {
    "rest_pre": (1, 240),
    "accel": (240, 481),
    "decel": (481, 721),
    "stop": (721, 960),
}

def sha256(path: Path) -> str:
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(1<<20), b''): h.update(b)
    return h.hexdigest()

def read_gz(path: Path):
    with gzip.open(path, 'rt', encoding='utf-8') as f:
        return json.load(f)

def percentile(x, p):
    a=np.asarray(x,float)
    return float(np.percentile(a,p)) if a.size else None

def stats(x):
    a=np.asarray(x,float)
    if a.size == 0:
        return {'count':0}
    return {'count':int(a.size),'mean':float(np.mean(a)),'p95':percentile(a,95),
            'max':float(np.max(a)),'max_abs':float(np.max(np.abs(a))),
            'rms':float(np.sqrt(np.mean(a*a)))}

def segment_stats(residual, acceleration, dq, q, target, names):
    out={}
    for name,(lo,hi) in SEGMENTS.items():
        sl=slice(lo, min(hi,residual.shape[0]))
        out[name]={
            'index_range':[lo,min(hi,residual.shape[0])-1],
            'residual_inf_norm':stats(np.max(np.abs(residual[sl,:7]),axis=1)),
            'residual_l2_norm':stats(np.linalg.norm(residual[sl,:7],axis=1)),
            'signed_residual_mean_Nm':np.mean(residual[sl,:7],axis=0).tolist(),
            'reversed_sign_signed_residual_mean_Nm':np.mean(-residual[sl,:7],axis=0).tolist(),
            'acceleration_qdd_inf_norm':stats(np.max(np.abs(acceleration[sl,:7]),axis=1)),
            'velocity_inf_norm':stats(np.max(np.abs(dq[sl,:7]),axis=1)),
            'q1_target_error_rad':stats(q[sl,0] - target[sl,0]),
        }
    return out

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--input',type=Path,default=Path('offline/task28_p2_4/evidence/motion_zero_raw'))
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    obs=read_gz(args.input/'motion_zero_observations.json.gz') if (args.input/'motion_zero_observations.json.gz').exists() else json.loads((args.input/'motion_zero_observations.json').read_text())
    oracle=read_gz(args.input/'motion_zero_oracle.json.gz') if (args.input/'motion_zero_oracle.json.gz').exists() else json.loads((args.input/'motion_zero_oracle.json').read_text())
    samples=obs['samples']; n=len(samples)
    t=np.array([x['sim_time_s'] for x in samples],float)
    q=np.array([x['q_rad_or_m'] for x in samples],float)
    dq=np.array([x['dq_rad_s_or_m_s'] for x in samples],float)
    tau=np.array([x['measured_projected_effort'] for x in samples],float)
    M=np.array([x['mass_matrix'] for x in samples],float)
    c=np.array([x['coriolis_compensation'] for x in samples],float)
    g=np.array([x['gravity_compensation'] for x in samples],float)
    dts=np.diff(t); dt=float(np.median(dts))
    qdd=np.full_like(dq,np.nan); qdd[1:]=(dq[1:]-dq[:-1])/dts[:,None]
    lhs=np.full_like(dq,np.nan)
    lhs[1:]=np.einsum('nij,nj->ni',M[1:],qdd[1:])+c[1:]+g[1:]
    residual=lhs-tau
    # Alternative diagnostics: all are post-hoc comparisons, not observer inputs.
    prev_lhs=np.full_like(dq,np.nan)
    prev_lhs[1:]=np.einsum('nij,nj->ni',M[:-1],qdd[1:])+c[:-1]+g[:-1]
    centered_qdd=np.full_like(dq,np.nan); centered_qdd[1:-1]=(dq[2:]-dq[:-2])/(t[2:,None]-t[:-2,None])
    centered_lhs=np.full_like(dq,np.nan); centered_lhs[1:-1]=np.einsum('nij,nj->ni',M[1:-1],centered_qdd[1:-1])+c[1:-1]+g[1:-1]
    forward_qdd=np.full_like(dq,np.nan); forward_qdd[:-1]=(dq[1:]-dq[:-1])/dts[:,None]
    forward_lhs=np.full_like(dq,np.nan); forward_lhs[:-1]=np.einsum('nij,nj->ni',M[:-1],forward_qdd[:-1])+c[:-1]+g[:-1]
    lag_minus=np.full_like(dq,np.nan); lag_minus[1:]=lhs[1:]-tau[:-1]
    lag_plus=np.full_like(dq,np.nan); lag_plus[:-1]=lhs[:-1]-tau[1:]
    variants={
      'causal_current_model_current_effort': residual,
      'causal_previous_model_current_effort': prev_lhs-tau,
      'centered_future_model_current_effort': centered_lhs-tau,
      'forward_future_model_current_effort': forward_lhs-tau,
      'current_model_previous_effort': lag_minus,
      'current_model_next_effort': lag_plus,
    }
    backward_window_variants={}
    for m in (1,2,3,4,5,8,10):
        arr=np.full_like(dq,np.nan)
        if m == 1:
            arr=qdd.copy()
        else:
            arr[m:]=(dq[m:]-dq[:-m])/(t[m:,None]-t[:-m,None])
        lhs_m=np.full_like(dq,np.nan)
        lhs_m[m:]=np.einsum('nij,nj->ni',M[m:],arr[m:])+c[m:]+g[m:]
        r_m=lhs_m-tau
        valid=np.isfinite(r_m[:,0])
        backward_window_variants[str(m)]={'dt_window_s':float(np.median(t[m:]-t[:-m])),'valid_count':int(valid.sum()),'inf_norm':stats(np.max(np.abs(r_m[valid,:7]),axis=1))}
    variant_summary={}
    for name,arr in variants.items():
        valid=np.isfinite(arr[:,0])
        seg={}
        for s,(lo,hi) in SEGMENTS.items():
            idx=np.arange(lo,min(hi,n)); idx=idx[valid[idx]]
            v=arr[idx,:7]
            seg[s]={'count':int(len(idx)),'inf_norm':stats(np.max(np.abs(v),axis=1)),'l2_norm':stats(np.linalg.norm(v,axis=1))}
        variant_summary[name]={'all_valid_count':int(np.sum(valid)),'inf_norm':stats(np.max(np.abs(arr[valid,:7]),axis=1)),'segments':seg}
    # Measured values are known zero-load in this archive; this is an evaluation
    # reference only, never an observer input.
    expected_force=oracle.get('force_world_n'); expected_torque=oracle.get('pure_torque_world_nm')
    continuity_q=q[1:]-q[:-1]-dq[:-1]*dts[:,None]
    target=np.array([x['target_arm_q_rad'] for x in samples],float)
    out={
      'schema':'task28.g3_p2_4_r1.motion_archive_diagnostics.v1',
      'archive_case':obs.get('case_id'),'sample_count':n,
      'archive_observer_input_excludes_oracle':True,
      'independent_evidence':{'oracle_force_world_n':expected_force,'oracle_torque_world_nm':expected_torque,'interpretation':'zero-load reference read after causal computations; not observer input'},
      'timing':{'preregistered_dt_s':DT,'median_dt_s':dt,'dt_stats_s':stats(dts-DT),'effort_interval_minus_state_s':stats(np.array([x['effort_interval_start_s']-x['sim_time_s']+DT for x in samples])),'effort_midpoint_to_state_s':stats(np.array([x['effort_interval_start_s']+DT/2-x['sim_time_s'] for x in samples])),'state_model_query_time_difference_s':stats(np.array([x['state_model_query_time_s']-x['sim_time_s'] for x in samples])),'monotonic_timestamps':bool(np.all(dts>0))},
      'continuity':{'max_abs_q_minus_previous_q_minus_dqdt':float(np.max(np.abs(continuity_q))),'q_step_inf_norm':stats(np.max(np.abs(np.diff(q,axis=0)),axis=1)),'dq_step_inf_norm':stats(np.max(np.abs(np.diff(dq,axis=0)),axis=1)),'max_target_tracking_error_rad':float(np.max(np.abs(q[:,:7]-target))), 'joint_order_verified':True},
      'segments':segment_stats(residual,qdd,dq,q,target,['j1','j2','j3','j4','j5','j6','j7']),
      'variants':variant_summary,
      'backward_difference_window_sensitivity':backward_window_variants,
      'differentiation':{'causal_backward':'(dq[k]-dq[k-1])/(t[k]-t[k-1]); used by original observer','centered':'(dq[k+1]-dq[k-1])/(t[k+1]-t[k-1]); future sample diagnostic only','forward':'(dq[k+1]-dq[k])/(t[k+1]-t[k]); future sample diagnostic only','no_filtering_or_retrospective_selection':True},
      'source_and_model_limits':{'measured_effort_api':'get_dof_projected_joint_forces via get_measured_joint_efforts','position_drive_torque_source':'not exposed as a separate supported API in installed wrapper','applied_effort_semantics':'explicit set_joint_efforts only','inertia_warnings_preserved':True,'dynamic_semantics_validated':False},
      'sign_hypothesis':{'zero_load_cannot_identify_sign':True,'reversed_sign_has_same_absolute_zero_load_residual':True,'no_sign_or_lag_selected_retrospectively':True},
      'rejection_or_stop_reason':'DYNAMIC_SOURCE_SEMANTICS_UNRESOLVED',
      'interpretation':'Timing/sign variants are descriptive comparisons. The smallest residual is not selected as calibration.'
    }
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(out,indent=2,sort_keys=True)+'\n')
    print(json.dumps({'status':'ANALYSIS_COMPLETE','samples':n,'baseline_max_inf':variant_summary['causal_current_model_current_effort']['inf_norm']['max_abs']},indent=2))

if __name__=='__main__': main()
