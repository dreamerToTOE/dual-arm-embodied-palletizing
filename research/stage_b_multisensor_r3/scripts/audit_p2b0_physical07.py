#!/usr/bin/env python3
"""Separate legacy-dropout audit. Sanitized extract replays without local raw/GT files."""
import argparse, hashlib, json, sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
R3=ROOT/'research/stage_b_multisensor_r3'
from research.stage_b_multisensor_r3.scripts.replay_p2b0_history import rows, dump, save_rows, stats, sha


def extract(raw, out):
    out.mkdir(parents=True,exist_ok=False)
    prior=json.loads((R3/'evidence/p2a_occlusion/summary.json').read_text())
    gap=max(prior['stage_b']['absence_intervals'],key=lambda r:r['observed_absence_span_s'])
    lo,hi=gap['first_missing_step'],gap['last_missing_step']
    trace=raw/'physical_trace/physical_trace.jsonl';timeline=raw/'r2b_camera/camera_timeline.jsonl'
    selected=[]
    for r in rows(trace):
        if lo<=r['physics_step']<=hi:
            selected.append(dict(run_id='physical07_legacy_ONLY', epoch_id=0, time_base_id='legacy_PhysicalTrace_dt_since_subscription',
                **{k:r[k] for k in ('physics_step','time_s','dt_s','q_actual_rad','dq_actual_rad_s','suction_state','suction_command','controller_command')}))
    images=[{k:r[k] for k in ('physics_step','time_s','physics_state_time_s','frame_pair_ok','rgb','depth','scheduler_skipped_intervals')} for r in rows(timeline) if lo<=r['physics_step']<=hi]
    detections={r['physics_step']:r for r in rows(raw/'visual_detections.jsonl')}
    for r in images:
        d=detections[r['physics_step']]
        r['visual_status']=d.get('status',d.get('visual',{}).get('status'))
    save_rows(out/'robot_observables.jsonl',selected);save_rows(out/'visual_dropout.jsonl',images)
    contact=raw/'physical_trace/contact_events.jsonl'
    dump(out/'provenance.json',{'schema':'r3.p2b0.physical07_extract.v1','run_id':'physical07_legacy_ONLY',
        'raw_root':str(raw),'raw_files_sha256':{str(p.relative_to(raw)):sha(p) for p in [trace,timeline,raw/'visual_detections.jsonl',contact]},
        'prior_gap':gap,'source_code_reference':'isaac/scripts/task28_task27_one_pass.py PhysicalTrace; retained historical producer revision incomplete',
        'sensor_meaning':{'suction_state':'Surface_Gripper.is_closed Boolean software attachment state; not pressure or independent seal/force measurement',
            'suction_command':'last requested close/open; not measured state','contact_events':'PhysX pair/contact/impulse oracle ONLY; excluded from extract',
            'force':'No independent tool/wall force sensor; impulse/dt would be an oracle-derived average, not measured force'},
        'clock_caveat':'Legacy receipt times/physics steps preserved; exact exposure association remains unverified. Never combine with r3_p2b0_sync_holdout01.',
        'files_sha256':{n:sha(out/n) for n in ['robot_observables.jsonl','visual_dropout.jsonl']}})


def audit(data,out):
    m=json.loads((data/'provenance.json').read_text())
    for n,h in m['files_sha256'].items():assert sha(data/n)==h
    robot=rows(data/'robot_observables.jsonl');vision=rows(data/'visual_dropout.jsonl')
    def finite(r,k):return set(r[k])=={'left','right'} and all(np.array(r[k][s]).shape==(7,) and np.isfinite(r[k][s]).all() for s in ('left','right'))
    result={'run_id':m['run_id'],'robot_rows':len(robot),'step_start':robot[0]['physics_step'],'step_end':robot[-1]['physics_step'],
        'missing_steps':sum(max(0,b['physics_step']-a['physics_step']-1) for a,b in zip(robot,robot[1:])),
        'dt_s':stats(np.diff([r['time_s'] for r in robot])),
        'finite_q_rows':sum(finite(r,'q_actual_rad') for r in robot),'finite_dq_rows':sum(finite(r,'dq_actual_rad_s') for r in robot),
        'suction_boolean_rows':{s:sum(isinstance(r['suction_state'][s],bool) for r in robot) for s in ('left','right')},
        'suction_command_present_rows':sum(r['suction_command'] is not None for r in robot),
        'distinct_controller_sequences':len({r['controller_command']['sequence'] for r in robot if r['controller_command']}),
        'missing_visual_detections':sum(r['visual_status']=='UNKNOWN' for r in vision), 'acquired_images':len(vision),
        'paired_images':sum(r['frame_pair_ok'] for r in vision),'observed_absence_span_s':vision[-1]['time_s']-vision[0]['time_s'],
        'right_censored':m['prior_gap']['right_censored'],'precise_exposure_synchronization_validated':False,
        'cube_pose_from_fk':None,'attachment_verified':False,'combined_with_new_run':False}
    dump(out,result);return result


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--data',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);ap.add_argument('--extract-raw-root',type=Path);a=ap.parse_args()
    if a.extract_raw_root:extract(a.extract_raw_root,a.data)
    print(json.dumps(audit(a.data,a.out),indent=2))
