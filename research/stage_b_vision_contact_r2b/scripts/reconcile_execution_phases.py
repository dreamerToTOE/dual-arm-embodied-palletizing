#!/usr/bin/env python3
"""Reconcile executor receipts with camera samples; never use Cube GT for phase labels."""
from __future__ import annotations
import argparse, json, re
from pathlib import Path
import numpy as np

PHASES = [
    ('cube_supply','########## Task27 batch 1/1','selected LEFT_CANDIDATE_1/8','COMPLETED'),
    ('dual_arm_approach','selected LEFT_CANDIDATE_1/8','COMMON_LIFT synchronized FCL PASS','COMPLETED'),
    ('dual_arm_attachment','COMMON_LIFT synchronized FCL PASS','COMMON_LIFT GT=','COMPLETED'),
    ('coordinated_transport','COMMON_X_TRAVEL synchronized FCL PASS','COMMON_Y_ALIGN GT=','COMPLETED'),
    ('placement_release','COMMON_DROP_TO_TABLE synchronized FCL PASS','after COMMON_DROP_TO_TABLE','COMPLETED'),
    ('release_retreat','PREPLANNED_COMMON_RETREAT synchronized FCL PASS','双臂共同退出已使推入臂到达高位','COMPLETED'),
    ('single_arm_recontact','REGRASP_NEG_X batch=1 candidates=','after REGRASP:','COMPLETED'),
    ('single_arm_insertion','PUSH slice 1/16','PUSH complete:','COMPLETED'),
    ('deep_wall_arrival','PUSH complete:','+X support seat','COMPLETED'),
    ('auxiliary_side_compaction','EXECUTION SIDE_HIGH_APPROACH candidates=','SIDE_COMPACTION LIFT execution failed','PREFLIGHT_BLOCKED'),
]

def parse_log(path):
    out=[]
    for i,line in enumerate(path.read_text(errors='replace').splitlines(),1):
        m=re.search(r'\[(\d+\.\d+)\].*?: (.*)',line)
        if m: out.append({'line':i,'wall_time_s':float(m.group(1)),'text':m.group(2)})
    return out

def first_match(events, needle, after=0):
    for e in events:
        if e['line']>=after and needle in e['text']: return e
    return None

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('run',type=Path); ap.add_argument('--raw-run',type=Path); ap.add_argument('--out',type=Path,required=True); args=ap.parse_args(); root=args.run; raw=args.raw_run or root
    events=parse_log(root/'task27_executor.log'); trace=[]
    with open(raw/'physical_trace/physical_trace.jsonl') as f:
        for line in f: trace.append(json.loads(line))
    pairs=[(r['time_s'],r['controller_command']['time_wall_s']) for r in trace if r.get('controller_command') and r['controller_command'].get('time_wall_s')]
    sim_from_wall=np.polyfit(np.asarray([p[1] for p in pairs]),np.asarray([p[0] for p in pairs]),1) if len(pairs)>=2 else None
    def to_sim(wall):
        if sim_from_wall is None: return None
        return float(sim_from_wall[0]*wall+sim_from_wall[1])
    timeline=[json.loads(x) for x in (root/'r2b_camera/camera_timeline.jsonl').read_text().splitlines() if x.strip()]
    detections=[]
    dp=root/'visual_detections.jsonl'
    if dp.exists(): detections=[json.loads(x) for x in dp.read_text().splitlines() if x.strip()]
    receipts=[]; cursor=0
    for skill,start_text,end_text,status in PHASES:
        s=first_match(events,start_text,cursor)
        e=first_match(events,end_text,s['line'] if s else cursor)
        if s: cursor=s['line']
        if not s:
            receipts.append({'skill_id':skill,'execution_status':'NOT_EXECUTED','receipt_source':'executor_log','camera_samples':0,'camera_detection_rate':None}); continue
        ssim=to_sim(s['wall_time_s']); esim=to_sim(e['wall_time_s']) if e else ssim
        lo=min(ssim,esim) if ssim is not None and esim is not None else None; hi=max(ssim,esim) if lo is not None else None
        cams=[r for r in timeline if lo is not None and lo<=float(r['time_s'])<=hi]
        valid=[r for r in detections if r.get('status')=='PRESENT' and lo is not None and lo<=float(r['time_s'])<=hi]
        step_lo=min((int(r['physics_step']) for r in trace if lo is not None and lo<=r['time_s']<=hi),default=None); step_hi=max((int(r['physics_step']) for r in trace if lo is not None and lo<=r['time_s']<=hi),default=None)
        if skill == 'auxiliary_side_compaction' and e and 'LIFT execution failed' in e['text']:
            status = 'EXECUTION_FAILED'
        observed='EXECUTED_WITH_CAMERA_SAMPLES' if cams and status=='COMPLETED' else ('EXECUTED_WITHOUT_CAMERA_SAMPLES' if status=='COMPLETED' else status)
        rec={'skill_id':skill,'start_event':s['text'],'end_event':(e['text'] if e else None),'start_wall_time_s':s['wall_time_s'],'end_wall_time_s':(e['wall_time_s'] if e else None),'start_sim_time_s':ssim,'end_sim_time_s':esim,'physics_step_interval':[step_lo,step_hi],'execution_status':observed,'executor_receipt_status':status,'controller_owner':'Task26BatchedFeedBridge / task27 executor','robot_cube_state_receipt':'physical_trace q_actual,dq_actual,tcp_world_m,cube_world_m,suction_state; Cube field evaluation-only','camera_samples':len(cams),'camera_detections':len(valid),'camera_detection_rate':(len(valid)/len(cams) if cams else None),'phase_source':'executor log event receipts; no Cube-GT threshold labels'}
        if skill=='auxiliary_side_compaction':
            pair=first_match(events,'FCL pair:',s['line']); depth=first_match(events,'FCL contact:',s['line']); rec['failure_root_cause']={'fcl_pair':pair['text'] if pair else None,'contact':depth['text'] if depth else None,'safe_abort_message':'both side suctions are OPEN is downstream safe-abort handling'}
        receipts.append(rec)
    result={'schema':'task28.r2b.execution_phase_receipts.v1','run_id':root.name,'mapping_method':'least-squares wall timestamp to simulation time using recorded controller command receipt times; phase boundaries remain executor-log receipts','mapping_pairs':len(pairs),'receipts':receipts,'denominator_policy':'Each skill has its own executor interval. Camera frame counts are interval-local and may overlap only when explicitly marked; they are not summed as a global total.','status_separation':{'runner_finished':json.loads((root/'summary.json').read_text()).get('status'),'executor_exit':json.loads((root/'summary.json').read_text()).get('executor_exit'),'manipulation_result':'FAILED_OR_BLOCKED' if json.loads((root/'summary.json').read_text()).get('executor_exit') not in (0,None) else 'EXECUTOR_COMPLETED','camera_observation_result':'SEE_INTERVAL_COUNTS'}}
    args.out.parent.mkdir(parents=True,exist_ok=True); args.out.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n'); print(json.dumps(result,indent=2,ensure_ascii=False))
if __name__=='__main__': main()
