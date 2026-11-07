#!/usr/bin/env python3
"""Frozen A/B evaluation: prediction reads observations, scoring reads oracle later."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from research.stage_b_multisensor_r3.planar_pose_estimator import angle_error_rad,estimate_v0,estimate_v1


def jsonl(path):return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def stats(values):
 if not values:return None
 a=np.array(values,float);return {'median':float(np.median(a)),'p95':float(np.percentile(a,95)),'max':float(np.max(a))}
def gap(xyz,yaw):return 1.16-float(xyz[0])-.06*(abs(math.cos(yaw))+abs(math.sin(yaw)))


def score(rows,labels,method,*,common_only=False):
 table=[];negative=[];detected=0;accepted=0
 for row in rows:
  pred=row[method];truth=labels[row['key']]
  detected+=row['detected'];accepted+=pred['status']=='PRESENT'
  if not truth['on_table']:
   negative.append(pred['status']=='PRESENT');continue
  if pred['status']!='PRESENT' or (common_only and not all(row[v]['status']=='PRESENT' for v in ('v0','v1'))):continue
  xy=np.array(pred['cube_xyz_m'][:2])-np.array(truth['cube_xyz_m'][:2]);yaw=abs(angle_error_rad(pred['yaw_rad'],truth['yaw_rad']))*180/math.pi
  table.append({'xy':float(np.linalg.norm(xy)),'x':float(xy[0]),'y':float(xy[1]),'yaw':yaw,'gap':abs(gap(pred['cube_xyz_m'],pred['yaw_rad'])-gap(truth['cube_xyz_m'],truth['yaw_rad'])),'reproj':pred['reprojection_error_px'],'uncertainty':pred.get('uncertainty_xy_m')})
 n_table=sum(t['on_table'] for t in labels.values());runtimes=[r[method]['runtime_ms'] for r in rows if r['detected']]
 return {'total_frames':len(rows),'detected_frames':detected,'detection_rate':detected/len(rows),'accepted_all':accepted,'table_oracle_frames':n_table,'accepted_table_scored':len(table),'table_rejection_or_missing_rate':1-len(table)/n_table if n_table else None,'all_rejection_or_missing_rate':1-accepted/len(rows),'invalid_support_oracle_frames':len(negative),'invalid_support_false_accepts':sum(negative),'xy_error_m':stats([t['xy'] for t in table]),'axis_signed_mean_m':None if not table else [float(np.mean([t[k] for t in table])) for k in ('x','y')],'axis_abs_p95_m':None if not table else [float(np.percentile([abs(t[k]) for t in table],95)) for k in ('x','y')],'yaw_error_deg':stats([t['yaw'] for t in table]),'deep_wall_gap_abs_error_m':stats([t['gap'] for t in table]),'reprojection_error_px':stats([t['reproj'] for t in table]),'runtime_ms_detected_only':stats(runtimes),'local_linearized_xy_sigma_m':stats([t['uncertainty'] for t in table if t['uncertainty'] is not None]),'uncertainty_scope':'local >=1px noise, excludes calibration/model/timing bias; no calibrated error-bound claim'}


def evaluate(dataset,config,out):
 manifest=json.loads((dataset/'manifest.json').read_text())
 assert manifest['observation_sha256']==sha(dataset/'observations.jsonl'),'observation hash mismatch'
 observations=jsonl(dataset/'observations.jsonl')
 assert manifest['config_sha256']==sha(config),'dataset/config drift'
 params=json.loads(config.read_text());K=np.array(manifest['K']);M=np.array(manifest['world_from_camera']);predictions=[]
 options={k:params[k] for k in ('marker_size_m','marker_offset_m')}
 for obs in observations:
  forbidden=set(params['forbidden_estimator_inputs']).intersection(obs)
  assert not forbidden,f'oracle leaked into observations: {forbidden}'
  corners=obs.get('corners_px');detected=obs['status']=='PRESENT' and corners is not None
  row={'key':obs['key'],'detected':detected,'receipt_time_s':obs['receipt_time_s']}
  for method in ('v0','v1'):
   if not detected:pred={'status':'UNKNOWN','runtime_ms':0.,'reason':'marker_not_detected'}
   else:
    try:
     if method=='v0':pred=estimate_v0(corners,K,M,**options).to_dict()
     else:pred=estimate_v1(corners,K,M,**options,**{k:params[k] for k in ('table_height_m','cube_size_m','support_tolerance_m','tilt_limit_deg','residual_limit_px')},support_context=obs.get('support_context')).to_dict()
    except (ValueError,RuntimeError) as exc:pred={'status':'UNKNOWN','runtime_ms':0.,'reason':str(exc)}
   row[method]=pred
  predictions.append(row)
 out.mkdir(parents=True,exist_ok=True)
 # Oracle is opened only after all deployable predictions have been written.
 (out/f'{manifest["name"]}_predictions.jsonl').write_text(''.join(json.dumps(r,allow_nan=False)+'\n' for r in predictions))
 assert manifest['oracle_sha256']==sha(dataset/'oracle.jsonl'),'oracle hash mismatch'
 labels={r['key']:r for r in jsonl(dataset/'oracle.jsonl')}
 assert set(labels)=={r['key'] for r in predictions}
 return {'dataset':manifest['name'],'data_manifest_sha256':sha(dataset/'manifest.json'),'config_sha256':sha(config),'v0':score(predictions,labels,'v0'),'v1':score(predictions,labels,'v1'),'matched_acceptance_v0':score(predictions,labels,'v0',common_only=True),'matched_acceptance_v1':score(predictions,labels,'v1',common_only=True),'rejection_reasons_v1':{reason:sum((r['v1'].get('reason') or 'NONE')==reason for r in predictions) for reason in sorted({r['v1'].get('reason') or 'NONE' for r in predictions})}},predictions,labels


def main():
 ap=argparse.ArgumentParser();ap.add_argument('--dataset',type=Path,action='append',required=True);ap.add_argument('--config',type=Path,default=ROOT/'research/stage_b_multisensor_r3/config/p1_frozen.json');ap.add_argument('--out',type=Path,required=True);args=ap.parse_args()
 results=[];failures=[]
 for dataset in args.dataset:
  result,rows,labels=evaluate(dataset,args.config,args.out.parent);results.append(result)
  scored=[(np.linalg.norm(np.array(r['v1']['cube_xyz_m'][:2])-np.array(labels[r['key']]['cube_xyz_m'][:2])),r) for r in rows if labels[r['key']]['on_table'] and r['v1']['status']=='PRESENT']
  for error,row in sorted(scored,key=lambda x:x[0],reverse=True)[:3]:failures.append({'key':row['key'],'v1_xy_error_m':float(error),'prediction':row['v1'],'oracle':labels[row['key']]})
  for row in rows:
   if not labels[row['key']]['on_table'] and row['v1']['status']=='PRESENT':failures.append({'key':row['key'],'failure':'off_table_or_tilt_false_accept','prediction':row['v1'],'oracle':labels[row['key']]})
 result={'schema':'r3.p1.frozen_ab_results.v2','config_sha256':sha(args.config),'runs':results,'representative_failures':failures,'training':'No fitted parameters. SDK K/config fixed before physical06 and fresh static scene holdouts; physical05/07 are development diagnostics only. No random adjacent-frame split.','exposure_limit':'Historical physical06/07 score at physics receipt; RGB exposure unknown inside render-wait interval. Static holdout has no motion/latency bias.'}
 args.out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'runs':[{k:r[k] for k in ('dataset','v0','v1')} for r in results]},indent=2))

if __name__=='__main__':main()
