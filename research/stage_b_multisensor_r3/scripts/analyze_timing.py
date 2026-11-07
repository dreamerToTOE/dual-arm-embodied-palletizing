#!/usr/bin/env python3
"""Offline render-wait displacement diagnostic; never a correction to estimates."""
import argparse,json
from pathlib import Path
import numpy as np


def main():
 ap=argparse.ArgumentParser();ap.add_argument('--dataset',type=Path,action='append',required=True);ap.add_argument('--out',type=Path,required=True);args=ap.parse_args();output={}
 for ds in args.dataset:
  obs=[json.loads(l) for l in (ds/'observations.jsonl').read_text().splitlines()];labs=[json.loads(l) for l in (ds/'oracle.jsonl').read_text().splitlines()]
  displacement=[np.linalg.norm(np.array(r['cube_xyz_after_render_wait_m'])-r['cube_xyz_m']) for r in labs if r.get('cube_xyz_after_render_wait_m') is not None]
  lat=[r['writer_latency_wall_s'] for r in obs if r.get('writer_latency_wall_s') is not None];age=[r['receipt_to_render_wait_s'] for r in obs if r.get('receipt_to_render_wait_s') is not None]
  assert displacement and lat and age,'missing timing data'
  output[ds.name]={'receipt_to_render_wait_sim_s_p50_p95_max':np.percentile(age,[50,95,100]).tolist(),'writer_wall_latency_s_p50_p95_max':np.percentile(lat,[50,95,100]).tolist(),'offline_gt_displacement_during_render_wait_m_p50_p95_max':np.percentile(displacement,[50,95,100]).tolist(),'use_in_estimator':False,'interpretation':'image exposure is not assigned to either endpoint; this is an offline bound diagnostic, not a latency correction'}
 args.out.write_text(json.dumps(output,indent=2)+'\n')

if __name__=='__main__':main()
