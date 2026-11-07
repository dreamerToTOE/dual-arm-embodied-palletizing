#!/usr/bin/env python3
"""Pixel-reviewed historical failures, acquisition gaps and observable coverage."""
import argparse,csv,hashlib,json,shutil,sys
from collections import Counter
from pathlib import Path
import cv2,numpy as np
from PIL import Image,ImageDraw
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from research.stage_b_vision_contact_r2b.visual.aruco_pose import detect_aruco


def rows(p):return [json.loads(l) for l in p.read_text().splitlines() if l.strip()]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def absence_intervals(frames):
    output=[];i=0
    while i<len(frames):
        if frames[i]['status']=='PRESENT':i+=1;continue
        start=i
        while i<len(frames) and frames[i]['status']!='PRESENT':i+=1
        last=frames[i-1];first=frames[start];previous=frames[start-1] if start else None;following=frames[i] if i<len(frames) else None
        output.append({'first_missing_step':first['physics_step'],'last_missing_step':last['physics_step'],
            'first_missing_s':first['time_s'],'last_missing_s':last['time_s'],'missing_acquired_images':i-start,
            'observed_absence_span_s':last['time_s']-first['time_s'],
            'last_valid_to_next_valid_or_last_observed_s':(following or last)['time_s']-(previous or first)['time_s'],
            'right_censored':following is None,'left_censored':previous is None})
    return output


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--raw',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);args=ap.parse_args()
    r3=ROOT/'research/stage_b_multisensor_r3';compact=ROOT/'research/stage_b_vision_contact_r2b/evidence/r2b_rear_wall_physical07';args.out.mkdir(parents=True,exist_ok=True)
    phases=json.loads((compact/'execution_phase_receipts.json').read_text())['receipts'];side=next(p for p in phases if p['skill_id']=='auxiliary_side_compaction');start=next(p['end_sim_time_s'] for p in phases if p['skill_id']=='placement_release')
    visual=rows(compact/'visual_detections.jsonl');timeline={r['physics_step']:r for r in rows(compact/'r2b_camera/camera_timeline.jsonl')};trace=rows(args.raw/'physical_trace/physical_trace.jsonl');by_step={r['physics_step']:r for r in trace}
    selected=[r for r in visual if side['physics_step_interval'][0]<=r['physics_step']<=side['physics_step_interval'][1]];missing=[r for r in selected if r['status']!='PRESENT']
    review=json.loads((r3/'config/p2a_visual_review.json').read_text());assert set(review['labels'])=={str(r['physics_step']) for r in missing}
    K=np.array(json.loads((r3/'config/p1_frozen.json').read_text())['K']);records=[]
    for r in missing:
        t=timeline[r['physics_step']];image=args.raw/'r2b_camera'/r['rgb'];depth=args.raw/'r2b_camera'/r['depth'];im=cv2.imread(str(image));assert im is not None
        dep=np.load(depth,mmap_mode='r',allow_pickle=False);d=detect_aruco(im,K,np.zeros(5),.06,23)
        records.append({'physics_step':r['physics_step'],'receipt_time_s':r['time_s'],'rgb':r['rgb'],'rgb_sha256':sha(image),'depth':r['depth'],'depth_sha256':sha(depth),'depth_finite_fraction':float(np.isfinite(dep).mean()),'frame_pair_ok':t['frame_pair_ok'],'historical_status':r['status'],'redetected_status':d.status,**review['labels'][str(r['physics_step'])]})
    with (args.out/'107_frame_review.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(records[0]),lineterminator="\n");writer.writeheader();writer.writerows(records)
    for page in range((len(missing)+23)//24):
        for kind in ('full','crop'):
            canvas=Image.new('RGB',(1280,6*212),'white');draw=ImageDraw.Draw(canvas)
            for j,r in enumerate(missing[page*24:(page+1)*24]):
                im=Image.open(args.raw/'r2b_camera'/r['rgb']).convert('RGB')
                if kind=='crop':im=im.crop((680,340,960,536))
                im.thumbnail((320,180));x=j%4*320;y=j//4*212;canvas.paste(im,(x,y));draw.text((x+3,y+182),f"#{page*24+j:03d} step={r['physics_step']} t={r['time_s']:.3f}",fill='black')
            canvas.save(args.out/f'{kind}_sheet_{page:02d}.jpg',quality=93)
    for r in missing[::26]:shutil.copy2(args.raw/'r2b_camera'/r['rgb'],args.out/f"representative_{r['physics_step']}.png")
    stage=[r for r in visual if start<=r['time_s']<=trace[-1]['time_s']];intervals=absence_intervals(selected);stage_intervals=absence_intervals(stage)
    coverage={}
    for gap in stage_intervals:
        subset=[r for r in trace if gap['first_missing_s']<=r['time_s']<=gap['last_missing_s']]
        gap['suction_boolean_rows']={s:sum(isinstance((r.get('suction_state') or {}).get(s),bool) for r in subset) for s in ('left','right')}
        gap['distinct_controller_sequences']=len({r['controller_command']['sequence'] for r in subset if r.get('controller_command')})
        gap['robot_rows']=len(subset);gap['observable_available_rows']={k:sum(r.get(k) is not None for r in subset) for k in ('q_actual_rad','dq_actual_rad_s','tcp_world_m','suction_state','suction_command','controller_command')}
    selected_t=[timeline[r['physics_step']] for r in selected]
    result={'schema':'r3.p2a.occlusion_audit.v1','raw_root':str(args.raw),'source_sha256':{str(p.relative_to(ROOT)):sha(p) for p in (compact/'visual_detections.jsonl',compact/'execution_phase_receipts.json',compact/'r2b_camera/camera_timeline.jsonl',r3/'config/p2a_visual_review.json')},
        'all_rgb_files':len(list((args.raw/'r2b_camera/rgb').glob('*.png'))),'all_depth_files':len(list((args.raw/'r2b_camera/depth').glob('*.npy'))),
        'side_compaction':{'sampled_images':len(selected),'detected':sum(r['status']=='PRESENT' for r in selected),'nondetections':len(missing),'pixel_review_causes':dict(Counter(r['cause'] for r in records)),
            'successful_acquisitions':sum(t['frame_pair_ok'] for t in selected_t),'missing_rgb_files':sum(not (args.raw/'r2b_camera'/r['rgb']).is_file() for r in selected),'missing_depth_files':sum(not (args.raw/'r2b_camera'/r['depth']).is_file() for r in selected),
            'same_default_detector_reproduced_nondetection':sum(r['redetected_status']!='PRESENT' for r in records),'scheduler_skipped_nominal_slots':sum(t.get('scheduler_skipped_intervals',0) for t in selected_t),
            'intercapture_s_median_max':np.percentile(np.diff([r['time_s'] for r in selected]),[50,100]).tolist(),'absence_intervals':intervals},
        'stage_b':{'start_s_from_release_receipt':start,'last_trace_s':trace[-1]['time_s'],'sampled_images':len(stage),'detected':sum(r['status']=='PRESENT' for r in stage),'absence_intervals':stage_intervals},
        'legacy_timestamp_semantics_check':{k:dict(zip(('median','p95','max'),map(float,np.percentile([t[k]-t['capture_request_wall_s'] for t in timeline.values() if t.get(k) is not None],[50,95,100])))) for k in ('rendered_sensor_frame_wall_s','depth_acquisition_wall_s','writer_complete_wall_s')},
        'phase_end_caveat':{'executor_wall_fit_end_sim_s':side['end_sim_time_s'],'trace_end_s':trace[-1]['time_s'],'policy':'clip to actual recorded simulation; wall-regression extrapolation is not extra image-acquisition coverage'},
        'review_policy':review['review_basis'],'legacy_tcp':'USD link readback, not independent FK; q/dq remain available for future FK',
        'classification_limit':'88 probable wall-border occlusions from pixels; 19 UNKNOWN marginal border/rasterization/decoder cases. No tool/arm complete occlusion or out-of-FOV assigned from detection failure alone.'}
    (args.out/'summary.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result['side_compaction'].items() if k!='absence_intervals'},indent=2))


if __name__=='__main__':main()
