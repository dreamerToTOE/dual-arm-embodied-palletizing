#!/usr/bin/env python3
"""Observable-only preparation/predictions precede isolated Cube/joint label scoring."""
import argparse,collections,json,sys
from pathlib import Path
import cv2,numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT));R3=ROOT/'research/stage_b_multisensor_r3'
from offline.task28_g1_1.task28_validator.model import RobotModel
from research.stage_b_multisensor_r3.history_contract import identity
from research.stage_b_multisensor_r3.history_contract_r1 import ReadinessHistory,decode_birth
from research.stage_b_multisensor_r3.r2_scene_binding import estimate_cube_pixels
from research.stage_b_multisensor_r3.offline_cube_estimator import OfflineCubeEstimator,ConnectionEstimator
from research.stage_b_multisensor_r3.scripts.replay_p2b0_history import sha,rows,dump,save_rows,stats,transform_error


def prepare(run,out,cfg):
    out.mkdir();m=json.loads((run/'manifest.json').read_text())
    assert sha(R3/'offline_cube_estimator.py')==m['frozen_estimator_sha256']==sha(run/'estimator_snapshot.py')
    assert sha(R3/'config/p2b1_estimator_frozen.json')==m['frozen_estimator_config_sha256']
    for n,h in m['data_sha256'].items():
        if not n.startswith('oracle'):assert sha(run/n)==h
    measured=rows(run/'robot_history.jsonl');physics=rows(run/'physics_history.jsonl');frames=rows(run/'observations.jsonl')
    times={r['physics_step']:r['time_s'] for r in physics};epochs={identity(e):e for e in rows(run/'camera_epochs.jsonl')}
    model=RobotModel();history=ReadinessHistory(measured,model.fk,cfg);states=[]
    for r in measured:
        s=history.exact_at(r,r['time_s']);assert s['status']=='ALIGNED'
        states.append(dict(s,physics_step=r['physics_step']))
    save_rows(out/'robot_inputs.jsonl',states)
    K=np.array(m['K']);M=np.array(m['world_from_camera']);p1=json.loads((R3/'config/p1_frozen.json').read_text());visual=[];assoc=[]
    for f in frames:
        assert sha(run/f['rgb'])==f['rgb_sha256'] and sha(run/f['depth'])==f['depth_sha256']
        rgb=cv2.imread(str(run/f['rgb']));depth=np.load(run/f['depth'],allow_pickle=False)['distance_to_image_plane'];birth=decode_birth(rgb,depth,m['clock_layout'])
        a=history.associate_epoch(f,times,epochs[identity(f)],birth);assoc.append(a)
        pose=estimate_cube_pixels(rgb,depth,K,M,m['capture_roi'],p1);pose.pop('runtime_ms',None);pose.get('rgb_pose',{}).pop('runtime_ms',None)
        visual.append({k:f[k] for k in ('frame_id','sdk_time_s','arrival_physics_time_s','epoch_id','rgb_sha256','depth_sha256')} | {
            'cube_pose':pose,'image_epoch_accepted':a['status']=='UNCERTAIN','admissible_interval_s':a.get('admissible_interval_s'),
            'measurement_time_s':None,'birth':birth})
    save_rows(out/'visual_inputs.jsonl',visual);save_rows(out/'associations.jsonl',assoc)
    dump(out/'input_manifest.json',{'oracle_inputs':[],'run_manifest_sha256':sha(run/'manifest.json'),
        'files_sha256':{n:sha(out/n) for n in ['robot_inputs.jsonl','visual_inputs.jsonl','associations.jsonl']},
        'pixel_estimator_sha256':sha(R3/'r2_scene_binding.py'),'estimator_sha256':sha(R3/'offline_cube_estimator.py'),
        'coverage':{'frames':len(assoc),'accepted':sum(a['status']=='UNCERTAIN' for a in assoc),
            'full_grid_coverage':sum(a.get('full_grid_robot_coverage',False) for a in assoc),
            'present_visual':sum(v['cube_pose']['status']=='PRESENT' for v in visual)}})
    return states,visual,m


def predict_case(states,visual,mask,cfg,path):
    estimators={name:OfflineCubeEstimator(name,cfg) for name in cfg['method_names']+cfg['ablation_names']}
    start=states[0]['time_s'];i=0;history=[];output=[];visibility=[]
    for r in states:
        history.append(r);events=[]
        while i<len(visual) and visual[i]['arrival_physics_time_s']<=r['time_s']+1e-8:
            v=visual[i];relative=v['arrival_physics_time_s']-start;hidden=any(lo<=relative<=hi for lo,hi in mask)
            visibility.append({'frame_id':v['frame_id'],'arrival_relative_s':relative,'hidden_by_offline_mask':hidden,
                'detected':v['cube_pose']['status']=='PRESENT','interval_s':v['admissible_interval_s']})
            if not hidden:events.append(v)
            i+=1
        for name,e in estimators.items():
            prediction=e.step(r,events,history)
            output.append(dict(prediction,method=name,physics_step=r['physics_step'],time_s=r['time_s'],relative_time_s=r['time_s']-start))
    save_rows(path,output);return output,visibility


def score_case(predictions,truth,mask,plan):
    result={}
    for method in sorted({p['method'] for p in predictions}):
        selected=[p for p in predictions if p['method']==method];valid=[p for p in selected if p['status']=='ESTIMATED'];errors=[];matrix=collections.Counter();covered=[];occlusion=[]
        for p in valid:
            target=truth[p['physics_step']];error=float(np.linalg.norm(np.array(p['xy_m'])-target['cube_xyz_m'][:2]));errors.append(error)
            label='CONNECTED' if target['fixture_joint_enabled'] else 'DISCONNECTED';state=p['connection']['state'];matrix[(label,state)]+=1
            covered.append(error<=p['position_uncertainty_radius_m'])
            if any(a<=p['relative_time_s']<=b for a,b in mask):occlusion.append((p['relative_time_s'],error))
        recovery=[]
        for _,end in mask:
            after=[p for p in valid if end<p['relative_time_s']<=end+plan['recovery_window_s']]
            if not after:continue
            es=[float(np.linalg.norm(np.array(p['xy_m'])-truth[p['physics_step']]['cube_xyz_m'][:2])) for p in after]
            hits=[(p,e) for p,e in zip(after,es) if e<=plan['correction_threshold_m']]
            first_good=hits[0][0] if hits else None
            recovery.append({'mask_end_s':end,'first_estimated_after_mask_error_mm':es[0]*1000,
                'first_visual_update_after_recovery_error_mm':next((e*1000 for p,e in zip(after,es) if p['new_visual_updates']),None),
                'time_to_under_3mm_s':first_good['relative_time_s']-end if first_good else None,
                'visual_updates_to_under_3mm':sum(p['new_visual_updates'] for p in after if first_good is not None and p['time_s']<=first_good['time_s']) if first_good else None})
        connected=sum(n for (g,_),n in matrix.items() if g=='CONNECTED');disconnected=sum(n for (g,_),n in matrix.items() if g=='DISCONNECTED')
        result[method]={'samples':len(selected),'predictions':len(valid),'prediction_coverage':len(valid)/len(selected),
            'xy_error_mm':stats(np.array(errors)*1000),'uncertainty_radius_coverage_fraction':float(np.mean(covered)) if covered else None,
            'unknown_rate':sum(p['connection']['state']=='UNKNOWN' for p in valid)/len(valid) if valid else None,
            'connection_confusion':{g:{s:matrix[(g,s)] for s in ('CONNECTED','UNKNOWN','DISCONNECTED')} for g in ('CONNECTED','DISCONNECTED')},
            'false_disconnect_alarms':matrix[('CONNECTED','DISCONNECTED')],
            'strict_missed_disconnects_including_unknown':sum(n for (g,s),n in matrix.items() if g=='DISCONNECTED' and s!='DISCONNECTED'),
            'connected_label_samples':connected,'disconnected_label_samples':disconnected,
            'occlusion_error_mm':stats([e*1000 for _,e in occlusion]),
            'occlusion_first_last_error_mm':[occlusion[0][1]*1000,occlusion[-1][1]*1000] if occlusion else None,
            'recovery':recovery}
    return result


def evaluate(runs,out):
    out.mkdir(parents=True,exist_ok=False);cfg=json.loads((R3/'config/p2b1_frozen.json').read_text());estcfg=json.loads((R3/'config/p2b1_estimator_frozen.json').read_text());plan=json.loads((R3/'config/p2b1_evaluation_frozen.json').read_text())
    all_cases={};prepared={};prediction_files=[]
    for run in runs:
        input_cfg=dict(cfg,history_max_samples=plan['offline_history_max_samples'])
        states,visual,m=prepare(run,out/run.name,input_cfg);prepared[run.name]=(states,visual,m)
        for case in plan['case_plan'][m['condition']]:
            path=out/run.name/(case+'_predictions.jsonl');pred,visibility=predict_case(states,visual,plan['masks'][case],estcfg,path)
            save_rows(out/run.name/(case+'_visibility.jsonl'),visibility);prediction_files.append(path)
            all_cases[run.name+'/'+case]={'predictions':pred,'run':run,'mask':plan['masks'][case]}
    # Freeze all predictions for every episode/ablation before opening any holdout oracle.
    dump(out/'prediction_manifest.json',{'oracle_inputs':[],'estimator_config_sha256':sha(R3/'config/p2b1_estimator_frozen.json'),
        'evaluation_config_sha256':sha(R3/'config/p2b1_evaluation_frozen.json'),'estimator_source_sha256':sha(R3/'offline_cube_estimator.py'),
        'evaluator_source_sha256':sha(Path(__file__)),'files_sha256':{str(p.relative_to(out)):sha(p) for p in prediction_files}})
    scores={};scope={};fk={}
    for run in runs:
        states,_,m=prepared[run.name]
        for n,h in m['data_sha256'].items():
            if n.startswith('oracle'):assert sha(run/n)==h
        truth={r['physics_step']:r for r in rows(run/'oracle.jsonl')};tcp={r['physics_step']:r for r in rows(run/'oracle_tcp.jsonl')}
        z=[r['cube_xyz_m'][2] for r in truth.values()];tilt=[np.degrees(np.arccos(np.clip(Rotation.from_quat(np.roll(r['cube_quaternion_wxyz'],-1)).as_matrix()[2,2],-1,1))) for r in truth.values()]
        scope[run.name]={'table_z_min_max_m':[min(z),max(z)],'tilt_max_deg':max(tilt),'table_support_pass':max(abs(np.array(z)-.26))<=cfg['table_support_z_tolerance_m'] and max(tilt)<=cfg['table_tilt_limit_deg'],
            'joint_label_counts':dict(collections.Counter('CONNECTED' if r['fixture_joint_enabled'] else 'DISCONNECTED' for r in truth.values())),
            'cube_motion_peak_to_peak_m':np.ptp([r['cube_xyz_m'] for r in truth.values()],axis=0).tolist()}
        pe=[];ae=[]
        for s in states:
            for side in ('left','right'):
                p,a=transform_error(s['tcp_fk'][side],tcp[s['physics_step']]['tcp_physx'][side]);pe.append(p*1000);ae.append(a)
        fk[run.name]={'position_error_mm':stats(pe),'rotation_error_rad':stats(ae)}
        # Independently verify enabled fixture coupling, not just its software flag.
        first=min(set(truth)&set(tcp));C0=np.eye(4);C0[:3,:3]=Rotation.from_quat(np.roll(truth[first]['cube_quaternion_wxyz'],-1)).as_matrix();C0[:3,3]=truth[first]['cube_xyz_m']
        local=np.linalg.inv(np.array(tcp[first]['tcp_physx']['left']))@C0;relative=[]
        for step,c in truth.items():
            if c['fixture_joint_enabled'] and step in tcp:
                relative.append(float(np.linalg.norm((np.array(tcp[step]['tcp_physx']['left'])@local)[:3,3]-c['cube_xyz_m']))*1000)
        scope[run.name]['enabled_fixture_relative_position_error_mm']=stats(relative)
        for name,c in all_cases.items():
            if c['run']==run:scores[name]=score_case(c['predictions'],truth,c['mask'],plan)
    # Legacy physical07: only actually recorded observables, no invented connection labels.
    legacy=rows(R3/'evidence/p2b0_physical07/robot_observables.jsonl');connection=ConnectionEstimator(estcfg)
    legacy_states=[connection.observe(None,[],r['suction_state'])['state'] for r in legacy]
    summary={'decision':'PARTIAL','scope_checks':scope,'independent_fk':fk,'cases':scores,
        'legacy_physical07':{'rows':len(legacy),'unknown':legacy_states.count('UNKNOWN'),'observed_visual_absence_s':14.533334091305733,
            'independent_attachment_labels_available':False,'cube_gt_exists_in_original_trace':True,'cube_gt_used_as_input':False,
            'xy_scoring_here':'not claimed: sanitized excerpt has no initial visual anchor and legacy image exposure association is unverified'},
        'claims':{'conditional_attached_pose_prediction':'evaluate K1 on independently enabled fixed-joint episodes only',
            'connection_identification':'not established positively; UNKNOWN when matching motion/software feedback does not discriminate',
            'complete_occlusion_reliability':'not established; evaluate strict missed losses and uncertainty coverage'},
        'input_restriction':'exact measured robot states only; full visual interval remains uncertain; no GT/contact labels in estimator',
        'source_limit':'controlled PhysX fixture, not validated physical suction contact; offline masks are observation loss, not a new optical wall experiment'}
    dump(out/'summary.json',summary);dump(out/'replay_hashes.json',{'prediction_manifest.json':sha(out/'prediction_manifest.json'),'summary.json':sha(out/'summary.json')});return summary


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--runs',type=Path,nargs='+',required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    print(json.dumps(evaluate(a.runs,a.out),indent=2))
