#!/usr/bin/env python3
"""Independent recorded holdout: causal coverage, three lookup methods, isolated scoring."""
import argparse, bisect, collections, json, sys
from pathlib import Path
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
R3=ROOT/'research/stage_b_multisensor_r3'
from offline.task28_g1_1.task28_validator.model import RobotModel
from research.stage_b_multisensor_r3.history_contract import identity, IDENTITY
from research.stage_b_multisensor_r3.history_contract_r1 import ReadinessHistory, decode_birth, interpolate_rows
from research.stage_b_multisensor_r3.r1_time_code import decode_clock
from research.stage_b_multisensor_r3.r2_scene_binding import estimate_cube_pixels
from research.stage_b_multisensor_r3.scripts.replay_p2b0_history import sha, rows, dump, save_rows, stats, transform_error


def passed(r,cfg):
    return all(r[k]<=cfg[c] for k,c in [('q_error_rad','interpolation_q_tolerance_rad'),('dq_error_rad_s','interpolation_dq_tolerance_rad_s'),('tcp_error_m','interpolation_tcp_tolerance_m'),('rotation_error_rad','interpolation_rotation_tolerance_rad')])


def metrics(records,cfg):
    good=[r for r in records if r['status']!='UNKNOWN']
    return {'queries':len(records),'accepted':len(good),'rejected':len(records)-len(good),
        'within_frozen_thresholds':sum(passed(r,cfg) for r in good),
        'q_error_rad':stats([r['q_error_rad'] for r in good]),'dq_error_rad_s':stats([r['dq_error_rad_s'] for r in good]),
        'tcp_error_mm':stats([r['tcp_error_m']*1000 for r in good]),'rotation_error_rad':stats([r['rotation_error_rad'] for r in good])}


def evaluate(run,out):
    out.mkdir(parents=True,exist_ok=False);m=json.loads((run/'manifest.json').read_text());cfg=json.loads((run/'config_snapshot.json').read_text())
    assert sha(run/'config_snapshot.json')==m['frozen_eval_config_sha256']==sha(R3/'config/p2b0_r1_frozen.json')
    assert sha(R3/'history_contract_r1.py')==m['frozen_method_sha256']==sha(run/'method_snapshot.py')
    for n,h in m['data_sha256'].items():
        if not n.startswith('oracle'):assert sha(run/n)==h
    robot=rows(run/'robot_history.jsonl');physics=rows(run/'physics_history.jsonl');observations=rows(run/'observations.jsonl')
    times={r['physics_step']:r['time_s'] for r in physics};epochs={identity(e):e for e in rows(run/'camera_epochs.jsonl')}
    assert all(abs(times[r['physics_step']]-r['time_s'])<1e-9 for r in robot)
    model=RobotModel();history=ReadinessHistory(robot,model.fk,cfg);groups={k:[r for r in robot if identity(r)==k] for k in epochs}
    states=[]
    for r in robot:
        v=history.exact_at(r,r['time_s']);assert v['status']=='ALIGNED'
        states.append(dict(v,physics_step=r['physics_step']))
    save_rows(out/'exact_robot_states.jsonl',states)
    associations=[];visual=[];frames_by_epoch=collections.defaultdict(list)
    K=np.array(m['K']);M=np.array(m['world_from_camera']);p1=json.loads((R3/'config/p1_frozen.json').read_text())
    for f in observations:
        assert sha(run/f['rgb'])==f['rgb_sha256'] and sha(run/f['depth'])==f['depth_sha256']
        rgb=cv2.imread(str(run/f['rgb']));depth=np.load(run/f['depth'],allow_pickle=False)['distance_to_image_plane']
        birth=decode_birth(rgb,depth,m['clock_layout']);clock=decode_clock(rgb,depth,m['clock_layout'])
        association=history.associate_epoch(f,times,epochs[identity(f)],birth)
        associations.append(dict(association,mode=f['mode'],camera_stream_id=f['camera_stream_id']))
        pose=estimate_cube_pixels(rgb,depth,K,M,m['capture_roi'],p1);pose.pop('runtime_ms',None);pose.get('rgb_pose',{}).pop('runtime_ms',None)
        visual.append(dict(f,birth=birth,clock=clock,cube_pose=pose,
            measurement_time_s=None,online_fusion_eligible=False,restricted_robot_input='EXACT_MEASURED_ONLY',
            image_epoch_accepted=association['status']=='UNCERTAIN'))
        frames_by_epoch[identity(f)].append((f,birth))
    save_rows(out/'visual_history.jsonl',visual);save_rows(out/'associations.jsonl',associations)
    # Exercise a retained real old image against the new epoch, including metadata relabelling.
    old_key,new_key=sorted(epochs);old_frame,old_birth=frames_by_epoch[old_key][-1];new_epoch=epochs[new_key]
    retired=history.associate_epoch(old_frame,times,new_epoch,old_birth)
    relabelled=dict(old_frame,**{k:new_epoch[k] for k in IDENTITY},camera_stream_id=new_epoch['camera_stream_id'])
    relabelled_result=history.associate_epoch(relabelled,times,new_epoch,old_birth)
    dump(out/'stale_buffer_negative.json',{'old_frame_id':old_frame['frame_id'],'old_birth':old_birth,'new_epoch':new_epoch,
        'retired_identity':retired,'relabelled_old_pixels':relabelled_result})
    predictions=[]
    for key,data in groups.items():
        transition_steps=[b['physics_step'] for a,b in zip(data,data[1:]) if a['controller_command']!=b['controller_command']]
        for parity in cfg['holdout_parities']:
            endpoints=data[parity::2];endpoint_times=[r['time_s'] for r in endpoints];sparse=ReadinessHistory(endpoints,model.fk,cfg,expected_step_stride=2)
            for r in data:
                if r in endpoints or r['controller_command'] is None or not endpoint_times[0]<r['time_s']<endpoint_times[-1]:continue
                i=bisect.bisect_left(endpoint_times,r['time_s']);a,b=endpoints[i-1:i+1]
                near=any(abs(r['physics_step']-step)<=cfg['transition_near_ticks'] for step in transition_steps)
                moving=abs(r['controller_command']['target_velocity_rad_s'])>=cfg['stable_motion_min_speed_rad_s']
                stratum='transition' if near else ('stable_motion' if moving else 'stable_hold')
                for method in cfg['methods']:
                    result=sparse.exact_at(r,r['time_s']) if method=='exact' else interpolate_rows(a,b,r['time_s'],method,model.fk,cfg['max_interpolation_bracket_s'])
                    predictions.append(dict(result,**{k:r[k] for k in IDENTITY},physics_step=r['physics_step'],time_s=r['time_s'],
                        parity=parity,method=method,stratum=stratum,bracket_steps=[a['physics_step'],b['physics_step']],
                        constant_command_bracket=a['controller_command']==b['controller_command']))
    save_rows(out/'holdout_predictions.jsonl',predictions)
    pred_names=['exact_robot_states.jsonl','visual_history.jsonl','associations.jsonl','stale_buffer_negative.json','holdout_predictions.jsonl']
    dump(out/'prediction_manifest.json',{'schema':'r3.p2b0_r1.predictions.v1','oracle_inputs':[],
        'config_sha256':sha(run/'config_snapshot.json'),'method_sha256':m['frozen_method_sha256'],
        'capture_manifest_sha256':sha(run/'manifest.json'),'prediction_sha256':{n:sha(out/n) for n in pred_names},
        'source_sha256':{str(p.relative_to(ROOT)):sha(p) for p in [Path(__file__),R3/'history_contract_r1.py',R3/'history_contract.py',R3/'time_alignment.py',R3/'r2_scene_binding.py',ROOT/'offline/task28_g1_1/task28_validator/model.py']}})
    # Only now open isolated PhysX TCP reference; withheld q/dq never entered endpoints.
    assert sha(run/'oracle_tcp.jsonl')==m['data_sha256']['oracle_tcp.jsonl']
    truth={(identity(r),r['physics_step']):r for r in rows(run/'oracle_tcp.jsonl')}
    measured={(identity(r),r['physics_step']):r for r in robot}
    fk_scores=[]
    for r in states:
        for side in ('left','right'):
            e,a=transform_error(r['tcp_fk'][side],truth[(identity(r),r['physics_step'])]['tcp_physx'][side])
            fk_scores.append({'epoch_id':r['epoch_id'],'physics_step':r['physics_step'],'side':side,'position_error_m':e,'rotation_error_rad':a})
    scored=[]
    for r in predictions:
        s={k:v for k,v in r.items() if k not in ('q','dq','tcp_fk')}
        if r['status']!='UNKNOWN':
            target=measured[(identity(r),r['physics_step'])];ref=truth[(identity(r),r['physics_step'])]
            errors=[transform_error(r['tcp_fk'][side],ref['tcp_physx'][side]) for side in ('left','right')]
            s.update(q_error_rad=max(float(np.max(abs(np.array(r['q'][side])-target['q'][side]))) for side in ('left','right')),
                dq_error_rad_s=max(float(np.max(abs(np.array(r['dq'][side])-target['dq'][side]))) for side in ('left','right')),
                tcp_error_m=max(x[0] for x in errors),rotation_error_rad=max(x[1] for x in errors))
        scored.append(s)
    save_rows(out/'evaluation_fk.jsonl',fk_scores);save_rows(out/'evaluation_interpolation.jsonl',scored)
    accepted=[a for a in associations if a['status']=='UNCERTAIN'];rejected=[a for a in associations if a['status']!='UNCERTAIN']
    coverage={'raw_frames':len(associations),'accepted_frames':len(accepted),'rejected_frames':len(rejected),
        'rejection_reasons':dict(collections.Counter(a.get('reason') for a in rejected)),
        'full_grid_coverage_frames':sum(a['full_grid_robot_coverage'] for a in accepted),
        'grid_candidates':sum(len(a['physics_grid_candidates']) for a in accepted),
        'supported_grid_candidates':sum(a['supported_grid_candidates'] for a in accepted),
        'unsupported_acquisition_duration_s':stats([a['robot_acquisition_unsupported_duration_s'] for a in accepted]),
        'visual_uncertainty_width_s':stats([a['admissible_interval_s'][1]-a['admissible_interval_s'][0] for a in accepted]),
        'per_epoch':{str(key[1]):{'robot_samples':len(data),'missing_steps':sum(max(0,b['physics_step']-a['physics_step']-1) for a,b in zip(data,data[1:])),
            'first_time_s':data[0]['time_s'],'last_time_s':data[-1]['time_s']} for key,data in groups.items()}}
    method_scores={}
    for method in cfg['methods']:
        group=[r for r in scored if r['method']==method]
        method_scores[method]={'unfiltered':metrics(group,cfg),'constant_command_guarded':metrics([r for r in group if r['constant_command_bracket']],cfg),
            'strata':{s:metrics([r for r in group if r['stratum']==s],cfg) for s in ('transition','stable_motion','stable_hold')}}
        method_scores[method]['constant_command_guarded']['rejected_by_guard']=sum(not r['constant_command_bracket'] for r in group)
        method_scores[method]['constant_command_guarded']['all_queries']=len(group)
    checks={'complete_acquisition':len(accepted)>0 and all(a['full_grid_robot_coverage'] and a['robot_acquisition_unsupported_duration_s']<1e-9 for a in accepted),
        'no_missing_robot_steps':all(v['missing_steps']==0 for v in coverage['per_epoch'].values()),
        'fk':max(r['position_error_m'] for r in fk_scores)<=cfg['fk_position_tolerance_m'] and max(r['rotation_error_rad'] for r in fk_scores)<=cfg['fk_rotation_tolerance_rad'],
        'retired_buffers_rejected':retired['status']=='UNKNOWN' and relabelled_result['reason']=='old_or_unproven_image_birth',
        'same_source_clocks':all(abs(r['time_s']-r['core_sim_time_s'])<1e-9 for r in robot) and all(abs(f['sdk_time_s']-f['fabric_sim_time_s'])<1e-9 for f in observations),
        'no_capture_errors':not m['errors'], 'exact_never_fabricates_withheld_values':method_scores['exact']['unfiltered']['accepted']==0}
    interpolation_pass=any(method_scores[method]['unfiltered']['queries']>0 and method_scores[method]['unfiltered']['within_frozen_thresholds']==method_scores[method]['unfiltered']['queries'] for method in ('linear','hermite'))
    summary={'schema':'r3.p2b0_r1.readiness.v1','decision':'PASS' if all(checks.values()) and interpolation_pass else 'PARTIAL',
        'restricted_readiness':'PASS_EXACT_ONLY_A' if all(checks.values()) else 'BLOCKED_C',
        'p2b1_input_option':'A' if all(checks.values()) else 'C','dynamic_interpolation':'PASS' if interpolation_pass else 'PARTIAL',
        'checks':checks,'coverage':coverage,'robot_samples':len(robot),'motion_peak_to_peak_rad':{side:np.ptp([r['q'][side] for r in robot],axis=0).tolist() for side in ('left','right')},
        'fk':{'arm_samples':len(fk_scores),'position_error_mm':stats([r['position_error_m']*1000 for r in fk_scores]),'rotation_error_rad':stats([r['rotation_error_rad'] for r in fk_scores])},
        'methods':method_scores,'exact_full_history':{'lookup_samples':len(states),'lookup_acceptance':len(states),'q_dq_self_consistency_only':True,'independent_fk_scoring':'fk field'},
        'online_fusion_eligible':False,'validated_interpolation_granted':False,
        'continuous_candidate_policy':'Full visual interval retained. Exact lookup supports physics samples only; no snapping and no inferred continuous q/dq. Acquisition coverage is not continuous interpolation validation.',
        'unavailable_channels':cfg['unavailable_channels']}
    dump(out/'summary.json',summary)
    names=pred_names+['prediction_manifest.json','evaluation_fk.jsonl','evaluation_interpolation.jsonl','summary.json']
    dump(out/'replay_hashes.json',{n:sha(out/n) for n in names});return summary


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--run',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    print(json.dumps(evaluate(a.run,a.out),indent=2))
