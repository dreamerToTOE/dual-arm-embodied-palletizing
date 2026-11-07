#!/usr/bin/env python3
"""Decode raw time targets, freeze calibration, then score independent holdout."""
import argparse,collections,csv,hashlib,json,sys,time
from pathlib import Path
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from research.stage_b_multisensor_r3.r1_time_code import SOURCES,decode_clock,freeze_offset_interval,timestamp_interface


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def rows(path):return [json.loads(line) for line in path.read_text().splitlines()]
def dump(path,data):path.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
def stats(values):
    return dict(zip(('median','p95','max'),map(float,np.percentile(values,[50,95,100])))) if values else None


def evaluate(run,out,freeze=None,binding_path=None):
    out.mkdir(parents=True,exist_ok=False)
    m=json.loads((run/'manifest.json').read_text())
    for name,digest in m['data_sha256'].items():assert sha(run/name)==digest,name
    observations=rows(run/'observations.jsonl');history=rows(run/'physics_history.jsonl')
    physics={r['physics_step']:r for r in history};commands={r['physics_step']:r for r in rows(run/'stimulus_commands.jsonl')}
    tick=float(np.median(np.diff([r['time_s'] for r in history])))
    assert np.max(abs(np.diff([r['time_s'] for r in history])-tick))<1e-9
    seen={};correspondence=[];offsets=[];valid=[];counts=collections.Counter();phases=collections.Counter()
    for i,r in enumerate(observations):
        assert sha(run/r['rgb'])==r['rgb_sha256'] and sha(run/r['depth'])==r['depth_sha256']
        rgb=cv2.imread(str(run/r['rgb']),cv2.IMREAD_UNCHANGED)
        with np.load(run/r['depth']) as archive:depth=archive['distance_to_image_plane']
        assert rgb.shape[:2]==depth.shape
        decoded=decode_clock(rgb,depth,m['clock_layout'])
        paired=all(d['paired'] for d in decoded.values());counts['rgb_depth_paired_captures']+=paired
        row=dict(capture_index=i,frame_id=r['frame_id'],mode=r['mode'],paired=paired,
            sdk_time_s=r['sdk_time_s'],fabric_sim_time_s=r['fabric_sim_time_s'],
            reference_n=r['reference_time']['referenceTimeNumerator'],reference_d=r['reference_time']['referenceTimeDenominator'],
            callback_physics_step=r['callback_physics_step'],arrival_physics_time_s=r['arrival_physics_time_s'],
            callback_wall_s=r['callback_wall_s'],arrival_wall_s=r['arrival_wall_s'],writer_start_wall_s=r['writer_start_wall_s'],writer_complete_wall_s=r['writer_complete_wall_s'])
        for source in SOURCES:
            for k,v in decoded[source].items():row[source+'_'+k]=v
        assert r['reference_time']==r['dispatcher_reference'],'ReferenceTime disagrees with graph dispatcher'
        if r['mode']=='pattern':
            row['accepted']=False;row['reason']='pattern_control'
            if r['pattern_settled']:
                counts['settled_pattern_checks']+=1
                counts['settled_pattern_correct']+=paired and all(d['rgb_code']==r['expected_pattern'] for d in decoded.values())
        else:
            counts['dynamic_captures']+=1
            identity=(r['rgb_sha256'],r['depth_sha256'],r['sdk_time_s'],json.dumps(r['reference_time'],sort_keys=True))
            duplicate=r['frame_id'] in seen
            if duplicate:
                counts['duplicate_dynamic_captures']+=1
                assert identity==seen[r['frame_id']],'same SDK frame id with different content/metadata'
            seen[r['frame_id']]=identity
            post=decoded['post']['rgb_code'];pre=decoded['pre']['rgb_code'];app=decoded['app']['rgb_code']
            reason='duplicate' if duplicate else ('rgb_depth_unpaired' if not paired else
                'code_not_in_dynamic_history' if post not in physics or app not in commands else
                'pre_post_disagree' if pre!=post else None)
            row['accepted']=reason is None;row['reason']=reason
            if reason is None:
                t=physics[post]['time_s'];offset=r['fabric_sim_time_s']-t
                row.update(image_post_epoch_s=t,image_time_lower_s=t,image_time_upper_s=t+tick,
                    sdk_minus_image_epoch_s=offset,app_epoch_s=physics[app]['time_s'],post_minus_app_ticks=post-app,
                    phase=commands[app]['phase'],velocity_m_s=commands[app]['command_velocity_m_s'])
                valid.append((r,row));offsets.append(offset);phases[(row['phase'],row['velocity_m_s'])]+=1
        correspondence.append(row)
    assert len(seen)==counts['dynamic_captures']-counts['duplicate_dynamic_captures']
    pattern_ok=counts['settled_pattern_checks']>0 and counts['settled_pattern_correct']==counts['settled_pattern_checks']
    config=json.loads((run/'config_snapshot.json').read_text())
    full_phase_coverage=set(range(len(config[m['split']]))).issubset({p for p,_ in phases})
    binding=json.loads(binding_path.read_text()) if binding_path else None
    if freeze:
        assert m['split']=='calibration' and binding is None
        assert pattern_ok and valid and full_phase_coverage,'cannot calibrate an unverified/undecodable/incomplete clock sequence'
        binding={'schema':'r3.p2a_r1.frozen_time_interval.v1','status':'FROZEN_CALIBRATION',
            'configuration_fingerprint':m['configuration_fingerprint'],'physics_tick_s':tick,
            'offset_interval_s':freeze_offset_interval(offsets,tick),'observed_epoch_offset_range_s':[min(offsets),max(offsets)],
            'image_code_quantization_interval_s':[0,tick],
            'method':'Envelope of Fabric time minus independently decoded POST epoch; one full physics tick retained as quantization. No Cube oracle input.',
            'calibration_run':run.name,'calibration_manifest_sha256':sha(run/'manifest.json'),
            'calibration_observations_sha256':sha(run/'observations.jsonl'),'calibration_valid_unique_frames':len(valid),
            'created_unix_s':time.time(),'limitations':'Empirical bound for this fingerprint only; variable delay, no point capture time or online fusion authorization.'}
        with freeze.open('x') as f:f.write(json.dumps(binding,indent=2)+'\n')
        binding_path=freeze
    if binding:
        assert m['configuration_fingerprint']==binding['configuration_fingerprint']
        if m['split']=='holdout':assert m['binding_sha256_at_capture']==sha(binding_path),'binding was not frozen before capture'
    exports=[];containment=[];worst=[];midpoint_errors=[]
    for r,row in zip(observations,correspondence):
        if row['mode']!='dynamic' or row['reason']=='duplicate':continue
        observation=dict(r,configuration_fingerprint=m['configuration_fingerprint'],binding_sha256=sha(binding_path) if binding else None,time_binding_ready=row['accepted'])
        interface=timestamp_interface(observation,binding or {})
        interface['image_clock_interval_s']=[row['image_time_lower_s'],row['image_time_upper_s']] if row['accepted'] else None
        interface['image_clock_status']='INDEPENDENT_RGB_DEPTH_CODE_INTERVAL' if row['accepted'] else 'UNRESOLVED_TRANSITION'
        interface['capture_index']=row['capture_index']
        exports.append(interface)
        if interface['measurement_time_interval_s']:
            lo,hi=interface['measurement_time_interval_s'];a,b=interface['image_clock_interval_s'];mid=interface['candidate_midpoint_s']
            containment.append(lo<=a+1e-8 and hi>=b-1e-8)
            worst.append(max(abs(mid-a),abs(mid-b)));midpoint_errors.append(abs(mid-(a+b)/2))
            row.update(predicted_lower_s=lo,predicted_upper_s=hi,interval_contains_visual_interval=containment[-1],worst_midpoint_error_bound_s=worst[-1])
    # Predictions and binding are durable before the evaluation-only oracle is opened.
    (out/'timestamp_interface.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in exports))
    fields=list(dict.fromkeys(k for r in correspondence for k in r))
    with (out/'frame_correspondence.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fields,lineterminator='\n');writer.writeheader();writer.writerows(correspondence)
    prediction_sha=sha(out/'timestamp_interface.jsonl')
    oracle=rows(run/'oracle.jsonl')
    xyz=np.array([r['cube_xyz_m'] for r in oracle]);quats=np.array([r['cube_quaternion_wxyz'] for r in oracle])
    scope={'oracle_used_only_after_prediction_export':True,'z_range_m':[float(xyz[:,2].min()),float(xyz[:,2].max())],
        'all_kinematic':all(r['kinematic_enabled'] for r in oracle)} if oracle else {}
    if oracle:
        scope['max_quaternion_distance_from_identity']=float(np.max(np.minimum(np.linalg.norm(quats-np.array([1,0,0,0]),axis=1),np.linalg.norm(quats+np.array([1,0,0,0]),axis=1))))
        scope['x_range_m']=[float(xyz[:,0].min()),float(xyz[:,0].max())]
        oracle_steps={r['physics_step']:r for r in oracle};errors=[]
        for command in commands.values():
            for n in (1,2):
                state=oracle_steps.get(command['physics_step']+n)
                if state:errors.append(float(np.linalg.norm(np.array(state['cube_xyz_m'])-command['cube_command_xyz_m'])))
        scope['command_vs_next_two_physics_steps_error_m']=stats(errors)
    sdk_fabric=[abs(r['sdk_time_s']-r['fabric_sim_time_s']) for r in observations]
    independent_holdout=m['split']=='holdout' and binding is not None and binding['calibration_run']!=run.name
    decision='PASS' if independent_holdout and full_phase_coverage and pattern_ok and valid and containment and all(containment) and max(worst)<=tick+1e-8 else 'PARTIAL' if valid else 'INCONCLUSIVE'
    result={'schema':'r3.p2a_r1.time_evaluation.v1','decision':decision,'run':run.name,'split':m['split'],'counts':dict(counts),
        'unique_dynamic_frames':len(seen),'valid_unique_dynamic_frames':len(valid),'pattern_control_pass':pattern_ok,
        'full_phase_coverage':full_phase_coverage,'independent_holdout':independent_holdout,
        'rejection_reasons':dict(collections.Counter(r['reason'] for r in correspondence if r['mode']=='dynamic' and not r['accepted'])),
        'physics_tick_s':tick,'offset_histogram_ticks':dict(collections.Counter(str(round(v/tick,6)) for v in offsets)),
        'offset_ms':stats([v*1000 for v in offsets]),'sdk_fabric_max_difference_s':max(sdk_fabric),
        'callback_core_minus_sdk_max_s':max(abs(r['callback_core_time_s']-r['sdk_time_s']) for r in observations),
        'reference_dispatcher_all_equal':True,'phase_coverage':[{'phase':p,'velocity_m_s':v,'unique_frames':n} for (p,v),n in sorted(phases.items())],
        'post_minus_app_tick_histogram':dict(collections.Counter(str(r['post_minus_app_ticks']) for _,r in valid)),
        'empirical_interval_contains_visual_interval':{'covered':sum(containment),'total':len(containment)},
        'candidate_midpoint_vs_visual_interval_midpoint_error_ms':stats([v*1000 for v in midpoint_errors]),
        'worst_midpoint_error_bound_ms':max(worst)*1000 if worst else None,
        'frozen_interval_width_ms':(binding['offset_interval_s'][1]-binding['offset_interval_s'][0])*1000 if binding else None,
        'arrival_minus_sdk_ms':stats([(r['arrival_physics_time_s']-r['sdk_time_s'])*1000 for r,_ in valid]),
        'all_dynamic_arrival_minus_sdk_ms':stats([(r['arrival_physics_time_s']-r['sdk_time_s'])*1000 for r in observations if r['mode']=='dynamic']),
        'arrival_minus_image_epoch_ms':stats([(r['arrival_physics_time_s']-row['image_post_epoch_s'])*1000 for r,row in valid]),
        'callback_to_arrival_wall_ms':stats([(r['arrival_wall_s']-r['callback_wall_s'])*1000 for r,_ in valid]),
        'writer_duration_wall_ms':stats([(r['writer_complete_wall_s']-r['writer_start_wall_s'])*1000 for r,_ in valid]),
        'scope_oracle':scope,'prediction_sha256':prediction_sha,'manifest_sha256':sha(run/'manifest.json'),
        'binding_sha256':sha(binding_path) if binding else None,'no_cube_oracle_in_calibration':True,
        'note':'Interval is empirical, configuration-specific, and includes one tick of visual-code quantization. Point measurement_time_s remains null; online_fusion_eligible remains false.'}
    dump(out/'summary.json',result);return result


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--run',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);ap.add_argument('--freeze',type=Path);ap.add_argument('--binding',type=Path);a=ap.parse_args()
    print(json.dumps(evaluate(a.run,a.out,a.freeze,a.binding),indent=2))
