#!/usr/bin/env python3
"""R2 pixel predictions and command-derived calibration precede isolated GT scoring."""
import argparse,bisect,collections,csv,hashlib,json,sys,time
from pathlib import Path
import cv2,numpy as np
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
R3=ROOT/'research/stage_b_multisensor_r3'
from research.stage_b_multisensor_r3.r1_time_code import decode_clock
from research.stage_b_multisensor_r3.r2_scene_binding import estimate_cube_pixels,scene_time_interface
from research.stage_b_multisensor_r3.planar_pose_estimator import project,cube_marker_world_corners


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def rows(p):return [json.loads(s) for s in p.read_text().splitlines()]
def dump(p,value):p.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
def save_rows(p,value):p.write_text(''.join(json.dumps(r,allow_nan=False)+'\n' for r in value))
def stats(a):return dict(zip(('median','p95','max'),map(float,np.percentile(a,[50,95,100])))) if len(a) else None


def command_position(commands,step):
    i=max(0,bisect.bisect_left([r['physics_step'] for r in commands],step)-1)
    return np.array(commands[i]['cube_command_xyz_m'])


def evaluate(run,out,binding_path=None,freeze=None):
    out.mkdir(parents=True,exist_ok=False);m=json.loads((run/'manifest.json').read_text());cfg=json.loads((run/'config_snapshot.json').read_text())
    for name,digest in m['data_sha256'].items():
        if not name.startswith('oracle'):assert sha(run/name)==digest
    obs=rows(run/'observations.jsonl');history=rows(run/'physics_history.jsonl');commands=rows(run/'stimulus_commands.jsonl')
    times={r['physics_step']:r['time_s'] for r in history};tick=float(np.median(np.diff(list(times.values()))));K=np.array(m['K']);M=np.array(m['world_from_camera'])
    p1=json.loads((R3/'config/p1_frozen.json').read_text());r1=json.loads((R3/'config/r1_time_binding_frozen.json').read_text())
    predictions=[];seen={};controls=[];duplicate_count=0;paired_count=0
    for i,r in enumerate(obs):
        assert sha(run/r['rgb'])==r['rgb_sha256'] and sha(run/r['depth'])==r['depth_sha256']
        rgb=cv2.imread(str(run/r['rgb']));depth=np.load(run/r['depth'],allow_pickle=False)['distance_to_image_plane']
        clock=decode_clock(rgb,depth,m['clock_layout']);paired_count+=all(v['paired'] for v in clock.values())
        if r['mode']=='pattern':
            if r['pattern_settled']:controls.append(all(v['paired'] and v['rgb_code']==r['expected_pattern'] for v in clock.values()))
            continue
        identity=(r['rgb_sha256'],r['depth_sha256'],r['sdk_time_s'])
        if r['frame_id'] in seen:
            assert seen[r['frame_id']]==identity,'same SDK frame id has different data'
            duplicate_count+=1;continue
        seen[r['frame_id']]=identity
        pose=estimate_cube_pixels(rgb,depth,K,M,m['capture_roi'],p1);post=clock['post']['rgb_code'];app=clock['app']['rgb_code']
        clock_valid=all(v['paired'] for v in clock.values()) and post==clock['pre']['rgb_code'] and post in times and app in times
        record={'capture_index':i,'frame_id':r['frame_id'],'sdk_time_s':r['sdk_time_s'],'fabric_sim_time_s':r['fabric_sim_time_s'],
            'reference_time':r['reference_time'],'callback_entry_step':r['callback_entry_step'],'clock':clock,'clock_valid':clock_valid,'pose':pose,'rgb_sha256':r['rgb_sha256'],'depth_sha256':r['depth_sha256']}
        if clock_valid:
            command=min(commands,key=lambda c:abs(c['physics_step']-app));record.update(clock_step=post,clock_time_s=times[post],phase=command['phase'],velocity_m_s=command['command_velocity_m_s'])
        predictions.append(record)
    save_rows(out/'pixel_predictions.jsonl',predictions)
    tolerance=.00025
    binding=json.loads(binding_path.read_text()) if binding_path else None
    if freeze:
        assert m['split']=='calibration' and binding is None and all(controls) and controls
        informative=[]
        for r in predictions:
            if not r['clock_valid'] or r['pose']['status']!='PRESENT':continue
            n=r['clock_step'];matches=[s-n for s in range(n-6,n+3) if s in times and np.linalg.norm(np.array(r['pose']['cube_xyz_m'])[:2]-command_position(commands,s)[:2])<=tolerance]
            if 0<len(matches)<=2:informative.append({'frame_id':r['frame_id'],'clock_relative_ticks':matches,'phase':r['phase']})
        assert len(informative)>=30,'insufficient independently identifiable calibration motion'
        shifts=sorted({s for r in informative for s in r['clock_relative_ticks']})
        assert max(shifts)-min(shifts)<=6,'calibration did not establish a useful bounded transfer'
        assert set(range(len(cfg['calibration']))).issubset({r.get('phase') for r in predictions})
        binding={'schema':'r3.p2a_r2.scene_binding.v1','status':'FROZEN_CALIBRATION','configuration_fingerprint':m['configuration_fingerprint'],
            'cube_minus_clock_candidate_ticks':shifts,'pose_match_tolerance_m':tolerance,'precise_transfer_validated':False,
            'calibration_run':run.name,'calibration_manifest_sha256':sha(run/'manifest.json'),'created_unix_s':time.time(),
            'informative_calibration_frames':informative,'method':'Cube RGB-D pixels matched to independently logged kinematic commands around visual POST code; no Cube oracle used. Preserve the union of observed physics-step candidates.',
            'pixel_estimator_source_sha256':sha(R3/'r2_scene_binding.py'),'p1_config_sha256':sha(R3/'config/p1_frozen.json'),
            'depth_pixel_center_offset_px':.5,'scope':'Instrumented tabletop stimulus only; no point time for unresolved states; no online fusion.'}
        with freeze.open('x') as f:f.write(json.dumps(binding,indent=2)+'\n')
        binding_path=freeze
    assert binding,'freeze a calibration or supply the frozen model'
    assert binding['configuration_fingerprint']==m['configuration_fingerprint']
    assert binding['pixel_estimator_source_sha256']==sha(R3/'r2_scene_binding.py')
    if m['split']=='holdout':assert m['binding_sha256_at_capture']==sha(binding_path)
    interfaces=[]
    for r in predictions:
        frame=dict(r,configuration_fingerprint=m['configuration_fingerprint']);possible=[r['clock_step']+s for s in binding['cube_minus_clock_candidate_ticks']] if r['clock_valid'] else []
        frame['cube_matches_known_stimulus']=r['pose']['status']=='PRESENT' and any(s in times and np.linalg.norm(np.array(r['pose']['cube_xyz_m'])[:2]-command_position(commands,s)[:2])<=binding['pose_match_tolerance_m'] for s in possible)
        interface=scene_time_interface(frame,r['clock'],times,binding)
        interface['clock_conditioned_candidates']=interface['candidates']
        if interface['candidates']:
            # Revisited positions can alias distant past states; a local clock window is not independent proof.
            aliases=[{'physics_step':h['physics_step'],'time_s':h['time_s']} for h in history
                if h['mode']!='warmup' and h['physics_step']<=r['callback_entry_step'] and
                np.linalg.norm(np.array(r['pose']['cube_xyz_m'])[:2]-command_position(commands,h['physics_step'])[:2])<=binding['pose_match_tolerance_m']]
            interface['candidates']=aliases
            if any(a['physics_step'] not in possible for a in aliases):interface['timing_status']='MULTI_HYPOTHESIS_WITH_HISTORICAL_ALIASES'
            interface['timestamp_source']='PIXEL_CUBE_AND_CAUSAL_COMMAND_HISTORY_CLOCK_PRIOR_SEPARATE'
        interface.update(capture_index=r['capture_index'],cube_pose=r['pose'],binding_sha256=sha(binding_path),
            metadata_hypotheses=[{'sdk_shift_ticks':shift,'time_s':r['sdk_time_s']+shift*tick} for shift in (0,-2)],
            v1_r1_interval_s=[r['sdk_time_s']-r1['offset_interval_s'][1],r['sdk_time_s']-r1['offset_interval_s'][0]],
            v1_scope='FROZEN_R1_NUMERIC_REFERENCE_ONLY_FINGERPRINT_MISMATCH',cube_matches_known_stimulus=frame['cube_matches_known_stimulus'])
        interfaces.append(interface)
    save_rows(out/'time_binding_interface.jsonl',interfaces)
    # No Cube/Fabric oracle is parsed before predictions, frozen model and interfaces exist.
    for name,digest in m['data_sha256'].items():
        if name.startswith('oracle'):assert sha(run/name)==digest
    oracle=rows(run/'oracle.jsonl');probes=rows(run/'oracle_scene_probes.jsonl');truth={r['physics_step']:r for r in oracle}
    clock_modes=collections.Counter();scored=[];motion=[];body_projection=[];pose_errors=[];rgb_errors=[];candidate_coverage=[]
    for r,interface in zip(predictions,interfaces):
        row={'frame_id':r['frame_id'],'capture_index':r['capture_index'],'phase':r.get('phase'),'velocity_m_s':r.get('velocity_m_s'),
            'sdk_time_s':r['sdk_time_s'],'clock_step':r.get('clock_step'),'pose_status':r['pose']['status'],'timing_status':interface['timing_status']}
        if r['clock_valid']:clock_modes[str(round((r['sdk_time_s']-r['clock_time_s'])/tick,6))]+=1
        if r['pose']['status']=='PRESENT' and r['clock_valid']:
            xy=np.array(r['pose']['cube_xyz_m'])[:2];n=r['clock_step'];sdk_step=min(times,key=lambda s:abs(times[s]-r['sdk_time_s']))
            v1=[s for s,t in times.items() if interface['v1_r1_interval_s'][0]-1e-8<=t<=interface['v1_r1_interval_s'][1]+1e-8 and s in truth]
            v2=[c['physics_step'] for c in interface['candidates'] if c['physics_step'] in truth]
            groups={'V0':[sdk_step] if sdk_step in truth else [],'V1':v1,'V2':v2}
            for label,steps in groups.items():
                if steps:
                    es=[float(np.linalg.norm(xy-np.array(truth[s]['cube_xyz_m'])[:2])) for s in steps]
                    row[label+'_xy_error_min_m']=min(es);row[label+'_xy_error_max_m']=max(es)
            if n in truth:
                pose_errors.append(float(np.linalg.norm(xy-np.array(truth[n]['cube_xyz_m'])[:2])))
                rgb_errors.append(float(np.linalg.norm(np.array(r['pose']['rgb_pose']['cube_xyz_m'])[:2]-np.array(truth[n]['cube_xyz_m'])[:2])))
            projected=project(cube_marker_world_corners(*xy,r['pose']['xyyaw'][2]),K,M)
            body_projection.append(float(np.mean(np.linalg.norm(projected-np.array(r['pose']['corners_px']),axis=1))))
            # Complete causal history, not a window around the timestamp being tested.
            search=[s for s in truth if s<=r['callback_entry_step']]
            matching=[s for s in search if np.linalg.norm(xy-np.array(truth[s]['cube_xyz_m'])[:2])<=binding['pose_match_tolerance_m']]
            row['scoring_reference_steps']=json.dumps(matching)
            observable=bool(matching and len(matching)<=2 and matching==list(range(min(matching),max(matching)+1)))
            row['timing_observable_from_cube']=observable
            if matching and v2:candidate_coverage.append(bool(set(v2)&set(matching)))
            if observable:
                for label,steps in groups.items():
                    if steps:
                        ts=[times[s] for s in steps];ref=[times[s] for s in matching]
                        row[label+'_time_distance_min_s']=min(abs(a-b) for a in ts for b in ref)
                        row[label+'_time_error_worst_s']=max(abs(a-b) for a in ts for b in ref)
                motion.append(row)
            else:row['timing_unresolved_reason']='static_or_repeated_pose_or_insufficient_image_evidence'
        scored.append(row)
    with (out/'frame_correspondence.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,list(dict.fromkeys(k for r in scored for k in r)),lineterminator='\n');writer.writeheader();writer.writerows(scored)
    comparisons={}
    common=[r for r in scored if all(k+'_xy_error_max_m' in r for k in ('V0','V1','V2'))]
    for label in ('V0','V1','V2'):
        comparisons[label]={'matched_common_xy_frames':len(common),
            'xy_error_min_mm':stats([r[label+'_xy_error_min_m']*1000 for r in common]),'xy_error_max_mm':stats([r[label+'_xy_error_max_m']*1000 for r in common]),
            'time_observable_frames':sum(label+'_time_error_worst_s' in r for r in motion),
            'timestamp_set_min_distance_ms':stats([r[label+'_time_distance_min_s']*1000 for r in motion if label+'_time_distance_min_s' in r]),
            'timestamp_supported_worst_error_ms':stats([r[label+'_time_error_worst_s']*1000 for r in motion if label+'_time_error_worst_s' in r])}
    def probe_stats(phase):
        ps=[r for r in probes if r['phase']==phase];cube= [r['fabric_current_readback']['cube'] for r in ps]
        return {'samples':len(ps),'fabric_world_position_available':sum(v['world_position'] is not None for v in cube),
            'fabric_world_matrix_available':sum(v['world_matrix'] is not None for v in cube),
            'usdrt_authored_translate_available':sum(v['authored_translate_read_through_usdrt'] is not None for v in cube),
            'usd_vs_physx_translation_mm':stats([np.linalg.norm(np.array(r['cube_usd_matrix'])[3,:3]-r['cube_physx_xyz_m'])*1000 for r in ps]),
            'usdrt_authored_vs_physx_mm':stats([np.linalg.norm(np.array(r['fabric_current_readback']['cube']['authored_translate_read_through_usdrt'])-r['cube_physx_xyz_m'])*1000 for r in ps if r['fabric_current_readback']['cube']['authored_translate_read_through_usdrt'] is not None])}
    phase_results=[]
    for phase in sorted({r.get('phase') for r in scored if r.get('phase') is not None}):
        selected=[r for r in scored if r.get('phase')==phase];phase_results.append({'phase':phase,'velocity_m_s':selected[0]['velocity_m_s'],'unique_frames':len(selected),'cube_timing_observable':sum(r.get('timing_observable_from_cube',False) for r in selected),'v2_worst_time_error_ms':stats([r['V2_time_error_worst_s']*1000 for r in selected if 'V2_time_error_worst_s' in r])})
    scope={'all_kinematic':all(r['kinematic_enabled'] for r in oracle),'max_abs_z_error_m':max(abs(r['cube_xyz_m'][2]-.26) for r in oracle),'max_quaternion_error':max(np.linalg.norm(np.array(r['cube_quaternion_wxyz'])-[1,0,0,0]) for r in oracle)}
    command_errors=[np.linalg.norm(np.array(r['cube_xyz_m'])-command_position(commands,r['physics_step'])) for r in oracle if r['physics_step']>commands[0]['physics_step']]
    result={'schema':'r3.p2a_r2.scene_evaluation.v1','decision':'PARTIAL' if common and controls and all(controls) else 'INCONCLUSIVE','run':run.name,'split':m['split'],'raw_captures':len(obs),'unique_dynamic_frames':len(predictions),'duplicate_dynamic_captures':duplicate_count,
        'paired_clock_captures':paired_count,'settled_controls_correct':sum(controls),'settled_controls_total':len(controls),'pixel_pose_status':dict(collections.Counter(r['pose']['status'] for r in predictions)),
        'cube_qualified_time_interfaces':sum(bool(r['candidates']) for r in interfaces),'point_timestamps_exported':sum(r['measurement_time_s'] is not None for r in interfaces),
        'cube_timing_observable_frames':len(motion),'cube_timing_unresolved_frames':len(predictions)-len(motion),'sdk_minus_clock_ticks':dict(clock_modes),'comparisons':comparisons,
        'v2_reference_intersection':{'covered':sum(candidate_coverage),'tested':len(candidate_coverage)},'v2_candidate_span_ms':stats([(max(c['time_s'] for c in r['candidates'])-min(c['time_s'] for c in r['candidates']))*1000 for r in interfaces if r['candidates']]),
        'clock_conditioned_candidate_span_ms':stats([(max(c['time_s'] for c in r['clock_conditioned_candidates'])-min(c['time_s'] for c in r['clock_conditioned_candidates']))*1000 for r in interfaces if r['clock_conditioned_candidates']]),
        'interfaces_with_historical_aliases':sum(r['timing_status']=='MULTI_HYPOTHESIS_WITH_HISTORICAL_ALIASES' for r in interfaces),
        'r1_interval_width_ms':(r1['offset_interval_s'][1]-r1['offset_interval_s'][0])*1000,'r1_strict_fingerprint_compatible':r1['configuration_fingerprint']==m['configuration_fingerprint'],
        'depth_pose_vs_clock_physx_xy_mm':stats(np.array(pose_errors)*1000),'unchanged_p1_rgb_pose_vs_clock_physx_xy_mm':stats(np.array(rgb_errors)*1000),'rgb_vs_depth_box_marker_projection_px':stats(body_projection),
        'pixel_estimator_runtime_ms':stats([r['pose']['runtime_ms'] for r in predictions if 'runtime_ms' in r['pose']]),'probe_stats':{p:probe_stats(p) for p in ('pre_step','post_step','after_command_before_update','camera_after_annotators','after_app_update')},
        'phase_results':phase_results,'tabletop_oracle_scope':scope,'oracle_vs_independent_command_history_m':stats(command_errors),
        'binding_sha256':sha(binding_path),'manifest_sha256':sha(run/'manifest.json'),'pixel_predictions_sha256':sha(out/'pixel_predictions.jsonl'),'interface_sha256':sha(out/'time_binding_interface.jsonl'),
        'limitations':['Only the controlled kinematic tabletop sequence is qualified.','Static/repeated Cube poses do not independently reveal time.','USDRT authored transforms are not proof of the render-latched historical Fabric snapshot.','R1 interval is a frozen numeric comparison only: its fingerprint differs.','No midpoint timestamp, no Cube oracle in prediction or transfer calibration.'],
        'p2b_readiness':{'offline_fusion':'LIMITED_READY: timing/vision candidate replay only; synchronized q/dq/FK data is still required for FK-vision fusion','shadow_evaluation':'NOT_READY: original task without timing cells has not been qualified; offline shadow replay is possible','online_fusion':'NOT_READY'}}
    dump(out/'summary.json',result);return result


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--run',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);ap.add_argument('--binding',type=Path);ap.add_argument('--freeze',type=Path);a=ap.parse_args()
    print(json.dumps(evaluate(a.run,a.out,a.binding,a.freeze),indent=2))
