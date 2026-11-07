#!/usr/bin/env python3
"""Pixel/observable replay first, isolated TCP oracle scoring second. No simulation required."""
import argparse, collections, hashlib, json, sys
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[3]; sys.path.insert(0, str(ROOT))
R3 = ROOT/'research/stage_b_multisensor_r3'
from offline.task28_g1_1.task28_validator.model import RobotModel
from research.stage_b_multisensor_r3.history_contract import SynchronizedHistory, IDENTITY
from research.stage_b_multisensor_r3.time_alignment import RobotHistory
from research.stage_b_multisensor_r3.r1_time_code import decode_clock
from research.stage_b_multisensor_r3.r2_scene_binding import estimate_cube_pixels
from research.stage_b_vision_contact_r2b.visual.aruco_pose import detect_aruco


def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def rows(p): return [json.loads(s) for s in Path(p).read_text().splitlines()]
def dump(p, value): Path(p).write_text(json.dumps(value, sort_keys=True, indent=2, allow_nan=False)+'\n')
def save_rows(p, value): Path(p).write_text(''.join(json.dumps(r, sort_keys=True, allow_nan=False)+'\n' for r in value))
def stats(values): return dict(zip(('median', 'p95', 'max'), map(float, np.percentile(values, [50, 95, 100])))) if len(values) else None


def transform_error(a, b):
    a, b = np.array(a), np.array(b)
    return float(np.linalg.norm(a[:3, 3]-b[:3, 3])), float(Rotation.from_matrix(a[:3, :3]@b[:3, :3].T).magnitude())


def replay(run, out):
    out.mkdir(parents=True, exist_ok=False)
    manifest = json.loads((run/'manifest.json').read_text()); cfg = json.loads((run/'config_snapshot.json').read_text())
    assert sha(run/'config_snapshot.json') == manifest['frozen_eval_config_sha256'] == sha(R3/'config/p2b0_frozen.json')
    assert sha(R3/'r2_scene_binding.py') == manifest['frozen_pixel_estimator_sha256']
    for name, digest in manifest['data_sha256'].items():
        if not name.startswith('oracle'): assert sha(run/name) == digest
    measured = rows(run/'robot_history.jsonl'); physics = rows(run/'physics_history.jsonl'); observations = rows(run/'observations.jsonl')
    times = {r['physics_step']: r['time_s'] for r in physics}; model = RobotModel()
    assert all(r['physics_step'] in times and abs(times[r['physics_step']]-r['time_s']) < 1e-9 for r in measured)
    history = SynchronizedHistory(measured, model.fk, cfg)
    key = {k: manifest[k] for k in IDENTITY}
    states = [history.at(key, r['time_s']) for r in measured]
    assert all(s['status'] == 'ALIGNED' for s in states)
    for s, r in zip(states, measured): s['physics_step'] = r['physics_step']
    save_rows(out/'robot_states.jsonl', states)
    K = np.array(manifest['K']); M = np.array(manifest['world_from_camera']); cropped_K = K.copy(); cropped_K[:2, 2] -= manifest['capture_roi'][:2]
    p1 = json.loads((R3/'config/p1_frozen.json').read_text())
    old_binding = json.loads((R3/'config/r2_scene_binding_frozen.json').read_text())
    visual = []; associations = []; controls = []; paired = []; gaps = []; previous = None
    for r in observations:
        assert sha(run/r['rgb']) == r['rgb_sha256'] and sha(run/r['depth']) == r['depth_sha256']
        image = cv2.imread(str(run/r['rgb'])); depth = np.load(run/r['depth'], allow_pickle=False)['distance_to_image_plane']
        clock = decode_clock(image, depth, manifest['clock_layout']); pair_ok = all(v['paired'] for v in clock.values())
        if r['mode'] == 'pattern':
            if r['pattern_settled']: controls.append(pair_ok and all(v['rgb_code'] == r['expected_pattern'] for v in clock.values()))
            continue
        pose = estimate_cube_pixels(image, depth, K, M, manifest['capture_roi'], p1)
        pose.pop('runtime_ms', None)  # Runtime is not part of deterministic replay content.
        pose.get('rgb_pose', {}).pop('runtime_ms', None)
        detection = detect_aruco(image, cropped_K, np.zeros(5), p1['marker_size_m'], 23)
        association = history.associate(r, times, manifest['physics_epoch_start_s'])
        v = {k: r[k] for k in (*IDENTITY, 'frame_id', 'rgb', 'depth', 'rgb_sha256', 'depth_sha256', 'sdk_time_s', 'fabric_sim_time_s', 'reference_time', 'dispatcher_reference', 'callback_entry_step', 'callback_core_time_s', 'callback_entry_wall_s', 'callback_wall_s', 'arrival_physics_time_s', 'arrival_wall_s', 'writer_start_wall_s', 'writer_complete_wall_s')}
        v.update(clock=clock, pair_ok=pair_ok, pose=pose, aruco_status=detection.status,
                 aruco_confidence=detection.confidence, confidence_meaning='existing detector heuristic, NOT calibrated attachment or pose probability',
                 measurement_valid=pair_ok and pose['status'] == 'PRESENT', rejection_reason=None if pair_ok and pose['status'] == 'PRESENT' else ('rgb_depth_pair_failed' if not pair_ok else pose.get('reason','depth_fit_uncertain')),
                 measurement_time_s=None, timing_status=association['timing_status'], online_fusion_eligible=False)
        v['historical_r2_clock_prior'] = {'scope':'REFERENCE_ONLY_NEW_CAPTURE_FINGERPRINT_MISMATCH_NOT_A_BOUND',
            'binding_sha256':sha(R3/'config/r2_scene_binding_frozen.json'),
            'candidate_steps':[clock['post']['rgb_code']+shift for shift in old_binding['cube_minus_clock_candidate_ticks']] if pair_ok else [],
            'constrains_admissible_interval':False}
        if previous is not None:
            gaps.append(r['sdk_time_s']-previous)
        previous = r['sdk_time_s']; paired.append(pair_ok); visual.append(v); associations.append(association)
    # These artifacts are complete before any oracle is opened.
    save_rows(out/'visual_history.jsonl', visual); save_rows(out/'associations.jsonl', associations)
    dump(out/'observable_prediction_manifest.json', {'schema':'r3.p2b0.observable_predictions.v1',
        'source_manifest_sha256':sha(run/'manifest.json'), 'config_sha256':sha(run/'config_snapshot.json'),
        'source_sha256':{str(p.relative_to(ROOT)):sha(p) for p in [Path(__file__), R3/'history_contract.py', R3/'time_alignment.py', R3/'r2_scene_binding.py', R3/'r1_time_code.py', R3/'planar_pose_estimator.py', R3/'config/p1_frozen.json', R3/'config/r2_scene_binding_frozen.json', ROOT/'research/stage_b_vision_contact_r2b/visual/aruco_pose.py', ROOT/'offline/task28_g1_1/task28_validator/model.py', model.fixture_dir/'fixture_manifest.json']},
        'files':{n:sha(out/n) for n in ['robot_states.jsonl','visual_history.jsonl','associations.jsonl']},
        'oracle_inputs':[], 'state_reference':'robot_state_ref is physics_step in robot_states.jsonl, scoped by run_id/epoch_id/time_base_id',
        'continuous_candidates':'Full closed admissible interval is authoritative. Physics grid is a normalized index, not the entire uncertainty set. at(identity,t) exposes q/dq/FK at any supported real-valued candidate.'})
    # Hold out actual odd 60-Hz samples; interpolate from even 30-Hz samples.
    sparse = SynchronizedHistory(measured[::2], model.fk, cfg, expected_step_stride=2)
    baseline = RobotHistory(model.fk,max_samples=cfg['history_max_samples'],max_bracket_s=cfg['max_interpolation_bracket_s'])
    for r in measured[::2]:
        baseline.append(**{k:r[k] for k in ('time_s','physics_step','q','dq','suction_observation','suction_command','controller_command')})
    interpolation = []; unchecked = []
    for i in range(1, len(measured)-1, 2):
        r = measured[i]; prediction = sparse.at(key, r['time_s'])
        record = {'physics_step':r['physics_step'], 'status':prediction['status']}
        if prediction['status'] != 'ALIGNED': record['reason'] = prediction['reason']
        if prediction['status'] == 'ALIGNED':
            record.update(q_error_rad=max(float(np.max(abs(np.array(prediction['q'][s])-r['q'][s]))) for s in ('left','right')),
                          dq_error_rad_s=max(float(np.max(abs(np.array(prediction['dq'][s])-r['dq'][s]))) for s in ('left','right')),
                          tcp_fk=prediction['tcp_fk'])
        interpolation.append(record)
        prediction = baseline.at(r['time_s'])
        unchecked.append({'physics_step':r['physics_step'],'status':prediction['status'],
            'q_error_rad':max(float(np.max(abs(np.array(prediction['q'][s])-r['q'][s]))) for s in ('left','right')),
            'dq_error_rad_s':max(float(np.max(abs(np.array(prediction['dq'][s])-r['dq'][s]))) for s in ('left','right')),
            'tcp_fk':prediction['tcp_fk']})
    # Evaluation boundary: simulator references below never enter visual/history predictions.
    assert sha(run/'oracle_tcp.jsonl') == manifest['data_sha256']['oracle_tcp.jsonl']
    truth = {r['physics_step']:r for r in rows(run/'oracle_tcp.jsonl')}
    fk_errors = []
    for s in states:
        for side in ('left','right'):
            p, a = transform_error(s['tcp_fk'][side], truth[s['physics_step']]['tcp_physx'][side])
            fk_errors.append({'physics_step':s['physics_step'],'side':side,'position_error_m':p,'rotation_error_rad':a})
    for r in interpolation+unchecked:
        if r['status'] == 'ALIGNED':
            errors = [transform_error(r['tcp_fk'][s], truth[r['physics_step']]['tcp_physx'][s]) for s in ('left','right')]
            r.update(tcp_error_m=max(e[0] for e in errors), rotation_error_rad=max(e[1] for e in errors))
            del r['tcp_fk']
    save_rows(out/'evaluation_fk.jsonl', fk_errors); save_rows(out/'evaluation_interpolation.jsonl', interpolation)
    save_rows(out/'evaluation_unchecked_linear_baseline.jsonl',unchecked)
    # Simulated sensor dropout deletes no robot samples and makes no Cube-state inference.
    dropout_start = visual[len(visual)//3]['sdk_time_s']; dropout_end = dropout_start+1.
    hidden = [v for v in visual if dropout_start <= v['sdk_time_s'] <= dropout_end]
    retained = [r for r in measured if dropout_start <= r['time_s'] <= dropout_end]
    callback_lag = [v['arrival_physics_time_s']-v['callback_core_time_s'] for v in visual]
    good_i = [r for r in interpolation if r['status'] == 'ALIGNED']
    summary = {'scope':cfg['scope'], 'run_id':run.name, 'decision':'PENDING_CHECKS',
        'sample_continuity':{'samples':len(measured),'first_step':measured[0]['physics_step'],'last_step':measured[-1]['physics_step'],
            'missing_steps':sum(max(0,b['physics_step']-a['physics_step']-1) for a,b in zip(measured, measured[1:])),
            'dt_s':stats(np.diff([r['time_s'] for r in measured])),
            'core_vs_physics_max_s':max(abs(r['core_sim_time_s']-r['time_s']) for r in measured),
            'timeline_minus_physics_s':stats([r['timeline_s']-r['time_s'] for r in measured]),
            'q_peak_to_peak_rad':{s:np.ptp([r['q'][s] for r in measured],axis=0).tolist() for s in ('left','right')}},
        'vision':{'frames':len(visual),'paired_rgb_depth':sum(paired),'settled_pattern_controls':len(controls),'settled_pattern_pass':sum(controls),
            'aruco_present':sum(v['aruco_status']=='PRESENT' for v in visual),'pose_status_counts':dict(collections.Counter(v['pose']['status'] for v in visual)),
            'sdk_fabric_max_difference_s':max(abs(v['sdk_time_s']-v['fabric_sim_time_s']) for v in visual),
            'sample_interval_s':stats(gaps),'callback_to_reader_sim_s':stats(callback_lag),
            'writer_duration_wall_s':stats([v['writer_complete_wall_s']-v['writer_start_wall_s'] for v in visual])},
        'association':{'status_counts':dict(collections.Counter(a['status'] for a in associations)),
            'uncertain_frames':sum(a['measurement_time_s'] is None for a in associations),
            'candidate_grid_total':sum(len(a.get('physics_grid_candidates',[])) for a in associations),
            'candidate_grid_supported':sum(a.get('supported_grid_candidates',0) for a in associations),
            'frames_with_some_robot_support':sum(a.get('supported_grid_candidates',0)>0 for a in associations),
            'frames_with_full_robot_support':sum(a.get('unsupported_grid_candidates',1)==0 for a in associations),
            'causal_interval_width_s':stats([a['admissible_interval_s'][1]-a['admissible_interval_s'][0] for a in associations if 'admissible_interval_s' in a]),
            'unsupported_prehistory_end_s':manifest['robot_history_start_s']},
        'fk':{'arm_samples':len(fk_errors),'position_error_mm':stats([r['position_error_m']*1000 for r in fk_errors]),'rotation_error_rad':stats([r['rotation_error_rad'] for r in fk_errors])},
        'interpolation':{'heldout_actual_samples':len(interpolation),'accepted':len(good_i),'rejected':len(interpolation)-len(good_i),
            'rejection_reasons':dict(collections.Counter(r['reason'] for r in interpolation if r['status']!='ALIGNED')),
            'q_error_rad':stats([r['q_error_rad'] for r in good_i]),'dq_error_rad_s':stats([r['dq_error_rad_s'] for r in good_i]),
            'tcp_error_mm':stats([r['tcp_error_m']*1000 for r in good_i]),'rotation_error_rad':stats([r['rotation_error_rad'] for r in good_i])},
        'unchecked_linear_baseline':{'scope':'Failure comparator only, old RobotHistory without command-boundary rejection; not deployable interpolation guarantee',
            'samples':len(unchecked),'q_error_rad':stats([r['q_error_rad'] for r in unchecked]),
            'dq_error_rad_s':stats([r['dq_error_rad_s'] for r in unchecked]),'tcp_error_mm':stats([r['tcp_error_m']*1000 for r in unchecked])},
        'injected_dropout':{'source':'offline test only, not a naturally observed dropout','interval_s':[dropout_start,dropout_end], 'hidden_visual_frames':len(hidden),'retained_robot_samples':len(retained),'robot_step_gaps':sum(b['physics_step']-a['physics_step']!=1 for a,b in zip(retained,retained[1:]))},
        'unavailable_channels':cfg['unavailable_channels'], 'online_fusion_eligible':False}
    checks = {'continuous_measured_history':summary['sample_continuity']['missing_steps']==0,
        'compatible_same_run_clocks':summary['sample_continuity']['core_vs_physics_max_s']<1e-6 and summary['vision']['sdk_fabric_max_difference_s']<1e-6,
        'fk_position':max(r['position_error_m'] for r in fk_errors)<=cfg['fk_position_tolerance_m'],
        'fk_rotation':max(r['rotation_error_rad'] for r in fk_errors)<=cfg['fk_rotation_tolerance_rad'],
        'interpolation':len(good_i)==len(interpolation) and all(r['q_error_rad']<=cfg['interpolation_q_tolerance_rad'] and r['dq_error_rad_s']<=cfg['interpolation_dq_tolerance_rad_s'] and r['tcp_error_m']<=cfg['interpolation_tcp_tolerance_m'] and r['rotation_error_rad']<=cfg['interpolation_rotation_tolerance_rad'] for r in good_i),
        'rgb_depth_consistency':all(paired) and bool(controls) and all(controls),
        'honest_ambiguity':all(a['status']=='UNCERTAIN' and a['measurement_time_s'] is None for a in associations),
        'dropout_robot_continuity':bool(retained) and summary['injected_dropout']['robot_step_gaps']==0}
    summary.update(checks=checks, decision='PASS' if all(checks.values()) else 'PARTIAL')
    dump(out/'summary.json',summary)
    dump(out/'replay_hashes.json',{n:sha(out/n) for n in ['robot_states.jsonl','visual_history.jsonl','associations.jsonl','observable_prediction_manifest.json','evaluation_fk.jsonl','evaluation_interpolation.jsonl','evaluation_unchecked_linear_baseline.jsonl','summary.json']})
    return summary


if __name__ == '__main__':
    ap=argparse.ArgumentParser(); ap.add_argument('--run',type=Path,required=True); ap.add_argument('--out',type=Path,required=True); a=ap.parse_args()
    print(json.dumps(replay(a.run,a.out),indent=2))
