#!/usr/bin/env python3
"""Replay archived observations, then independently evaluate applied loads.

No simulation is launched. Estimator receives observation-only ArmPair inputs;
application force/point/moment never enter the replay call. Sign controls are
independent historical calibration and are recomputed separately. Outputs are
mathematical virtual wrenches only; there is no common object in the scene.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from offline.task28_g1_1.task28_validator.closed_chain import nominal_grasps
from offline.task28_g1_1.task28_validator.model import RobotModel
from offline.task28_g1_1.task28_validator.se3 import transform, residual, checked_transform
from offline.task28_p2_2_r1.adapter import SOURCE, SEMANTIC, SignCalibration, StaticLimits
from offline.task28_p2_3.integration import ArmPair, IntegrationContract
from offline.task28_p2_3.evaluate import (CONFIG, EXPECTED_CASES, EXPECTED_CONFIG_SHA256,
    _runtime, _live_check, _window, _finite, _jsonable, _json_bytes)
from offline.task28_p2_3.sign_control import sha256, verify_sign_controls
from offline.task28_p2_3_r1.replay import replay_virtual_pair
from offline.task28_p2_3_r1.oracle import applied_oracle, independent_kkt, direct_measured_center

ARCHIVE = ROOT / 'offline/task28_p2_3'
RAW = ARCHIVE / 'evidence/formal_raw'
MANIFEST = ARCHIVE / 'evidence/artifact_manifest.json'
SIGN = ARCHIVE / 'evidence/sign_result.json'
BOUND_SOURCES = (
    'fixtures/task28_g1_0_new_single_cube_v0/fixture_manifest.json',
    'offline/task28_g1_1/tests/control_states.json',
    'offline/task28_g1_1/task28_validator/model.py',
    'offline/task28_g1_1/task28_validator/fixture_binding.json',
    'offline/task28_g1_1/task28_validator/se3.py',
    'offline/task28_g1_1/task28_validator/closed_chain.py',
    'offline/task28_p2_1/grasp.py',
    'offline/task28_p2_2/estimator.py',
    'offline/task28_p2_2_r1/adapter.py',
    'offline/task28_p2_3/integration.py',
    'offline/task28_p2_3/evaluate.py',
    'offline/task28_p2_3/sign_control.py',
    'offline/task28_p2_3_r1/replay.py',
    'offline/task28_p2_3_r1/evaluate_replay.py',
    'offline/task28_p2_3_r1/oracle.py',
)


def archive_integrity():
    """Verify the entire committed original task manifest, including all raw files."""
    manifest = json.loads(MANIFEST.read_text())
    failures = []
    for entry in manifest['files']:
        path = ARCHIVE / entry['path']
        if not path.is_file() or sha256(path) != entry['sha256'] or path.stat().st_size != entry['bytes']:
            failures.append(entry['path'])
    if failures:
        raise ValueError('ARCHIVE_MANIFEST_MISMATCH:' + ','.join(failures))
    return {'manifest_sha256': sha256(MANIFEST), 'files_verified': len(manifest['files']),
            'input_hashes': {entry['path']: entry['sha256'] for entry in manifest['files']}}


def observation_only(path, cfg, cfg_hash, case_id, kind, model, runtime):
    """Extract just measured q/dq/effort/live TCP and strict observation metadata.

    The JSON file also physically stores case loads; discard `case` and each
    `load_oracle` prior to validation. The estimator never reads those values.
    """
    raw = json.loads(path.read_text())
    stored_case_id = raw.pop('case')['id']
    for row in raw.get('observations', []):
        for side in ('left', 'right'):
            row['arms'][side].pop('load_oracle', None)
    if (stored_case_id != case_id or raw.get('config_sha256') != cfg_hash or
        raw.get('condition') != kind or raw.get('source_kind') != 'ISAAC_TWO_ARTICULATIONS_STANDALONE'):
        raise ValueError('RAW_METADATA_MISMATCH')
    rows = raw['observations']; n = cfg['observation_window_steps']
    if len(rows) != n or n < 2:
        raise ValueError('INCOMPLETE_OBSERVATION_WINDOW')
    for key in ('physics_step', 'global_collector_step'):
        if any(type(row[key]) is not int for row in rows):
            raise ValueError('STEP_ID_NOT_INTEGER')
    pre = [raw['prechecks'][s]['physics_step'] for s in ('left', 'right')]
    if pre[0] != pre[1] or type(pre[0]) is not int:
        raise ValueError('INVALID_PRECHECK_STEP')
    expected_last = pre[0] + cfg['settle_steps_each_condition']
    if rows[0]['physics_step'] != expected_last-n+1 or rows[-1]['physics_step'] != expected_last:
        raise ValueError('WINDOW_NOT_PREREGISTERED_SETTLED_TAIL')
    times = _finite([row['simulation_time_s'] for row in rows], (n,), 'INVALID_TIMESTAMP')
    steps = np.array([row['physics_step'] for row in rows])
    global_steps = np.array([row['global_collector_step'] for row in rows])
    if (np.any(times < 0) or np.any(np.diff(steps) != 1) or np.any(np.diff(global_steps) != 1)
        or not np.allclose(np.diff(times), cfg['physics_dt_s'], atol=1e-9, rtol=0)):
        raise ValueError('STALE_DUPLICATE_OR_OUT_OF_ORDER_OBSERVATION')
    data, live, fk = {}, {}, {}
    for side in ('left', 'right'):
        arms = [row['arms'][side] for row in rows]
        if any(a.get('source_id') != 'probe_' + side or a.get('joint_names') != cfg['joint_names'] for a in arms):
            raise ValueError('ARM_SOURCE_OR_JOINT_ORDER_MISMATCH:' + side)
        d = {'q': _finite([a['q_rad'] for a in arms], (n,7), 'INVALID_Q'),
             'dq': _finite([a['dq_rad_s'] for a in arms], (n,7), 'INVALID_DQ'),
             'effort': _finite([a['effort_nm'] for a in arms], (n,7), 'INVALID_EFFORT'),
             'times': times, 'steps': steps, 'global_steps': global_steps}
        threshold = cfg['thresholds']
        if np.max(np.abs(d['dq'])) > threshold['max_abs_velocity_rad_s']:
            raise ValueError('DYNAMIC_STATE:' + side)
        if np.max(np.ptp(d['q'], axis=0)) > threshold['max_window_q_range_rad']:
            raise ValueError('UNSTABLE_Q_WINDOW:' + side)
        if np.max(np.std(d['effort'], axis=0)) > threshold['max_window_effort_std_nm']:
            raise ValueError('UNSTABLE_EFFORT_WINDOW:' + side)
        live[side], fk[side] = _live_check(raw, side, model, runtime, cfg)
        data[side] = d
    return data, live, fk


def check_oracle_provenance(raw, case, cfg):
    """Validate archived known-load records only after the estimate is complete.

    The applied point and pre-step pose precede the post-step q/effort/pose by
    one physics step. They are bounded and disclosed, not called simultaneous.
    """
    result = {}
    for side in ('left','right'):
        worst_q = worst_p = worst_r = 0.
        for row in raw['observations']:
            arm = row['arms'][side]; o = arm['load_oracle']; spec = case[side]
            qpre = _finite(o['q_pre_step_rad'],(7,),'INVALID_ORACLE_Q_PRE')
            q = _finite(arm['q_rad'],(7,),'INVALID_Q')
            pre = checked_transform(o['tcp_pre_step_world']); post = checked_transform(arm['tcp_world'])
            point = _finite(o['application_point_world_m'],(3,),'INVALID_ORACLE_POINT')
            force = _finite(o['applied_force_world_n'],(3,),'INVALID_ORACLE_FORCE')
            torque = _finite(o['applied_pure_torque_world_nm'],(3,),'INVALID_ORACLE_TORQUE')
            if (not np.allclose(force,spec['force_world_n'],atol=1e-8,rtol=0)
                or not np.allclose(torque,spec['pure_torque_world_nm'],atol=1e-8,rtol=0)
                or not np.allclose(point,pre[:3,3]+spec['offset_from_tcp_world_m'],atol=1e-8,rtol=0)):
                raise ValueError('APPLIED_LOAD_OR_POINT_MISMATCH:'+side)
            worst_q = max(worst_q,float(np.max(np.abs(qpre-q))))
            p,r = residual(pre,post); worst_p=max(worst_p,p); worst_r=max(worst_r,r)
        if worst_q > cfg['thresholds']['max_pair_mean_q_difference_rad']:
            raise ValueError('PRE_POST_Q_MISMATCH:'+side)
        if (worst_p > cfg['integration_contract']['max_fk_translation_m'] or
            worst_r > cfg['integration_contract']['max_fk_rotation_rad']):
            raise ValueError('PRE_POST_TCP_MISMATCH:'+side)
        result[side] = {'max_q_inf_rad':worst_q,'max_tcp_translation_m':worst_p,
            'max_tcp_rotation_rad':worst_r,'timing':'load point pre-step; observation post-step'}
    return result


def evaluate():
    result = {'schema':'task28.g3_p2_3_r1.virtual_replay.v1', 'status':'BLOCKED',
              'scope':'ARCHIVED_VIRTUAL_WRENCH_ONLY', 'original_g3_p2_3_status':'BLOCKED',
              'execution_authorized':False, 'physical_robot_tested':False,
              'wrist_ft_sensor_required':False, 'dual_arm_force_control_implemented':False,
              'production_bridge_modified':False, 'physical_contact_measured':False,
              'unloaded_loaded_physically_synchronous':False, 'cases':[], 'global_failures':[]}
    try:
        result['archive_integrity'] = archive_integrity()
        result['source_sha256'] = {name:sha256(ROOT/name) for name in BOUND_SOURCES}
        cfg = json.loads(CONFIG.read_text()); cfg_hash = sha256(CONFIG)
        if cfg_hash != EXPECTED_CONFIG_SHA256 or tuple(c['id'] for c in cfg['cases']) != EXPECTED_CASES:
            raise ValueError('FORMAL_PREREGISTRATION_HASH_OR_MATRIX_MISMATCH')
        runtime = _runtime(json.loads((RAW/'collector_status.json').read_text()), cfg, cfg_hash)
        sign = verify_sign_controls(RAW, CONFIG)
        if sign != json.loads(SIGN.read_text()) or sign['status'] != 'SIGN_CONTROL_PASS':
            raise ValueError('INDEPENDENT_BILATERAL_SIGN_CALIBRATION_FAILED')
        sign_hash = sha256(SIGN)
        result['sign_calibration_sha256'] = sign_hash
        result['sign_calibration_status'] = sign['status']
    except (OSError, KeyError, TypeError, ValueError, IndexError) as exc:
        result['global_failures'].append(type(exc).__name__ + ':' + str(exc))
        return result
    th = cfg['thresholds']; ic = cfg['integration_contract']
    limits = StaticLimits(cfg['observation_window_steps']*cfg['physics_dt_s'], th['max_abs_velocity_rad_s'],
        th['max_window_q_range_rad'], th['max_window_effort_std_nm'], th['max_pair_mean_q_difference_rad'])
    contract = IntegrationContract('/World/Probe_left','/World/Probe_right','Cube_01',
        'left_fr3_side_suction_tcp','right_fr3_side_suction_tcp','ISAAC_TWO_ARTICULATIONS_STANDALONE',
        ic['max_sync_s'],ic['max_fk_translation_m'],ic['max_fk_rotation_rad'],
        ic['max_grasp_translation_m'],ic['max_grasp_rotation_rad'],ic['min_jacobian_singular_value'])
    obj = transform(Rotation.from_euler('z',cfg['state']['object_yaw_rad']).as_matrix(),cfg['state']['object_position_m'])
    grasps = nominal_grasps()
    nominal = {s:obj @ getattr(grasps,s) for s in ('left','right')}
    model = RobotModel()
    for case in cfg['cases']:
        cid = case['id']; out = {'id':cid, 'status':'REJECTED', 'failures':[]}
        result['cases'].append(out)
        try:
            paths = {k: RAW/(cid+'_'+k+'.json') for k in ('unloaded','loaded')}
            if sha256(paths['unloaded']) == sha256(paths['loaded']):
                raise ValueError('DUPLICATE_TRIAL_EVIDENCE')
            parsed = {k:observation_only(paths[k],cfg,cfg_hash,cid,k,model,runtime) for k in paths}
            if parsed['unloaded'][0]['left']['global_steps'][-1] >= parsed['loaded'][0]['left']['global_steps'][0]:
                raise ValueError('RESET_TRIAL_ORDER_INVALID')
            arms = []
            for side in ('left','right'):
                identity = '/World/Probe_'+side
                u, l = parsed['unloaded'][0][side], parsed['loaded'][0][side]
                arms.append(ArmPair(side,identity,_window(u),_window(l),cid+'_unloaded',cid+'_loaded',
                    tuple(map(int,u['steps'])),tuple(map(int,l['steps'])),parsed['loaded'][1][side][-1],
                    SignCalibration(SOURCE,SEMANTIC,True,sign_hash),identity,True,
                    'archived independent bilateral joint1 controls sha256:'+sign_hash,
                    'observed Isaac link7 pose times frozen link7-to-TCP'))
            # Estimation is completed before any case-specific oracle is accessed.
            estimated = replay_virtual_pair(*arms,obj,contract,limits)
            # Algebraic weighted-split validation uses estimated contacts, not
            # known applied loads. Independently assemble the matrix and KKT.
            kkt = independent_kkt(estimated['local_contacts_on_cube'],obj[:3,3],nominal)
            kkt_external_error = float(np.linalg.norm(np.asarray(estimated['weighted_external_local'])-kkt['weighted_external_local']))
            kkt_internal_error = float(np.linalg.norm(np.asarray(estimated['weighted_internal_local'])-kkt['weighted_internal_local']))
            kkt_net_error = float(np.linalg.norm(np.asarray(estimated['net_cube_wrench_world_center'])-kkt['net_world_center']))
            if max(kkt_external_error,kkt_internal_error,kkt_net_error,kkt['null_residual']) > th['null_space_residual']:
                out['failures'].append('INDEPENDENT_KKT_MISMATCH')
            direct_net = direct_measured_center(estimated,obj[:3,3])
            reference_error = float(np.linalg.norm(np.asarray(estimated['net_cube_wrench_world_center'])-direct_net))
            if reference_error > th['null_space_residual']:
                out['failures'].append('INDEPENDENT_MOMENT_REFERENCE_MISMATCH')
            loaded_raw = json.loads(paths['loaded'].read_text())
            pre_post = check_oracle_provenance(loaded_raw,case,cfg)
            oracle = applied_oracle(loaded_raw,obj[:3,3],nominal)
            per_arm = {}
            for side in ('left','right'):
                known = oracle['per_arm'][side]['on_robot_world_tcp']
                calc = estimated[side]
                w = np.asarray(calc['on_robot_world_tcp'])
                q = parsed['loaded'][0][side]['q'].mean(axis=0)
                tau = model.jacobian(side,q).T @ known
                observed = np.asarray(calc['adapter']['external_generalized_torque_on_robot_Nm'])
                te = float(np.max(np.abs(observed-tau)))
                fe = float(np.linalg.norm(w[:3]-known[:3]))
                me = float(np.linalg.norm(w[3:]-known[3:]))
                per_arm[side] = {'oracle_on_robot_tcp':known,'estimated_on_robot_tcp':w,
                    'oracle_joint_torque_nm':tau,'estimated_joint_torque_nm':observed,
                    'torque_inf_error_nm':te,'force_l2_error_n':fe,'moment_l2_error_nm':me}
                if te > th['max_abs_torque_error_nm']: out['failures'].append('TORQUE_ORACLE_MISMATCH:'+side)
                if fe > th['max_force_error_n']: out['failures'].append('FORCE_ORACLE_MISMATCH:'+side)
                if me > th['max_moment_error_nm']: out['failures'].append('MOMENT_ORACLE_MISMATCH:'+side)
                if np.linalg.norm(known) == 0 and np.max(np.abs(observed)) > th['zero_delta_tau_inf_nm']:
                    out['failures'].append('ZERO_CHANNEL_DELTA_TORQUE:'+side)
            d = np.asarray(estimated['net_cube_wrench_world_center']) - oracle['net_world_center']
            fe, me = float(np.linalg.norm(d[:3])), float(np.linalg.norm(d[3:]))
            if fe > th['max_object_wrench_force_error_n']: out['failures'].append('OBJECT_FORCE_MISMATCH')
            if me > th['max_object_wrench_moment_error_nm']: out['failures'].append('OBJECT_MOMENT_MISMATCH')
            if estimated['grasp_null_residual'] > th['null_space_residual']: out['failures'].append('GRASP_NULL_RESIDUAL')
            if estimated['decomposition_reconstruction_residual'] > th['null_space_residual']:
                out['failures'].append('RECONSTRUCTION_RESIDUAL')
            if oracle['net_direct_vs_kkt_residual'] > th['null_space_residual']:
                out['failures'].append('INDEPENDENT_MOMENT_SHIFT_MISMATCH')
            out.update({'input_sha256':{k:sha256(v) for k,v in paths.items()},
                'window_global_step_ranges':{k:[int(parsed[k][0]['left']['global_steps'][0]),
                    int(parsed[k][0]['left']['global_steps'][-1])] for k in paths},
                'live_fk':{side+'_'+k:parsed[k][2][side] for k in paths for side in ('left','right')},
                'estimated':estimated, 'per_arm_errors':per_arm, 'oracle':oracle, 'pre_post_comparison':pre_post,
                'independent_kkt_estimated_contacts':kkt, 'kkt_external_l2_error':kkt_external_error,
                'kkt_internal_l2_error':kkt_internal_error,'kkt_net_l2_error':kkt_net_error,
                'independent_direct_estimate_world_center':direct_net,'independent_reference_l2_error':reference_error,
                'object_force_l2_error_n':fe,'object_moment_l2_error_nm':me,
                'weighted_external_l2_error_mixed_si':float(np.linalg.norm(np.asarray(estimated['weighted_external_local'])-oracle['weighted_external_local'])),
                'weighted_internal_l2_error_mixed_si':float(np.linalg.norm(np.asarray(estimated['weighted_internal_local'])-oracle['weighted_internal_local']))})
            if not out['failures']: out['status']='PASS'
        except (OSError,KeyError,TypeError,ValueError,IndexError,np.linalg.LinAlgError) as exc:
            out['failures'].append(type(exc).__name__+':'+str(exc))
    passes = sum(c['status']=='PASS' for c in result['cases'])
    result['status'] = ('G3_P2_3_R1_VIRTUAL_REPLAY_PASS' if passes==len(EXPECTED_CASES)
                        else 'PARTIAL' if passes else 'BLOCKED')
    return _jsonable(result)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True,help='new JSON file; must not exist')
    args=parser.parse_args()
    result=evaluate()
    with args.output.open('xb') as handle: handle.write(_json_bytes(result))
    return 0 if result['status']=='G3_P2_3_R1_VIRTUAL_REPLAY_PASS' else 1


if __name__=='__main__': raise SystemExit(main())
