#!/usr/bin/env python3
"""Offline evaluation of frozen bilateral raw trials; never launches Isaac.

The static estimator receives observations only. Known externally applied loads
are used *after* estimation for an independent Cartesian wrench comparison.
Each reset is a distinct trial; the two arms are sampled together within one
trial but the two trials are not physically synchronous.
"""
from __future__ import annotations
import argparse
from dataclasses import asdict, is_dataclass
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from offline.task28_g1_1.task28_validator.model import RobotModel
from offline.task28_g1_1.task28_validator.se3 import transform, checked_transform, residual
from offline.task28_p2_2_r1.adapter import SOURCE, SEMANTIC, SignCalibration, StaticLimits, StaticWindow
from offline.task28_p2_3.integration import ArmPair, IntegrationContract, JOINT_NAMES, integrate_virtual_pair
from offline.task28_p2_3.sign_control import verify_sign_controls, _read, sha256

CONFIG = ROOT / 'offline/task28_p2_3/evidence/preregistration.json'
R1 = ROOT / 'offline/task28_p2_2_r1/evidence/sign_raw/collector_status.json'
COLLECTOR = ROOT / 'offline/task28_p2_3/collector.py'
EXPECTED_CONFIG_SHA256 = '255989c31bd86074291290ede4ffe008fd98e05113e6ea3abb87ec0c4cfaf689'
EXPECTED_CASES = ('zero','left_positive_fx','left_negative_fx','right_positive_fx',
                  'right_negative_fx','opposing_contact_line','same_direction',
                  'opposing_offset_forces','asymmetric')


def _jsonable(x):
    if is_dataclass(x): return _jsonable(asdict(x))
    if isinstance(x, np.ndarray): return x.tolist()
    if isinstance(x, np.generic): return x.item()
    if isinstance(x, dict): return {k:_jsonable(v) for k,v in x.items()}
    if isinstance(x, (tuple,list)): return [_jsonable(v) for v in x]
    return x


def _json_bytes(x):
    return (json.dumps(_jsonable(x), sort_keys=True, indent=2, allow_nan=False)+'\n').encode()


def _finite(value, shape, reason):
    x = np.asarray(value)
    if (x.shape != shape or not np.issubdtype(x.dtype, np.number)
            or np.issubdtype(x.dtype, np.bool_) or np.issubdtype(x.dtype, np.complexfloating)):
        raise ValueError(reason)
    x = x.astype(float)
    if not np.isfinite(x).all(): raise ValueError(reason)
    return x


def _runtime(status, cfg, cfg_hash):
    if (status.get('status') != 'COLLECTION_COMPLETE_NOT_EVALUATED'
            or status.get('source_kind') != 'ISAAC_TWO_ARTICULATIONS_STANDALONE'
            or status.get('config_sha256') != cfg_hash
            or status.get('collector_sha256') != sha256(COLLECTOR)
            or status.get('asset_sha256') != cfg['fixture_usd_sha256']
            or status.get('g1_source_state_sha256') != sha256(ROOT / 'offline/task28_g1_1/tests/control_states.json')):
        raise ValueError('COLLECTOR_STATUS_OR_PROVENANCE_MISMATCH')
    expected_files = [f"{c['id']}_{kind}.json" for c in cfg['cases'] for kind in ('unloaded','loaded')]
    if status.get('completed_condition_files') != expected_files:
        raise ValueError('COLLECTOR_FILE_LIST_MISMATCH')
    if not (0 < status.get('actual_physics_steps', -1) <= cfg['max_steps']):
        raise ValueError('INVALID_PHYSICS_STEP_BUDGET')
    r = status['runtime']; old = json.loads(R1.read_text())['runtime']
    if (r['observation_source'] != 'SingleArticulation.get_measured_joint_efforts'
            or r['force_application_api'] != old['force_application_api']
            or r['force_frame'] != 'world' or r['torque_frame'] != 'world'
            or abs(float(r['physics_dt_s'])-float(old['physics_dt_s'])) > 1e-12
            or not np.allclose(r['physics_gravity'][0], old['physics_gravity'][0], atol=1e-8, rtol=0)
            or abs(float(r['physics_gravity'][1])-float(old['physics_gravity'][1])) > 1e-6):
        raise ValueError('R1_SOURCE_OR_PHYSICS_MISMATCH')
    for side in ('left','right'):
        a = r['arms'][side]
        for key in ('all_dof_names','ordered_joint_names','ordered_dof_indices',
                    'dof_stiffness','dof_damping','asset_physx_articulation_properties'):
            if a.get(key) != old.get(key): raise ValueError('R1_ARM_RUNTIME_MISMATCH:' + side + ':' + key)
        if a['articulation_path'] != '/World/Probe_' + side or a['link7_path'] != '/World/Probe_' + side + '/fr3_link7':
            raise ValueError('WRONG_ARM_IDENTITY:' + side)
        if not np.allclose(a['base_world_position_m'], cfg['base_world_m'][side], atol=1e-6, rtol=0):
            raise ValueError('BASE_POSITION_MISMATCH:' + side)
        quat = _finite(a['base_world_orientation_wxyz'], (4,), 'BASE_ORIENTATION_INVALID')
        if not np.isclose(abs(quat @ np.asarray(cfg['base_world_orientation_wxyz'])),1., atol=1e-6, rtol=0):
            raise ValueError('BASE_ORIENTATION_MISMATCH:' + side)
    return r


def _live_check(raw, side, model, runtime, cfg):
    """Derive TCP from measured link7 pose, not from the model's own FK."""
    arm = runtime['arms'][side]
    q0 = np.asarray(cfg['state'][side + '_q'])
    frame0 = model._chain(side, q0)[0][side + '_fr3_link7']
    offset = np.linalg.inv(frame0) @ model.fk(side,q0)
    declared = checked_transform(arm['link7_to_frozen_virtual_tcp'])
    if not np.allclose(offset, declared, rtol=0, atol=1e-10):
        raise ValueError('FROZEN_LINK7_TCP_OFFSET_MISMATCH:' + side)
    worst_p = worst_r = 0.
    tcps = []
    for row in raw['observations']:
        a = row['arms'][side]
        pos = _finite(a['link7_position_world_m'],(3,),'INVALID_LINK_POSITION')
        quat = _finite(a['link7_orientation_world_wxyz'],(4,),'INVALID_LINK_QUATERNION')
        if abs(np.linalg.norm(quat)-1.) > 1e-5: raise ValueError('INVALID_LINK_QUATERNION')
        live = transform(Rotation.from_quat(quat[[1,2,3,0]]).as_matrix(),pos)
        q = _finite(a['q_rad'],(7,),'INVALID_Q')
        if not model.within_limits(q): raise ValueError('JOINT_LIMIT_VIOLATION:' + side)
        expected = model._chain(side,q)[0][side+'_fr3_link7']
        pe,re = residual(live,expected)
        worst_p,worst_r = max(worst_p,pe),max(worst_r,re)
        tcp = live @ offset
        logged = checked_transform(a['tcp_world'])
        if not np.allclose(tcp,logged,atol=1e-9,rtol=0): raise ValueError('LOGGED_TCP_MISMATCH:' + side)
        if (pe > cfg['thresholds']['max_link7_fk_position_difference_m']
                or re > cfg['thresholds']['max_link7_fk_rotation_difference_rad']):
            raise ValueError('LIVE_LINK7_FK_MISMATCH:' + side)
        tcps.append(tcp)
    return tcps, {'max_translation_m':worst_p,'max_rotation_rad':worst_r}


def _load_raw(path, cfg, cfg_hash, side, case, condition, model, runtime):
    raw = json.loads(path.read_text())
    # _read independently enforces step order, fresh windows, q, effort and load logs.
    data = _read(path,cfg,cfg_hash,side,case,condition)
    rows = raw['observations']; n = cfg['observation_window_steps']
    if any(type(row['physics_step']) is not int or type(row['global_collector_step']) is not int for row in rows):
        raise ValueError('STEP_ID_NOT_INTEGER')
    precheck_steps = [raw['prechecks'][s]['physics_step'] for s in ('left','right')]
    if precheck_steps[0] != precheck_steps[1] or type(precheck_steps[0]) is not int:
        raise ValueError('INVALID_PRECHECK_STEP')
    expected_last = precheck_steps[0] + cfg['settle_steps_each_condition']
    if rows[0]['physics_step'] != expected_last-n+1 or rows[-1]['physics_step'] != expected_last:
        raise ValueError('WINDOW_NOT_PREREGISTERED_SETTLED_TAIL')
    tcps, fk = _live_check(raw,side,model,runtime,cfg)
    return raw, data, tcps, fk


def _window(data):
    t = data['times']
    return StaticWindow(data['q'],data['dq'],data['effort'],t,float(t[-1]),JOINT_NAMES,SOURCE)


def _oracle(raw, side, center):
    """Independent action/reaction and center moments, from recorded applied loads."""
    forces=[]; moments=[]; tcp_moments=[]
    for row in raw['observations']:
        v=row['arms'][side]['load_oracle']
        f=np.asarray(v['applied_force_world_n'],float)
        m=np.asarray(v['applied_pure_torque_world_nm'],float)
        p=np.asarray(v['application_point_world_m'],float)
        tcp=np.asarray(v['tcp_pre_step_world'],float)[:3,3]
        forces.append(f); tcp_moments.append(m+np.cross(p-tcp,f))
        moments.append(-(m+np.cross(p-center,f)))
    return np.r_[np.mean(forces,axis=0),np.mean(tcp_moments,axis=0)], np.r_[-np.mean(forces,axis=0),np.mean(moments,axis=0)]



def _independent_weighted_oracle(raw, object_pose, nominal):
    """Direct mechanics and KKT solve, independent of P2-1 matrix/decomposition.

    Uses registered nominal transforms and P2-1's documented default metric;
    does not consume estimates, measured efforts, or GraspModel methods.
    """
    center = object_pose[:3,3]
    axes = object_pose[:3,:3]
    blocks=[]; local=[]
    for side in ('left','right'):
        t=getattr(nominal,side)
        rotate=axes@t[:3,:3]
        r=axes@t[:3,3]
        nominal_point=center+r
        w=[]
        for row in raw['observations']:
            v=row['arms'][side]['load_oracle']
            f=-np.asarray(v['applied_force_world_n'],float)
            moment=-np.asarray(v['applied_pure_torque_world_nm'],float)
            p=np.asarray(v['application_point_world_m'],float)
            w.append(np.r_[rotate.T@f, rotate.T@(moment+np.cross(p-nominal_point,f))])
        local.extend(np.mean(w,axis=0))
        x,y,z=r
        cross=np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
        blocks.append(np.block([[rotate,np.zeros((3,3))],[cross@rotate,rotate]]))
    G=np.hstack(blocks); x=np.asarray(local)
    # W uses force scale 1 N and torque scale 0.061 N m, repeated twice.
    W=np.diag(np.tile([1.,1.,1.,1/.061**2,1/.061**2,1/.061**2],2))
    # Direct constrained KKT solve: W ext - G.T lambda=0; G ext=G x.
    kkt=np.block([[W,-G.T],[G,np.zeros((6,6))]])
    solved=np.linalg.solve(kkt,np.r_[np.zeros(12),G@x])
    external=solved[:12]; internal=x-external
    return {'local_on_cube':x,'net_world_center':G@x,'weighted_external_local':external,
            'weighted_internal_local':internal,'null_residual':float(np.linalg.norm(G@internal)),
            'metric_diagonal':np.diag(W),'method':'independent direct 18x18 KKT on hand-constructed cross-product grasp map'}


def evaluate(input_dir, config_path=CONFIG):
    input_dir,config_path=Path(input_dir),Path(config_path)
    cfg=json.loads(config_path.read_text()); cfg_hash=sha256(config_path)
    result={'schema':'task28.g3_p2_3.evaluation.v1','status':'INCOMPLETE',
            'config_sha256':cfg_hash,'evaluator_sha256':sha256(Path(__file__)),
            'source_kind':'ISAAC_TWO_ARTICULATIONS_STANDALONE','scope':'VIRTUAL_CONTACT_STATIC',
            'execution_authorized':False,'physical_contact_measured':False,
            'unloaded_loaded_physically_synchronous':False,'flags':cfg['flags'],
            'global_failures':[],'cases':[]}
    fail=result['global_failures']
    if cfg_hash != EXPECTED_CONFIG_SHA256 or tuple(c.get('id') for c in cfg.get('cases',[])) != EXPECTED_CASES:
        fail.append('FORMAL_PREREGISTRATION_HASH_OR_MATRIX_MISMATCH')
    try:
        status_path=input_dir/'collector_status.json'
        status=json.loads(status_path.read_text())
        result['collector_status_sha256']=sha256(status_path)
        runtime=_runtime(status,cfg,cfg_hash)
    except (OSError,KeyError,TypeError,ValueError,IndexError) as exc:
        fail.append('GLOBAL_RUNTIME:'+str(exc)); runtime=None
    try:
        sign=verify_sign_controls(input_dir,config_path)
        result['sign_control']=sign
        sign_hash=hashlib.sha256(_json_bytes(sign)).hexdigest()
        result['sign_control_sha256']=sign_hash
        if sign['status'] != 'SIGN_CONTROL_PASS': fail.append('INDEPENDENT_BILATERAL_SIGN_CONTROL_FAILED')
    except (OSError,KeyError,TypeError,ValueError,IndexError) as exc:
        fail.append('SIGN_CONTROL_UNAVAILABLE:'+str(exc)); sign_hash=None
    for case in cfg['cases']:
        out={'id':case['id'],'status':'INCOMPLETE','failures':[]}
        result['cases'].append(out)
        if fail: out['failures'].append('GLOBAL_PREREQUISITES_FAILED'); continue
        try:
            model=RobotModel(); records={}; data={}; live={}; fk={}
            for kind in ('unloaded','loaded'):
                path=input_dir/(case['id']+'_'+kind+'.json')
                for side in ('left','right'):
                    records[kind],data[side,kind],live[side,kind],fk[side,kind]=_load_raw(path,cfg,cfg_hash,side,case,kind,model,runtime)
            if sha256(input_dir/(case['id']+'_unloaded.json')) == sha256(input_dir/(case['id']+'_loaded.json')):
                raise ValueError('DUPLICATE_TRIAL_EVIDENCE')
            if data['left','unloaded']['global_steps'][-1] >= data['left','loaded']['global_steps'][0]:
                raise ValueError('RESET_TRIAL_ORDER_INVALID')
            th=cfg['thresholds']; ic=cfg['integration_contract']
            limits=StaticLimits(cfg['observation_window_steps']*cfg['physics_dt_s'], th['max_abs_velocity_rad_s'],
                                th['max_window_q_range_rad'],th['max_window_effort_std_nm'],th['max_pair_mean_q_difference_rad'])
            arms=[]
            for side in ('left','right'):
                identity='/World/Probe_'+side
                arms.append(ArmPair(side,identity,_window(data[side,'unloaded']),_window(data[side,'loaded']),
                    case['id']+'_unloaded',case['id']+'_loaded',
                    tuple(map(int,data[side,'unloaded']['steps'])),tuple(map(int,data[side,'loaded']['steps'])),
                    live[side,'loaded'][-1],SignCalibration(SOURCE,SEMANTIC,True,sign_hash),identity,True,
                    'independent direct joint1 controls; sha256:'+sign_hash,'Isaac observed link7 pose times frozen link7-to-TCP'))
            contract=IntegrationContract('/World/Probe_left','/World/Probe_right','Cube_01',
                'left_fr3_side_suction_tcp','right_fr3_side_suction_tcp','ISAAC_TWO_ARTICULATIONS_STANDALONE',
                ic['max_sync_s'],ic['max_fk_translation_m'],ic['max_fk_rotation_rad'],
                ic['max_grasp_translation_m'],ic['max_grasp_rotation_rad'],ic['min_jacobian_singular_value'])
            obj=transform(Rotation.from_euler('z',cfg['state']['object_yaw_rad']).as_matrix(),cfg['state']['object_position_m'])
            from offline.task28_g1_1.task28_validator.closed_chain import nominal_grasps
            nominal=nominal_grasps()
            grasp_residual={}
            for side in ('left','right'):
                qmean=data[side,'loaded']['q'].mean(axis=0)
                p,r=residual(model.fk(side,qmean),obj @ getattr(nominal,side))
                grasp_residual[side]={'translation_m':p,'rotation_rad':r,
                    'pair_q_inf_rad':float(np.max(np.abs(qmean-data[side,'unloaded']['q'].mean(axis=0)) ))}
            out.update({'input_sha256':{k:sha256(input_dir/(case['id']+'_'+k+'.json')) for k in ('unloaded','loaded')},
                'live_fk':{s+'_'+k:v for (s,k),v in fk.items()},'pre_gate_grasp_residuals':grasp_residual,
                'window_global_step_ranges':{k:[int(data['left',k]['global_steps'][0]),int(data['left',k]['global_steps'][-1])] for k in ('unloaded','loaded')}})
            for side, gr in grasp_residual.items():
                if gr['translation_m'] > ic['max_grasp_translation_m']:
                    out['failures'].append('FROZEN_GRASP_TRANSLATION_MISMATCH:'+side)
                if gr['rotation_rad'] > ic['max_grasp_rotation_rad']:
                    out['failures'].append('FROZEN_GRASP_ROTATION_MISMATCH:'+side)
            if out['failures']:
                out['status']='REJECTED'
                out['estimation_status']='NOT_COMPUTED_FROZEN_GRASP_GATE_FAILED'
                continue
            # The known applied loads are deliberately absent from this call.
            estimated=integrate_virtual_pair(*arms,obj,contract,limits)
            errors={}; object_oracle=np.zeros(6)
            for side in ('left','right'):
                oracle,center_oracle=_oracle(records['loaded'],side,obj[:3,3]); object_oracle+=center_oracle
                calc=estimated[side]; w=np.asarray(calc['on_robot_world_tcp'])
                q=data[side,'loaded']['q'].mean(axis=0)
                tau_true=model.jacobian(side,q).T@oracle
                tau_observed=np.asarray(calc['adapter']['external_generalized_torque_on_robot_Nm'])
                tau_error=float(np.max(np.abs(tau_observed-tau_true)))
                force_error=float(np.linalg.norm(w[:3]-oracle[:3])); moment_error=float(np.linalg.norm(w[3:]-oracle[3:]))
                errors[side]={'known_on_robot_tcp_wrench':oracle,'estimated_on_robot_tcp_wrench':w,
                    'estimated_external_joint_torque_nm':tau_observed,'oracle_joint_torque_nm':tau_true,
                    'torque_inf_error_nm':tau_error,'force_l2_error_n':force_error,'moment_l2_error_nm':moment_error}
                if tau_error > th['max_abs_torque_error_nm']: out['failures'].append('TORQUE_ORACLE_MISMATCH:'+side)
                if force_error > th['max_force_error_n']: out['failures'].append('FORCE_ORACLE_MISMATCH:'+side)
                if moment_error > th['max_moment_error_nm']: out['failures'].append('MOMENT_ORACLE_MISMATCH:'+side)
                if np.linalg.norm(oracle) == 0 and np.max(np.abs(tau_observed)) > th['zero_delta_tau_inf_nm']:
                    out['failures'].append('ZERO_CHANNEL_DELTA_TORQUE:'+side)
            net=np.asarray(estimated['net_cube_wrench_world_center']); d=net-object_oracle
            weighted_oracle=_independent_weighted_oracle(records['loaded'],obj,nominal)
            ext_error=np.asarray(estimated['weighted_external_local'])-weighted_oracle['weighted_external_local']
            int_error=np.asarray(estimated['weighted_internal_local'])-weighted_oracle['weighted_internal_local']
            out['weighted_oracle']=weighted_oracle
            out['weighted_external_l2_error']=float(np.linalg.norm(ext_error))
            out['weighted_internal_l2_error']=float(np.linalg.norm(int_error))
            out['weighted_comparison_semantics']='descriptive mixed SI coordinates; no unregistered post-hoc threshold'
            object_force_error=float(np.linalg.norm(d[:3])); object_moment_error=float(np.linalg.norm(d[3:]))
            if object_force_error > th['max_object_wrench_force_error_n']: out['failures'].append('OBJECT_FORCE_MISMATCH')
            if object_moment_error > th['max_object_wrench_moment_error_nm']: out['failures'].append('OBJECT_MOMENT_MISMATCH')
            if estimated['grasp_null_residual'] > th['null_space_residual']: out['failures'].append('GRASP_NULL_RESIDUAL')
            out.update({'input_sha256':{k:sha256(input_dir/(case['id']+'_'+k+'.json')) for k in ('unloaded','loaded')},
                'per_arm':errors,'object_oracle_world_center':object_oracle,
                'object_force_l2_error_n':object_force_error,'object_moment_l2_error_nm':object_moment_error,
                'estimated':estimated,'live_fk':{s+'_'+k:v for (s,k),v in fk.items()},
                'window_global_step_ranges':{k:[int(data['left',k]['global_steps'][0]),int(data['left',k]['global_steps'][-1])] for k in ('unloaded','loaded')}})
            out['status']='REJECTED' if out['failures'] else 'PASS'
        except (OSError,KeyError,TypeError,ValueError,IndexError,np.linalg.LinAlgError) as exc:
            out['status']='REJECTED'; out['failures'].append(type(exc).__name__+':'+str(exc))
    if not fail and all(x['status']=='PASS' for x in result['cases']): result['status']='G3_P2_3_BILATERAL_STATIC_VIRTUAL_PASS'
    elif not fail: result['status']='PARTIAL'
    return _jsonable(result)


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--input',type=Path,required=True); parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(); args.output.mkdir(parents=False,exist_ok=False)
    result=evaluate(args.input)
    if 'sign_control' in result: (args.output/'sign_control.json').write_bytes(_json_bytes(result['sign_control']))
    (args.output/'evaluation.json').write_bytes(_json_bytes(result))
    return 0 if result['status']=='G3_P2_3_BILATERAL_STATIC_VIRTUAL_PASS' else 1

if __name__=='__main__': raise SystemExit(main())
