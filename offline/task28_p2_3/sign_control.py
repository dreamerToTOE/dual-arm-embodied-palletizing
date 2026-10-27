"""Independent bilateral first-joint sign controls from raw Isaac records.

No Jacobian, P2-1 decomposition or wrench estimate is used. For each arm
and each +/-Fx test, the direct world +Z joint-1 moment on the robot is
axis dot ((application_point - joint1_origin) cross force + pure_torque).
The held actuator effort delta must oppose this independently calculated
moment. The known load is only an independent sign-control oracle and is
never an input to the wrench estimator. Outputs are source-specific claims,
not hardware calibration certificates; a synthetic source cannot pass.
"""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
CONFIG = HERE / 'evidence/preregistration.json'
COLLECTOR = HERE / 'collector.py'


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _read(path, cfg, cfg_hash, side, case, condition):
    data = json.loads(Path(path).read_text())
    if (data.get('config_sha256') != cfg_hash or data.get('condition') != condition
            or data.get('source_kind') != 'ISAAC_TWO_ARTICULATIONS_STANDALONE'
            or data.get('case') != case):
        raise ValueError('RAW_METADATA_MISMATCH')
    rows = data['observations']
    if len(rows) != cfg['observation_window_steps'] or len(rows) < 2:
        raise ValueError('INCOMPLETE_OBSERVATION_WINDOW')
    a = [row['arms'][side] for row in rows]
    if any(r.get('source_id') != 'probe_' + side or r.get('joint_names') != cfg['joint_names'] for r in a):
        raise ValueError('ARM_SOURCE_OR_JOINT_ORDER_MISMATCH')
    def array(vals, shape, name):
        try:
            x = np.asarray(vals)
            if x.shape != shape or not np.issubdtype(x.dtype, np.number) or np.issubdtype(x.dtype, np.bool_) or np.issubdtype(x.dtype, np.complexfloating):
                raise ValueError('INVALID_' + name)
            x = x.astype(float)
            if not np.isfinite(x).all():
                raise ValueError('INVALID_' + name)
            return x
        except (TypeError, OverflowError):
            raise ValueError('INVALID_' + name) from None
    n = len(rows)
    q = array([v['q_rad'] for v in a], (n,7), 'Q')
    dq = array([v['dq_rad_s'] for v in a], (n,7), 'DQ')
    effort = array([v['effort_nm'] for v in a], (n,7), 'EFFORT')
    t = array([r['simulation_time_s'] for r in rows], (n,), 'TIMESTAMP')
    steps = array([r['physics_step'] for r in rows], (n,), 'STEPS')
    global_steps = array([r['global_collector_step'] for r in rows], (n,), 'GLOBAL_STEPS')
    if (np.any(t < 0) or np.any(np.diff(steps) != 1) or np.any(np.diff(global_steps) != 1)
            or not np.allclose(np.diff(t), cfg['physics_dt_s'], atol=1e-9, rtol=0)):
        raise ValueError('STALE_DUPLICATE_OR_OUT_OF_ORDER_OBSERVATION')
    if np.max(np.abs(dq)) > cfg['thresholds']['max_abs_velocity_rad_s']:
        raise ValueError('DYNAMIC_STATE')
    if np.max(np.ptp(q, axis=0)) > cfg['thresholds']['max_window_q_range_rad']:
        raise ValueError('UNSTABLE_Q_WINDOW')
    if np.max(np.std(effort, axis=0)) > cfg['thresholds']['max_window_effort_std_nm']:
        raise ValueError('UNSTABLE_EFFORT_WINDOW')
    o = [v['load_oracle'] for v in a]
    q_pre = array([v['q_pre_step_rad'] for v in o], (n,7), 'Q_PRE')
    p = array([v['application_point_world_m'] for v in o], (n,3), 'POINT')
    tcp = array([v['tcp_pre_step_world'] for v in o], (n,4,4), 'TCP_PRE')
    force = array([v['applied_force_world_n'] for v in o], (n,3), 'FORCE')
    torque = array([v['applied_pure_torque_world_nm'] for v in o], (n,3), 'MOMENT')
    spec = case[side] if condition == 'loaded' else {'force_world_n':[0,0,0], 'pure_torque_world_nm':[0,0,0], 'offset_from_tcp_world_m':[0,0,0]}
    if (not np.allclose(force, spec['force_world_n'], atol=1e-8, rtol=0)
            or not np.allclose(torque, spec['pure_torque_world_nm'], atol=1e-8, rtol=0)
            or not np.allclose(p, tcp[:,:3,3] + spec['offset_from_tcp_world_m'], atol=1e-8, rtol=0)):
        raise ValueError('SIGN_CONTROL_LOAD_OR_REFERENCE_POINT_MISMATCH')
    if np.max(np.abs(q_pre - q)) > cfg['thresholds']['max_pair_mean_q_difference_rad']:
        raise ValueError('PRE_POST_Q_MISMATCH')
    return {'q':q, 'dq':dq, 'effort':effort, 'p':p, 'force':force,
            'torque':torque, 'times':t, 'steps':steps, 'global_steps':global_steps,
            'sha256':sha256(path)}


def verify_sign_controls(input_dir, config_path=CONFIG):
    """Independently recompute ±Fx controls for both observed articulations.

    Each side has its own certificate results. Source and config hashes bind
    all observations to the frozen collector; check every status before using
    a side's sign. No wrenches are estimated by this function.
    """
    input_dir, config_path = Path(input_dir), Path(config_path)
    cfg = json.loads(config_path.read_text())
    cfg_hash = sha256(config_path)
    status_path = input_dir / 'collector_status.json'
    status = json.loads(status_path.read_text())
    result = {'schema':'task28.g3_p2_3.sign_control.v1', 'status':'REJECTED',
              'config_sha256':cfg_hash, 'collector_sha256':sha256(COLLECTOR),
              'collector_status_sha256':sha256(status_path), 'cases':[], 'failures':[],
              'scope':'ISAAC_STATIC_VIRTUAL_ONLY', 'execution_authorized':False,
              'method':'direct +Z joint1 axis moment, no Jacobian or wrench estimator'}
    failures = result['failures']
    if (status.get('status') != 'COLLECTION_COMPLETE_NOT_EVALUATED'
            or status.get('source_kind') != 'ISAAC_TWO_ARTICULATIONS_STANDALONE'
            or status.get('config_sha256') != cfg_hash
            or status.get('asset_sha256') != cfg['fixture_usd_sha256']
            or status.get('collector_sha256') != sha256(COLLECTOR)):
        failures.append('COLLECTOR_PROVENANCE_MISMATCH_OR_INCOMPLETE')
    runtime = status.get('runtime', {})
    if (runtime.get('observation_source') != 'SingleArticulation.get_measured_joint_efforts'
            or runtime.get('force_frame') != 'world' or runtime.get('torque_frame') != 'world'
            or runtime.get('force_application_api') != 'RigidPrim.apply_forces_and_torques_at_pos(is_global=True)'):
        failures.append('UNVERIFIED_RUNTIME_SOURCE')
    try:
        gravity = runtime['physics_gravity']
        if (abs(float(runtime['physics_dt_s']) - cfg['physics_dt_s']) > 1e-12
                or not np.allclose(np.asarray(gravity[0], float) * float(gravity[1]), cfg['gravity_m_s2'], atol=1e-6, rtol=0)):
            raise ValueError('PHYSICS_TIMING_OR_GRAVITY_MISMATCH')
    except (KeyError, TypeError, ValueError, IndexError) as exc:
        failures.append('INVALID_RUNTIME_PHYSICS:' + str(exc))
    cases = {case['id']:case for case in cfg['cases']}
    for side in ('left','right'):
        try:
            arm = runtime['arms'][side]
            names, indices = arm['all_dof_names'], arm['ordered_dof_indices']
            if (arm['articulation_path'] != '/World/Probe_' + side or arm['ordered_joint_names'] != cfg['joint_names']
                    or len(set(names)) != len(names) or len(indices) != 7 or len(set(indices)) != 7
                    or any(type(i) is not int or i < 0 or i >= len(names) or names[i] != name for i,name in zip(indices,cfg['joint_names']))
                    or not np.allclose(arm['base_world_position_m'], cfg['base_world_m'][side], rtol=0, atol=1e-6)
                    or not np.allclose(np.abs(np.dot(arm['base_world_orientation_wxyz'], cfg['base_world_orientation_wxyz'])), 1, rtol=0, atol=1e-6)):
                raise ValueError('ARTICULATION_ID_ORDER_OR_BASE_MISMATCH')
            stiffness, damping = np.asarray(arm['dof_stiffness'],float), np.asarray(arm['dof_damping'],float)
            if (stiffness.shape != (len(names),) or damping.shape != (len(names),)
                    or not np.isfinite(np.r_[stiffness,damping]).all() or np.any(stiffness[indices] <= 0)):
                raise ValueError('POSITION_HOLD_GAINS_INVALID')
        except (KeyError, TypeError, ValueError, IndexError) as exc:
            failures.append('RUNTIME_' + side + ':' + str(exc))
        for sign in ('positive','negative'):
            case_id = side + '_' + sign + '_fx'
            entry = {'side':side, 'case_id':case_id, 'status':'REJECTED', 'failures':[]}
            result['cases'].append(entry)
            try:
                case = cases[case_id]
                want_force = [1.,0.,0.] if sign == 'positive' else [-1.,0.,0.]
                if case[side]['force_world_n'] != want_force:
                    raise ValueError('CONTROL_DIRECTION_MISMATCH')
                pair = {kind:_read(input_dir / (case_id + '_' + kind + '.json'), cfg, cfg_hash, side, case, kind)
                        for kind in ('unloaded','loaded')}
                u, l = pair['unloaded'], pair['loaded']
                if u['sha256'] == l['sha256'] or not (u['global_steps'][-1] < l['global_steps'][0]):
                    raise ValueError('DUPLICATE_OR_OUT_OF_ORDER_PAIRED_TRIALS')
                if np.max(np.abs(l['q'].mean(axis=0)-u['q'].mean(axis=0))) > cfg['thresholds']['max_pair_mean_q_difference_rad']:
                    raise ValueError('PAIR_Q_MISMATCH')
                axis = np.array([0.,0.,1.]); origin = np.asarray(cfg['base_world_m'][side],float)
                moment = float(np.mean((np.cross(l['p']-origin,l['force']) + l['torque']) @ axis))
                delta = float(l['effort'][:,0].mean()-u['effort'][:,0].mean())
                d_q = float(l['q'][:,0].mean()-u['q'][:,0].mean())
                error = abs(delta + moment)
                entry.update({'axis_world':axis.tolist(), 'axis_origin_world_m':origin.tolist(),
                              'moment_on_robot_joint1_nm':moment, 'delta_effort_joint1_nm':delta,
                              'delta_q1_rad':d_q, 'equilibrium_residual_nm':error,
                              'alternative_sign_residual_nm':abs(delta-moment),
                              'input_sha256':{k:v['sha256'] for k,v in pair.items()},
                              'observation_times_s':{k:[float(v['times'][0]),float(v['times'][-1])] for k,v in pair.items()},
                              'global_step_ranges':{k:[int(v['global_steps'][0]),int(v['global_steps'][-1])] for k,v in pair.items()}})
                if (moment == 0 or delta * moment >= 0 or d_q * moment <= 0
                        or error > cfg['thresholds']['sign_joint1_residual_nm']
                        or abs(delta-moment) <= error):
                    raise ValueError('SIGN_EQUILIBRIUM_CONTROL_FAILED')
                entry['status'] = 'PASS'
            except (OSError, KeyError, ValueError, TypeError, IndexError) as exc:
                entry['failures'].append(str(exc))
    if not failures and len(result['cases']) == 4 and all(e['status']=='PASS' for e in result['cases']):
        for side in ('left','right'):
            moments = [e['moment_on_robot_joint1_nm'] for e in result['cases'] if e['side'] == side]
            if moments[0] * moments[1] >= 0:
                failures.append('OPPOSITE_MOMENTS_NOT_VERIFIED:' + side)
        if not failures:
            result['status'] = 'SIGN_CONTROL_PASS'
    return result
