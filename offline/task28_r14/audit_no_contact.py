"""Postcollection strict observer reuse, never a contact or feedback certificate."""
from __future__ import annotations
import collections
import json
from pathlib import Path
import numpy as np
from offline.task28_g1_1.task28_validator.model import RobotModel
from offline.task28_p2_4.observer import ObserverContract, Sample, step
from offline.task28_p2_4_r3.evaluate import evaluate
from .no_contact import HERE, R3, guard_command, sha, write


def observer_sample(row, pre, cfg, model, source_hash):
    names = tuple(cfg['expected_all_dof_names']); arm = names[:7]
    dt = cfg['physics_dt_s']; t = row['sim_time_s']
    q = np.asarray(row['q_rad_or_m']); dq = np.asarray(row['dq_rad_s_or_m_s'])
    # Same numerical/time/conditioning contract as P2-4 evaluate.py; no relaxation.
    contract = ObserverContract(names, arm, 'world', 'left_side_tcp', dt+1e-8,
        dt+1e-8, dt-1e-6, dt+1e-6, 1e-10, 1e6,
        'SOURCE_UNVALIDATED_CANDIDATE', source_hash,
        ('revolute',)*7+('prismatic',)*2, .1)
    provenance = {'q':'Isaac.get_joint_positions','dq':'Isaac.get_joint_velocities',
        'effort':'R14 guarded set_joint_efforts command over preceding interval; command provenance only',
        'mass':'Isaac.get_mass_matrices raw 9x9', 'coriolis':'Isaac.get_coriolis_and_centrifugal_forces',
        'gravity':'Isaac.get_generalized_gravity_forces', 'jacobian':'frozen G1 RobotModel left-side mathematical TCP at post q'}
    sample = Sample(names, arm, q, dq, row['commanded_effort'], row['mass_matrix'],
        row['coriolis_compensation'], row['gravity_compensation'], model.jacobian('left',q[:7]),
        q.copy(), dq.copy(), q[:7].copy(), t, t, pre['cursor'][1], t, t, t,
        str(row['physics_step_index']), provenance, 'world', 'left_side_tcp',
        ('revolute',)*7+('prismatic',)*2,
        ('rad,rad/s,rad/s2,N*m',)*7+('m,m/s,m/s2,N',)*2, 'effort_i/acceleration_j')
    return sample, contract


def audit(folder):
    cfg = json.loads((R3/'experiment_preregistration.json').read_text())
    p = json.loads((folder/'preregistration.json').read_text())
    life = json.loads((folder/'lifecycle.json').read_text())
    scope = json.loads((folder/'runtime_scope.json').read_text())
    original = evaluate(folder/'raw')
    rows = json.loads((folder/'raw/zero_load_hold_observations.json').read_text())['samples']
    pre = [json.loads(x) for x in (folder/'precommand.jsonl').read_text().splitlines()]
    reasons = []
    registration_hash = sha(folder/'preregistration.json')
    if scope.get('plan_sha256') != registration_hash:
        reasons.append('PLAN_HASH_BINDING')
    if p != __import__('offline.task28_r14.no_contact', fromlist=['plan']).plan():
        reasons.append('PLAN_OR_CURRENT_SOURCE_CHANGED')
    if life.get('source_hashes') != p.get('sources'):
        reasons.append('LIFECYCLE_SOURCE_BINDING')
    launch_root = folder/'source_at_launch'
    if not launch_root.is_dir():
        reasons.append('SOURCE_SNAPSHOT_MISSING')
    else:
        for rel, expected in p.get('sources', {}).items():
            snap = launch_root/rel
            if not snap.is_file() or sha(snap) != expected:
                reasons.append('SOURCE_SNAPSHOT:'+rel)
    if not original['gate_pass']: reasons += original['reasons']
    for key in ('clean_exit','isolation_verified','sources_unchanged','portable_resources_removed'):
        if not life.get(key): reasons.append('PROCESS_'+key.upper())
    asset = Path(cfg['asset'])
    raw_status = json.loads((folder/'raw/collector_status.json').read_text())
    if (not scope.get('ros_bridge_disabled') or not asset.is_file()
            or sha(asset) != cfg['fixture_usd_sha256']
            or raw_status.get('asset_sha256') != cfg['fixture_usd_sha256']):
        reasons.append('RUNTIME_SCOPE')
    if len(rows)!=240 or len(pre)!=240 or scope.get('guarded_commands')!=240:
        reasons.append('INCOMPLETE_RECORD')
    model = RobotModel(); history = None; records = []
    for i,(row,before) in enumerate(zip(rows,pre)):
        rec = {'step':row['step'],'time_s':row['sim_time_s'],'reasons':[]}
        try:
            if row['physics_step_index'] != before['cursor'][0]+1 or abs(row['sim_time_s']-before['cursor'][1]-cfg['physics_dt_s'])>1e-6:
                raise ValueError('PRE_POST_CURSOR')
            if not np.array_equal(row['commanded_effort'], before['command']): raise ValueError('PRE_POST_COMMAND')
            if np.max(abs(np.asarray(row['applied_effort_readback'])-before['command']))>1e-6:
                raise ValueError('APPLIED_COMMAND')
            guard_command(before['command'],before['q'],before['dq'],before['g'],
                          pre[i-1]['command'] if i else None,pre[i-1]['dq'] if i else None,cfg,p)
            s,c = observer_sample(row,before,cfg,model,sha(folder/'preregistration.json'))
            result = step(s,c,history); history = result.history
            rec['status'] = result.status
            if result.status not in ('WARMUP','CANDIDATE_ESTIMATE'): rec['reasons'].append(result.status)
            if result.wrench:
                rec.update(external_arm_torque_Nm=result.arm_external_torque_on_robot_Nm,
                           wrench=result.wrench.wrench_minimum_norm,rank=result.wrench.rank,
                           condition=result.wrench.condition_number,torque_fit_residual_Nm=result.wrench.reconstruction_residual_Nm)
        except (ValueError,KeyError,TypeError,np.linalg.LinAlgError) as exc:
            rec.update(status='REJECTED');rec['reasons'].append(str(exc));history=None
        if rec['reasons']: reasons += rec['reasons']
        records.append(rec)
    usable = [r for r in records if 'wrench' in r]
    q=np.array([r['q_rad_or_m'] for r in rows]);dq=np.array([r['dq_rad_s_or_m_s'] for r in rows])
    u=np.array([r['commanded_effort'] for r in rows]); applied=np.array([r['applied_effort_readback'] for r in rows])
    metrics = {
        'max_arm_speed_rad_s':float(abs(dq[:,:7]).max()),
        'max_arm_position_error_rad':float(abs(q[:,:7]-np.array(cfg['configuration_rad'])).max()),
        'max_applied_command_difference':float(abs(applied-u).max()),
        'max_effort_per_coordinate':abs(u).max(axis=0).tolist(),
        'max_later_effort_rate_per_coordinate_s':(abs(np.diff(u,axis=0))/cfg['physics_dt_s']).max(axis=0).tolist(),
        'max_source_read_duration_s':max(r['source_read_duration_s'] for r in rows),
        'max_raw_mass_asymmetry':max(float(abs(np.array(r['mass_matrix'])-np.array(r['mass_matrix']).T).max()) for r in rows),
        'saturated_frames':sum(bool(r['saturated']) for r in rows),
        'max_zero_load_joint_residual_Nm':max((max(abs(x) for x in r['external_arm_torque_Nm']) for r in usable),default=None),
        'max_zero_load_force_N':max((float(np.linalg.norm(r['wrench'][:3])) for r in usable),default=None),
        'max_zero_load_moment_Nm':max((float(np.linalg.norm(r['wrench'][3:])) for r in usable),default=None),
        'min_rank':min((r['rank'] for r in usable),default=None),
        'max_condition':max((r['condition'] for r in usable),default=None),
    }
    # Original R3 known-load numerical thresholds; zero load is one subset only.
    for name,limit in [('max_zero_load_joint_residual_Nm',.2),('max_zero_load_force_N',.5),('max_zero_load_moment_Nm',.2)]:
        if metrics[name] is None or metrics[name]>limit: reasons.append('ZERO_LOAD_'+name.upper())
    if len(usable)!=239: reasons.append('OBSERVER_CANDIDATE_COUNT')
    result = {'schema':'TASK28_R14_NO_CONTACT_AUDIT_V1',
        'status':'NO_CONTACT_SINGLE_POSE_CANDIDATE_PASS' if not reasons else 'NO_CONTACT_CHECK_REJECTED',
        'reasons':sorted(set(reasons)), 'legacy_stability_evaluation':original,
        'observer_status_counts':dict(collections.Counter(r['status'] for r in records)),
        'metrics':metrics, 'records':records,
        'zero_load_numerical_thresholds_source':'P2-4-R3 online_evaluate.py: 0.2 Nm / 0.5 N / 0.2 Nm; no edited thresholds',
        'strict_observer_contract_source':'P2-4 evaluate.py, command candidate source explicitly distinguished from projected effort',
        'source_status':'SOURCE_UNVALIDATED_CANDIDATE',
        'newton_euler_residual':'M_post*dq_backward+c_post+g_post-u_interval; no learned bias or sign fitting',
        'first_bootstrap_rate_exemption':'inherited P2-4-R3 initial full gravity; not current R8 original-scene admission',
        'fresh_simulation_runs':1,'no_contact_pose_only':True,'external_load_tested':False,
        'contact_force_observation_validated':False,'closed_loop_force_control':False,
        'fk_at_physical_tcp_independently_checked':False,'hardware_validated':False,
        'limitations':['One static isolated bare-FR3 pose, no mounted L-tool/bilateral/contact qualification',
            'Postcollection causal observer; not within control deadline and not force feedback',
            'Read-duration gate is not independent acquisition timestamps for each API',
            'No contact sensors recorded; zero-load scene/no external force calls establish scope, not certified collision envelope',
            'Jacobian is frozen mathematical left-side TCP; no physical contact location verification'],
        'source_hashes': {str(p.resolve().relative_to(HERE)):sha(p) for p in [folder/'preregistration.json',folder/'raw/zero_load_hold_observations.json',folder/'precommand.jsonl',folder/'runtime_scope.json',folder/'lifecycle.json']}}
    write(folder/'evaluation.json',result)
    print(json.dumps({k:result[k] for k in ('status','reasons','observer_status_counts','metrics')}))
    return result


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--folder', type=Path, default=None)
    args = parser.parse_args()
    folder = args.folder or sorted((HERE/'evidence/no_contact').glob('attempt_*'))[-1]
    audit(folder)
