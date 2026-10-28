"""CPU-only reproducible audit of all 75 QP-5 and 2 QP-5-R1 moving frames."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from offline.task28_g1_1.task28_validator.model import RobotModel
from offline.task28_p5_qp_4.reference import load_qp3_positive, source_hashes
from offline.task28_p5_qp_5_r2.calculations import array, angle, closure_diagnostics, sample_original, validate_row
from offline.task28_p5_qp_5_r2.controls import negative_controls

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads((ROOT/path).read_text())


def verify_sources(root=ROOT):
    binding = json.loads((HERE/'source_binding.json').read_text())
    for path, expected in binding['files'].items():
        if not (root/path).is_file():
            raise ValueError('MISSING_SOURCE:'+path)
        if sha(root/path) != expected:
            raise ValueError('SOURCE_HASH_MISMATCH:'+path)
    source_hashes()  # also verifies QP-4 and every frozen fixture file
    return binding


def reference_diagnostics(steps, model):
    intervals, jumps = [], []
    previous = np.zeros(14)
    for s in steps:
        q0, q1, dq = (np.asarray(s[k]) for k in ('q_initial','q_next','qdot14'))
        dt = s['dt_s']
        obj0, obj1 = np.asarray(s['object_pose_initial']), np.asarray(s['desired_object_pose_next'])
        dp = (obj1[:3,3]-obj0[:3,3])*(1/120)/dt
        intervals.append(dict(index=s['step_index'], start_s=s['time_s'], end_s=s['target_time_s'], q_start=q0.tolist(), q_end=q1.tolist(),
                              qdot=dq.tolist(), euler_error_inf_rad=float(np.max(np.abs(q1-q0-dt*dq))),
                              q_increment_per_physics_step_rad=(dq/120).tolist(), q_increment_inf_rad=float(np.max(np.abs(dq/120))),
                              object_increment_per_physics_step_m=dp.tolist(), object_increment_norm_m=float(np.linalg.norm(dp)),
                              object_rotation_per_physics_step_rad=angle(obj0[:3,:3].T@obj1[:3,:3])/dt/120))
        jumps.append(dict(time_s=s['time_s'], delta_qdot_rad_s=(dq-previous).tolist(), inf_rad_s=float(np.max(np.abs(dq-previous))),
                          smeared_over_20ms_diagnostic_inf_rad_s2=float(np.max(np.abs(dq-previous))/.02)))
        previous = dq
    jumps.append(dict(time_s=.1, delta_qdot_rad_s=(-previous).tolist(), inf_rad_s=float(np.max(np.abs(previous))),
                      smeared_over_20ms_diagnostic_inf_rad_s2=float(np.max(np.abs(previous))/.02)))
    times = sorted(set([i/120 for i in range(13)]+[0.,.02,.04,.06,.08,.1]))
    samples=[]
    for t in times:
        q,dq,obj=sample_original(steps,t)
        samples.append(dict(time_s=t,q14=q.tolist(),dq14=dq.tolist(),object_pose=obj.tolist(),
                            closure=closure_diagnostics(obj,model.fk('left',q[:7]),model.fk('right',q[7:]))))
    return dict(identity='task28_qp3_seed28_transport40_5step', derived_identity=None, duration_s=.1,
                acceleration_status='UNAVAILABLE_NO_VALID_TIME_PARAMETERIZATION',
                acceleration_note='jump/dt is hypothetical 20ms smearing, not the derivative at a discontinuity or a physical bound',
                intervals=intervals, velocity_jumps=jumps, sampled_reference_geometry=samples,
                max_reference_translation_m=max(max(x['closure']['metrics'][k] for k in ['relative_translation_m','left_absolute_translation_m','right_absolute_translation_m']) for x in samples))


def scan_archive(name, steps, model):
    r1 = name.endswith('_r1')
    base='offline/'+name
    c=read(base+'/preregistration.json')
    rows=read(base+'/evidence/formal_run_v1/tracking_observations.json')['observations']
    static=read(base+'/evidence/formal_run_v1/static_hold_observations.json')['observations']
    prior_cmd={s:np.asarray(static[-1]['arms'][s]['commanded_effort']) for s in ('left','right')}
    kp,kd=np.asarray(c['pd_kp']),np.asarray(c['pd_kd'])
    result=[]
    for i,row in enumerate(rows):
        if i and (row['sim_time_s'] <= rows[i-1]['sim_time_s'] or row['physics_step_index'] != rows[i-1]['physics_step_index']+1):
            raise ValueError('ARCHIVE_TIME_OR_SEQUENCE_DISCONTINUITY')
        qref,dqref,obj,cq,cdq=validate_row(row,steps,c,r1=r1)
        t=float(row['sim_time_s'])
        arms={}
        for side,sl in [('left',slice(0,7)),('right',slice(7,14))]:
            a=row['arms'][side]
            q,dq=array(a['q_rad'],(9,)),array(a['dq_rad_s'],(9,))
            pre=row['pre_step']['states'][side] if r1 else a
            pq,pdq=array(pre['q_rad'],(9,)),array(pre['dq_rad_s'],(9,))
            qr,dr=np.r_[cq[sl],c['finger_q_m']],np.r_[cdq[sl],[0.,0.]]
            pterm,dterm=kp*(qr-pq),kd*(dr-pdq)
            raw,cmd,app,g=[array(a[k],(9,)) for k in ('raw_effort','commanded_effort','applied_effort','gravity_pre')]
            # R1 serializes raw-P(arm only)-D, so its finger field includes P.
            # Do not silently interpret this mislabeled field as a raw gravity read.
            inferred_g=raw-pterm-dterm
            bounded=np.clip(raw,-np.asarray(c['effort_bounds']),np.asarray(c['effort_bounds']))
            limited=np.clip(bounded,prior_cmd[side]-np.asarray(c['effort_rate_bounds_per_step']),prior_cmd[side]+np.asarray(c['effort_rate_bounds_per_step']))
            if not np.allclose(cmd,limited,rtol=0,atol=1e-12) or bool(np.any(bounded!=raw))!=a['saturated'] or bool(np.any(limited!=bounded))!=a['rate_limited']:
                raise ValueError('EFFORT_ACTIVITY_RECONSTRUCTION_MISMATCH')
            model_tcp=model.fk(side,q[:7]); tcp=np.asarray(a['tcp_world'])
            link=np.asarray(a['link7_world']); model_link=model._chain(side,q[:7])[0][side+'_fr3_link7']
            arms[side]=dict(dof_names=a['dof_names'], units=['rad','rad','rad','rad','rad','rad','rad','m','m'],
                effort_units=['Nm']*7+['N']*2, actual_q=q.tolist(),actual_dq=dq.tolist(),
                matched_q_ref=qref[sl].tolist(),matched_dq_ref=dqref[sl].tolist(),q_error_rad=(q[:7]-qref[sl]).tolist(),dq_error_rad_s=(dq[:7]-dqref[sl]).tolist(),
                command_pre_q=pq.tolist(),command_pre_dq=pdq.tolist(),command_q_ref=qr.tolist(),command_dq_ref=dr.tolist(),
                actual_tcp=tcp.tolist(),model_tcp=model_tcp.tolist(),
                model_vs_tcp_translation_m=float(np.linalg.norm(tcp[:3,3]-model_tcp[:3,3])),model_vs_tcp_rotation_rad=angle(tcp[:3,:3].T@model_tcp[:3,:3]),
                link7_fk_translation_m=float(np.linalg.norm(link[:3,3]-model_link[:3,3])),
                raw_effort=raw.tolist(),commanded_effort=cmd.tolist(),applied_readback=app.tolist(),
                gravity_field_archived=g.tolist(),gravity_inferred_from_raw_PD=inferred_g.tolist(),P_feedback=pterm.tolist(),D_feedback=dterm.tolist(),
                gravity_field_minus_inferred=(g-inferred_g).tolist(),
                raw_reconstruction_error_arm_Nm=float(np.max(np.abs((g+pterm+dterm-raw)[:7]))),
                configured_effort_margin=(np.asarray(c['effort_bounds'])-np.abs(raw)).tolist(),
                effort_increment=(cmd-prior_cmd[side]).tolist(),configured_rate_margin=(np.asarray(c['effort_rate_bounds_per_step'])-np.abs(cmd-prior_cmd[side])).tolist(),
                readback_error_arm_Nm=float(np.max(np.abs((cmd-app)[:7]))),readback_error_fingers_N=float(np.max(np.abs((cmd-app)[7:]))),
                saturated=a['saturated'],rate_limited=a['rate_limited'])
            prior_cmd[side]=cmd
        closure=closure_diagnostics(obj,arms['left']['actual_tcp'],arms['right']['actual_tcp'])
        if r1 and (closure['valid'] != row['runtime_gate']['valid'] or any(abs(v-row['runtime_gate']['closure'][k])>1e-12 for k,v in closure['metrics'].items())):
            raise ValueError('ARCHIVED_GATE_RECOMPUTATION_MISMATCH')
        model_closure=closure_diagnostics(obj,arms['left']['model_tcp'],arms['right']['model_tcp'])
        ref_closure=closure_diagnostics(obj,model.fk('left',qref[:7]),model.fk('right',qref[7:]))
        # Projection onto the reference translation direction is a diagnostic
        # lag-equivalent, not a fitted actuator delay or a causal attribution.
        step=steps[min(int(t/.02),4)]
        vel=(np.asarray(step['desired_object_pose_next'])[:3,3]-np.asarray(step['object_pose_initial'])[:3,3])/.02
        common=np.asarray(closure['common_translation_vector_m'])
        lag=-np.dot(common,vel)/np.dot(vel,vel) if t<.1 else None
        result.append(dict(index=i,physics_step_index=row['physics_step_index'],time_s=t,
            controller_time_s=row.get('controller_time_s',t),physics_time_s=row.get('physics_time_s'),
            recorded_joint_reference_cursor_s=(row['controller_time_s'] if r1 else max(0,t-c['physics_dt_s'])),
            object_reference_cursor_s=t, recorded_reference=row['reference'],virtual_object_pose=obj.tolist(),arms=arms,
            closure=closure,model_closure=model_closure,reference_closure=ref_closure,translation_lag_equivalent_s=None if lag is None else float(lag),
            archived_runtime_gate=row.get('runtime_gate'),source_phase=row['phase']))
    first=next((r for r in result if not r['closure']['valid']),None)
    summary=dict(frame_count=len(result),first_geometry_rejection=None if first is None else dict(index=first['index'],physics_step_index=first['physics_step_index'],time_s=first['time_s'],reasons=first['closure']['reasons']),
        maximum_closure={k:max(r['closure']['metrics'][k] for r in result) for k in result[0]['closure']['metrics']},
        max_common_translation_m=max(r['closure']['common_translation_m'] for r in result),
        max_joint_position_error_rad=max(np.max(np.abs(a['q_error_rad'])) for r in result for a in r['arms'].values()),
        max_joint_velocity_error_rad_s=max(np.max(np.abs(a['dq_error_rad_s'])) for r in result for a in r['arms'].values()),
        max_reference_translation_m=max(max(r['reference_closure']['metrics'][k] for k in ['relative_translation_m','left_absolute_translation_m','right_absolute_translation_m']) for r in result),
        max_model_vs_tcp_translation_m=max(a['model_vs_tcp_translation_m'] for r in result for a in r['arms'].values()),
        max_PD_arm_reconstruction_error_Nm=max(a['raw_reconstruction_error_arm_Nm'] for r in result for a in r['arms'].values()),
        saturated_count=sum(a['saturated'] for r in result for a in r['arms'].values()),rate_limited_count=sum(a['rate_limited'] for r in result for a in r['arms'].values()))
    return dict(source=base+'/evidence/formal_run_v1/tracking_observations.json',source_sha256=sha(ROOT/base/'evidence/formal_run_v1/tracking_observations.json'),
                semantics='post-step matched-time; stops after rejection' if r1 else 'pre-step mixed-reference; retrospective only; archive continues after geometry failure',summary=summary,rows=result)


def dynamic_availability(steps):
    r3=read('offline/task28_p2_4_r3/evidence/stability_pass/zero_load_hold_observations.json')['samples']
    q0=np.asarray(steps[0]['q_initial'])
    bilateral=[a for name in ['task28_p5_qp_5','task28_p5_qp_5_r1'] for row in read('offline/'+name+'/evidence/formal_run_v1/tracking_observations.json')['observations'] for a in row['arms'].values()]
    return dict(status='NOT_ESTIMATED_AT_BILATERAL_MOVING_STATE',
        M_c_missing_in_QP5_and_R1=all('mass_matrix' not in a and 'coriolis_compensation' not in a for a in bilateral),
        gravity_available=all('gravity_pre' in a for a in bilateral), physical_bounds_authenticated=False,
        r3_sample_count=len(r3),r3_has_M_c_g=all(all(k in row for k in ['mass_matrix','coriolis_compensation','gravity_compensation']) for row in r3),
        r3_min_posture_difference_inf_rad={side:float(min(np.max(np.abs(np.asarray(r['q_rad_or_m'])[:7]-q0[sl])) for r in r3)) for side,sl in [('left',slice(0,7)),('right',slice(7,14))]},
        installed_API='Isaac articulation APIs require initialized valid Physics Simulation View; static source inspected, no API invoked',
        conclusion='R3 M/c/g at another posture cannot authenticate bilateral inverse dynamics. Original desired velocity jumps have no finite boundary qdd. No newly time-parameterized candidate or torque feasibility result.')


def build_diagnostics():
    binding=verify_sources()
    positive=load_qp3_positive();steps=positive['raw_qp2']['steps'];model=RobotModel()
    results={name:scan_archive(name,steps,model) for name in ['task28_p5_qp_5','task28_p5_qp_5_r1']}
    row=read('offline/task28_p5_qp_5_r1/evidence/formal_run_v1/tracking_observations.json')['observations'][0]
    controls=negative_controls(row,steps,read('offline/task28_p5_qp_5_r1/preregistration.json'))
    if not all(c['passed'] for c in controls):
        raise ValueError('NEGATIVE_CONTROL_FAILURE')
    return dict(schema='task28.g2_p5_qp_5_r2.diagnostics.v2',status='G2_P5_QP_5_R2_DIAGNOSTIC_PASS_ONLY',
                execution_authorized=False,new_physics_run=False,continuous_safety_certified=False,
                original_reference_identity='task28_qp3_seed28_transport40_5step',derived_reference_identity=None,
                source_binding=binding,reference=reference_diagnostics(steps,model),archives=results,
                dynamic_availability=dynamic_availability(steps),negative_controls=controls)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    if args.output.exists():
        raise FileExistsError('NEW_OUTPUT_FILE_REQUIRED')
    result=build_diagnostics();args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(result,stream,indent=2,sort_keys=True,allow_nan=False);stream.write('\n')
    print(json.dumps(dict(status=result['status'],archives={k:v['summary'] for k,v in result['archives'].items()}),sort_keys=True))


if __name__=='__main__':
    main()
