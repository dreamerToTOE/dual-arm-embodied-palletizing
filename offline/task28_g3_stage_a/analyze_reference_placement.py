"""All-frame reference/placement comparison; source FK and actual grasp stay separate."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from offline.task28_g1_1.task28_validator.model import RobotModel
from offline.task28_p5_qp_6_r12.qp_math import se3_log
from offline.task28_g3_stage_a.analyze_suspended_probe import open_trace
from offline.task28_g3_stage_a.analyze_full_cycle import distribution

PLACEMENT=np.array([.770,.060,.260])


def analyze(run, output=None):
    output=run if output is None else output
    output.mkdir(parents=True,exist_ok=True)
    model=RobotModel(); offsets=None; frozen_rel=None; loaded=[]; all_rows=[]
    first_open=None; last_closed=None; last_cube=None
    with open_trace(run) as stream:
        for line in stream:
            f=json.loads(line)
            if f.get('event')!='TORQUE_FRAME':continue
            qp=f.get('qp') or {}; mode=(qp.get('trajectory_adapter') or {}).get('task_mode')
            cube=f['cube_runtime']; pos=np.array(cube['pose_world']['position_m'])
            actual_obj=np.eye(4);actual_obj[:3,3]=pos
            actual_obj[:3,:3]=Rotation.from_quat(np.array(cube['pose_world']['orientation_xyzw'])[[1,2,3,0]]).as_matrix()
            closed=(f.get('reaction_diagnostic') or {}).get('suctions_closed') or {}
            row={'time_s':f['time_s'],'cube_x_m':pos[0],'cube_y_m':pos[1],'cube_z_m':pos[2],
                'supported':cube.get('table_support_contact'),
                'table_force_z_N':(cube.get('table_contact_force_world_N') or [None]*3)[2],
                'cube_speed_m_s':float(np.linalg.norm(cube['linear_velocity_m_s'])),
                'left_closed':closed.get('left'),'right_closed':closed.get('right')}
            if mode=='DUAL_ATTACHED' and all(closed.get(s) is True for s in ('left','right')):
                tcps={s:np.array(f['tcp_pose_world'][s]) for s in ('left','right')}
                if offsets is None:
                    offsets={s:np.linalg.inv(actual_obj)@T for s,T in tcps.items()}
                    frozen_rel=np.linalg.inv(tcps['left'])@tcps['right']
                source={s:model.fk(s,f['q_ref_rad'][s]) for s in tcps}
                objdiag=qp.get('object_reference') or {}
                if objdiag.get('active'):
                    desired_obj=np.array(objdiag['T_WO_des'])
                    desired={s:np.array(objdiag['tcp_desired_world'][s]) for s in tcps}
                else:
                    implied={s:T@np.linalg.inv(offsets[s]) for s,T in source.items()}
                    desired_obj=np.eye(4);desired_obj[:3,3]=np.mean([T[:3,3] for T in implied.values()],axis=0)
                    desired_obj[:3,:3]=Slerp([0,1],Rotation.from_matrix([implied['left'][:3,:3],implied['right'][:3,:3]]))(.5).as_matrix()
                    desired=source
                row.update({'desired_x_m':desired_obj[0,3],'desired_y_m':desired_obj[1,3],'desired_z_m':desired_obj[2,3],
                    'cube_tracking_error_mm':float(np.linalg.norm(pos-desired_obj[:3,3])*1000),
                    'cube_rotation_error_rad':float(np.linalg.norm(Rotation.from_matrix(actual_obj[:3,:3].T@desired_obj[:3,:3]).as_rotvec())),
                    'common_error_mm':float(np.linalg.norm(qp['common_tcp_error'][:3])*1000),
                    'relative_error_mm':float(np.linalg.norm(qp['relative_tcp_error'][:3])*1000),
                    'desired_closure_mm':float(np.linalg.norm(se3_log(np.linalg.inv(frozen_rel)@np.linalg.inv(desired['left'])@desired['right'])[:3])*1000),
                    'source_closure_mm':float(np.linalg.norm(se3_log(np.linalg.inv(frozen_rel)@np.linalg.inv(source['left'])@source['right'])[:3])*1000),
                    'actual_closure_mm':float(np.linalg.norm(se3_log(np.linalg.inv(frozen_rel)@np.linalg.inv(tcps['left'])@tcps['right'])[:3])*1000),
                    'left_q_error_rad':float(np.max(np.abs(np.array(f['q_rad']['left'])-f['q_ref_rad']['left']))),
                    'right_q_error_rad':float(np.max(np.abs(np.array(f['q_rad']['right'])-f['q_ref_rad']['right']))),
                    'ff_active':qp.get('g3_qp_contact_load_model')=='assumed_commanded_wrench_not_measured',
                    'internal_candidate_N':qp.get('g3_internal_force',{}).get('estimated_internal_force_N'),
                    'torque_target_residual_Nm':qp.get('internal_torque_target_residual_inf_Nm')})
                loaded.append(row);last_closed=row.copy()
            elif offsets is not None and first_open is None and not any(closed.values()):first_open=row.copy()
            last_cube=row.copy();all_rows.append(row)
    def metrics(rows):
        keys=('cube_tracking_error_mm','cube_rotation_error_rad','common_error_mm','relative_error_mm','desired_closure_mm','source_closure_mm','actual_closure_mm','internal_candidate_N','torque_target_residual_Nm')
        return {k:distribution([r[k] for r in rows if r.get(k) is not None]) for k in keys}
    result={'run_id':run.name,'loaded_samples':len(loaded),'all_loaded':metrics(loaded),
        'terminal_last_1s':metrics([r for r in loaded if r['time_s']>=loaded[-1]['time_s']-1]) if loaded else None,
        'high_transport':metrics([r for r in loaded if r['desired_x_m']>.52 and r['desired_z_m']>.4]),
        'last_bilateral_closed_frame':last_closed,'first_bilateral_open_frame':first_open,'final_frame':last_cube,
        'placement_target_world_m':PLACEMENT.tolist(),
        'final_placement_xyz_error_mm':((np.array([last_cube['cube_x_m'],last_cube['cube_y_m'],last_cube['cube_z_m']])-PLACEMENT)*1000).tolist()}
    result['final_xy_error_mm']=float(np.linalg.norm(result['final_placement_xyz_error_mm'][:2]))
    result['release_to_final_height_mm']=(first_open['cube_z_m']-last_cube['cube_z_m'])*1000 if first_open else None
    result['precision_placement_observed']=bool(first_open and last_closed.get('supported') is True
        and abs(result['release_to_final_height_mm'])<=.25 and result['final_xy_error_mm']<=.25
        and abs(result['final_placement_xyz_error_mm'][2])<=.25)
    columns=list(dict.fromkeys(k for r in all_rows for k in r))
    with (output/'reference_placement_timeseries.csv').open('w') as out:
        writer=csv.DictWriter(out,fieldnames=columns);writer.writeheader();writer.writerows(all_rows)
    (output/'reference_placement_metrics.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result


def plot(before,after):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(4,1,figsize=(11,10),layout='constrained')
    for run,label in ((before,'before'),(after,'corrected')):
        with (run/'reference_placement_timeseries.csv').open() as stream:
            rows=[r for r in csv.DictReader(stream) if r.get('desired_x_m')]
        t=np.array([float(r['time_s']) for r in rows]);t-=t[0]
        for ax,key in zip(axes[:3],('cube_tracking_error_mm','relative_error_mm','desired_closure_mm')):
            ax.plot(t,[float(r[key]) for r in rows],label=label)
        tail=rows[-min(60,len(rows)):]
        axes[3].plot(np.arange(len(tail))/60,[float(r['cube_z_m'])*1000-260 for r in tail],label=label+' pre-release height')
    for ax,title in zip(axes,('Cube vs planned object (mm)','Relative TCP tracking error (mm)','Desired grasp closure error (mm)','Cube above nominal support (mm)')):
        ax.set_ylabel(title);ax.grid(alpha=.3);ax.legend()
    axes[2].set_yscale('symlog',linthresh=1e-5)
    for ax in axes[:3]:ax.set_xlabel('Physics seconds since attachment')
    axes[3].set_xlabel('Last one second of bilateral attachment')
    fig.suptitle('Stage A reference and placement correction: all attached samples retained')
    fig.savefig(after/'before_after_reference_placement.png',dpi=150);plt.close(fig)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('run',type=Path);parser.add_argument('--before',type=Path)
    parser.add_argument('--assert-precision',action='store_true');args=parser.parse_args()
    baseline_output=args.run/'baseline_comparison'
    if args.before:analyze(args.before, baseline_output)
    m=analyze(args.run)
    if args.before:plot(baseline_output,args.run)
    print(json.dumps({k:v for k,v in m.items() if k not in ('all_loaded','terminal_last_1s','high_transport')},indent=2))
    if args.assert_precision:
        assert m['all_loaded']['desired_closure_mm']['max']<=.150, 'INCONSISTENT_REFERENCE'
        assert m['precision_placement_observed'], 'UNSUPPORTED_OR_INACCURATE_PLACEMENT'
