"""Sampled geometry/timing/smoothness, independent profile derivative checks."""
import argparse
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from .common import dump,sha,HERE
from .profiles import Profile
from offline.task28_g1_1.task28_validator.model import RobotModel
from offline.task28_g1_1.task28_validator.collision import CollisionValidator
from offline.task28_g1_1.task28_validator.closed_chain import evaluate_closure


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--output',type=Path,required=True); args=ap.parse_args()
    args.output.mkdir(exist_ok=False,parents=True); model=RobotModel(); fcl=CollisionValidator(model.fixture_dir)
    results={}
    for name in ['S05','S10']:
        p=Profile(name); rows=[]; bad=[]
        for k in range(501):
            t=k*p.duration/500; ref=p.sample(t); q=ref['q14']; frames=model.forward(q[:7],q[7:])
            closure=evaluate_closure(ref['pose'],model.fk('left',q[:7]),model.fk('right',q[7:]))
            collision=fcl.validate(frames,ref['pose'],attachment_mode='bilateral_transitioning')
            # Explicit end targets always derived from exactly the same Cube pose.
            nominal=evaluate_closure(ref['pose'],ref['left_tcp'],ref['right_tcp'])
            row={'time_s':t,'segment':ref['segment'],'q14':q,'dq14':ref['dq14'],'qdd14':ref['qdd14'],'object_pose':ref['pose'],'object_twist':ref['twist'],'object_acceleration':ref['acceleration'],
                 'joint_limit_valid':model.within_limits(q[:7]) and model.within_limits(q[7:]),'closure_valid':closure.valid,'closure_reasons':closure.reasons,'left_error_m':closure.left_object_translation_error_m,'right_error_m':closure.right_object_translation_error_m,'relative_error_m':closure.translation_error_m,'relative_rotation_rad':closure.rotation_error_rad,'collision_valid':collision.valid,'collision_reasons':collision.reasons,'shape_pairs':collision.checked_shape_pairs,'shared_object_tcp_valid':nominal.valid}
            rows.append(row)
            if not all(row[x] for x in ['joint_limit_valid','closure_valid','collision_valid','shared_object_tcp_valid']): bad.append(k)
        fd=[]
        for k in range(5):
            t=(k+.37)*p.segment_s; dt=1e-5; a=p.sample(t-dt); b=p.sample(t+dt); r=p.sample(t)
            # Direct pose velocities and derivative of analytic twist, independent
            # SO(3) differences; no original QP4 qdot is used.
            velocity=np.r_[(b['pose'][:3,3]-a['pose'][:3,3])/(2*dt),Rotation.from_matrix(b['pose'][:3,:3]@a['pose'][:3,:3].T).as_rotvec()/(2*dt)]
            fd.append({'velocity_error_inf':float(np.max(np.abs(velocity-r['twist']))),'acceleration_error_inf':float(np.max(np.abs((b['twist']-a['twist'])/(2*dt)-r['acceleration']))),'joint_acceleration_error_inf':float(np.max(np.abs((b['dq14']-a['dq14'])/(2*dt)-r['qdd14'])))})
        knot=[]
        for k in range(6):
            r=p.sample(k*p.segment_s); knot.append({'knot':k,'time_s':k*p.segment_s,'exact_pose_max_error':float(np.max(np.abs(r['pose']-p.poses[k]))),'exact_q_max_error':float(np.max(np.abs(r['q14']-p.qs[k]))),'velocity_inf':float(np.max(np.abs(r['twist']))),'acceleration_inf':float(np.max(np.abs(r['acceleration'])))})
        derivative_pass=max(v for r in fd for v in r.values())<1e-6
        results[name]={'status':'PASS_SAMPLED_ONLY' if not bad and derivative_pass else 'REJECTED','identity':p.identity,'duration_s':p.duration,'samples':501,'failed_sample_indices':bad,'max_left_error_m':max(r['left_error_m'] for r in rows),'max_right_error_m':max(r['right_error_m'] for r in rows),'max_relative_error_m':max(r['relative_error_m'] for r in rows),'fd_checks':fd,'knots':knot,'continuous_safety_certified':False,'FCL':'0.7.0','profile_source_sha':sha(HERE/'profiles.py')}
        dump(args.output/(name+'_samples.json'),rows)
    dump(args.output/'result.json',results)
    print({k:v['status'] for k,v in results.items()})
    return 0 if all(v['status']=='PASS_SAMPLED_ONLY' for v in results.values()) else 1


if __name__=='__main__': raise SystemExit(main())
