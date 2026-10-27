"""Reproduce fixed synthetic wrench controls; no simulation or physical input."""
import hashlib
import json
import platform
import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from offline.task28_p2_1.grasp import GraspModel, metric_matrix, request_physical_estimate

ROOT = Path(__file__).resolve().parents[2]

def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    identity = np.eye(4)
    model = GraspModel.frozen(identity)
    G = model.matrix('object')
    a = .061
    cases = {
        'zero': (np.zeros(12), np.zeros(6)),
        'left_force_x': (np.array([0.,1.,0.,0.,0.,0.]+[0.]*6), np.array([1.,0.,0.,0.,0.,a])),
        'equal_object_x': (np.array([0.,1.,0.,0.,0.,0.,0.,-1.,0.,0.,0.,0.]), np.array([2.,0.,0.,0.,0.,0.])),
        'opposing_local_x_squeeze': (np.array([1.,0.,0.,0.,0.,0.,1.,0.,0.,0.,0.,0.]), np.zeros(6)),
        'opposing_object_x_couple': (np.array([0.,1.,0.,0.,0.,0.,0.,1.,0.,0.,0.,0.]), np.array([0.,0.,0.,0.,0.,2*a])),
        'pure_object_z_moment': (np.array([0.,0.,0.,0.,0.,-.5,0.,0.,0.,0.,0.,-.5]), np.array([0.,0.,0.,0.,0.,1.])),
    }
    outcases = []
    for name,(x,oracle) in cases.items():
        actual=model.net(x,'object'); decomp=model.decompose(x,output_frame='object')
        outcases.append({'id':name,'contact_local_force_then_torque':x.tolist(),'analytic_net_object':oracle.tolist(),
            'actual_net_object':actual.tolist(),'analytic_residual_inf':float(np.max(np.abs(actual-oracle))),
            'external':decomp.external.tolist(),'internal':decomp.internal.tolist(),
            'internal_null_residual':decomp.null_residual,'reconstruction_residual':decomp.reconstruction_residual,
            'metric_orthogonality':decomp.metric_orthogonality})
    # A separate direct KKT solve is an independent numerical allocation oracle.
    target=np.array([3.,-2.,7.848, .1,-.2,.3])
    metrics={'symmetric':metric_matrix(), 'asymmetric':np.diag([1.,1.,1.,268.729,268.729,268.729,4.,4.,4.,268.729,268.729,268.729])}
    allocations=[]
    for label,W in metrics.items():
        alloc=model.allocate(target,W,output_frame='object')
        kkt=np.block([[W,G.T],[G,np.zeros((6,6))]])
        reference=np.linalg.solve(kkt,np.r_[np.zeros(12),target])[:12]
        allocations.append({'id':label,'metric_diagonal':np.diag(W).tolist(),'target':target.tolist(),
            'contacts':alloc.contacts.tolist(),'objective':alloc.objective,'rank':alloc.rank,
            'equilibrium_residual':alloc.residual,'independent_kkt_solution_difference_inf':float(np.max(np.abs(reference-alloc.contacts))),
            'independent_kkt_equilibrium_residual':float(np.linalg.norm(G@reference-target))})
    T=np.eye(4);T[:3,:3]=Rotation.from_euler('xyz',[.3,-.2,.7]).as_matrix();T[:3,3]=[.73,-.04,.39]
    rot=GraspModel.frozen(T);x=np.arange(1.,13.)/10.; neto=rot.net(x,'object');netw=rot.net(x,'world')
    expected=np.r_[T[:3,:3]@neto[:3],T[:3,:3]@neto[3:]]
    # Virtual power oracle: contact-point velocity v+omega cross r, locally expressed.
    v=np.array([.13,-.21,.08]); omega=np.array([-.6,.2,.4]); localpower=0.
    for idx,t in enumerate((rot.object_left,rot.object_right)):
        r=T[:3,:3]@t[:3,3]; R=T[:3,:3]@t[:3,:3]; f=x[idx*6:idx*6+3]; tau=x[idx*6+3:idx*6+6]
        localpower += float(f@(R.T@(v+np.cross(omega,r)))+tau@(R.T@omega))
    netpower=float(netw[:3]@v+netw[3:]@omega)
    source_paths=['fixtures/task28_g1_0_new_single_cube_v0/fixture_manifest.json','offline/task28_g1_1/task28_validator/closed_chain.py','offline/task28_g1_1/task28_validator/se3.py','docs/tasks/TASK28_G3_P2_0_OBSERVABILITY_AUDIT.md','offline/task28_p2_1/grasp.py','offline/task28_p2_1/generate_evidence.py']
    result={'schema':'task28.g3_p2_1.synthetic_math_evidence.v1','scope':'SYNTHETIC_OFFLINE_FULL_SIX_D_AGGREGATE_NO_PHYSICAL_ESTIMATION',
        'execution_authorized':False,'fixture':'task28_g1_0_new_single_cube_v0','geometry_semantics':'G1-1 nominal TCP ±0.061 m includes 1mm gap; not observed physical attachment',
        'matrix_object':G.tolist(),'rank_unweighted':int(np.linalg.matrix_rank(G)),'singular_values_unweighted':np.linalg.svd(G,compute_uv=False).tolist(),
        'metric_diagonal':np.diag(metric_matrix()).tolist(),'metric_semantics':'normalization force 1 N, moment 0.061 Nm; not capacity or D6 threshold',
        'cases':outcases,'allocations':allocations,'rotation_control':{'T_world_object':T.tolist(),'wrench':x.tolist(),'coordinate_rotation_residual_inf':float(np.max(np.abs(netw-expected))),'virtual_power_local':localpower,'virtual_power_net':netpower,'virtual_power_abs_error':abs(localpower-netpower)},
        'physical_request':request_physical_estimate(),'source_sha256':{p:sha(ROOT/p) for p in source_paths},
        'environment':{'python':sys.version,'numpy':np.__version__,'platform':platform.platform()},'random_seeds':None}
    assert all(c['analytic_residual_inf']<1e-12 and c['internal_null_residual']<1e-9 for c in outcases)
    assert all(c['equilibrium_residual']<1e-9 and c['independent_kkt_solution_difference_inf']<1e-9 for c in allocations)
    assert result['rotation_control']['virtual_power_abs_error']<1e-12
    output=ROOT/'offline/task28_p2_1/evidence/synthetic_math.json'
    with output.open('x') as f: json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps({'cases':len(outcases),'allocations':len(allocations),'rank':result['rank_unweighted'],'max_analytic_residual':max(c['analytic_residual_inf'] for c in outcases),'max_KKT_difference':max(c['independent_kkt_solution_difference_inf'] for c in allocations),'virtual_power_error':result['rotation_control']['virtual_power_abs_error']}))
if __name__=='__main__': main()
