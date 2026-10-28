"""Re-evaluate saved candidates with original G1/B0 and QP diagnostic gates.

No optimizer, NR search or controller rollout is run. Numerical data are the
committed QP-3 candidates. This is CPU-only archive verification.
"""
from dataclasses import asdict
import gzip
import json
import numpy as np
from .reference import ROOT, BINDING, load_archive, build_reference_sequence, sha256
from offline.task28_p5_qp_2.driver import FeedbackDriver
from offline.task28_p5_qp_0.core import VelocityInput, velocity_maps
from offline.task28_p4_nr_0.validation import Motion


def recheck():
    archive=load_archive(); positive=archive['positive']; refs=build_reference_sequence(positive)
    binding=json.loads(BINDING.read_text())
    reg=json.loads((ROOT/'offline/task28_p5_qp_2/evidence/preregistration.json').read_text())
    node=next(n for n in reg['nodes'] if n['identity']==binding['source_node_identity'])
    nr=json.loads(gzip.decompress((ROOT/binding['nr0_path']).read_bytes()))
    saved_node=nr['transport'][node['node_index']]
    request=positive['raw_qp2']['request']
    lineage=(node['source_sha256']==sha256(ROOT/binding['nr0_path']) and
             saved_node==[node['object_pose'],node['q14']] and
             request['q14']==node['q14'] and request['object_pose']==node['object_pose'])
    engine=FeedbackDriver(); rows=[]
    for i,(step,ref) in enumerate(zip(positive['raw_qp2']['steps'],refs)):
        q=np.asarray(step['q_initial']); qd=np.asarray(step['qdot14']); qn=np.asarray(step['q_next'])
        X=np.asarray(step['object_pose_initial']); Xn=np.asarray(step['desired_object_pose_next']); dt=step['dt_s']
        velocity_request=VelocityInput(q,X,np.asarray(step['desired_object_twist']),dt)
        A,b=engine._affine_equality(velocity_request)
        affine=float(np.max(np.abs(A@qd-b)))
        err=velocity_maps(engine.controller.model,q,X,engine.controller.grasps)['B']@qd-step['desired_object_twist']
        bounds=engine.diagnostics._frozen_bounds(q,qd,dt)
        trajectory=engine.validator.trajectory([(X,q),(Xn,qn)],Motion.GENERIC,step['identity'],start_time_s=step['time_s'],time_step_s=dt,enforce_temporal_confirmation=False)
        current=engine.validator.checker.evaluate(trajectory.waypoints[0])
        predicted=engine.validator.checker.evaluate(trajectory.waypoints[1])
        edge=engine.validator.checker.validate(trajectory)
        exact_q=list(ref.left_q_rad+ref.right_q_rad)==step['q_next']
        exact_qdot=list(ref.left_qdot_rad_s+ref.right_qdot_rad_s)==step['qdot14']
        accepted=(exact_q and exact_qdot and affine<=1e-8 and np.linalg.norm(err[:3])<=1e-4 and
                  np.linalg.norm(err[3:])<=1e-3 and bounds['valid'] and current['valid'] and predicted['valid'] and edge.status.value=='sampled_edge_valid')
        rows.append({'step':i,'accepted':bool(accepted),'exact_q_copy':exact_q,'exact_qdot_copy':exact_qdot,
                     'euler_max_error_rad':float(np.max(np.abs(qn-q-dt*qd))),
                     'hard_affine_residual_inf':affine,'tracking_translation_error_m_s':float(np.linalg.norm(err[:3])),
                     'tracking_rotation_error_rad_s':float(np.linalg.norm(err[3:])),
                     'minimum_position_margin_rad':bounds['minimum_position_margin_rad'],
                     'minimum_velocity_margin_rad_s':bounds['minimum_velocity_margin_rad_s'],
                     'current_state':current,'predicted_state':predicted,'sampled_edge':asdict(edge)})
    neg=archive['negative_payload_table']; nrq=neg['raw_qp2']['request']
    nstate=engine.validator.state(nrq['object_pose'],nrq['q14'])
    nvalid=(not nstate['valid'] and 'COLLISION:Cube_01/box|task28_table/box' in nstate['reasons'])
    return {'schema':'task28.g2_p5_qp_4.independent_archive_recheck.v1',
            'pass':bool(lineage and all(r['accepted'] for r in rows) and nvalid),
            'source_lineage_exact':lineage,'node_identity':node['identity'],
            'reference_scope':'QP3_FIXED_TWIST_LOCAL_EXAMPLE_STARTING_FROM_NR0_NODE_NOT_ENTIRE_NR0_PATH',
            'positive_steps':rows,'negative_identity':neg['identity'],'negative_state':nstate,
            'negative_payload_table_rejected':nvalid,'new_qp_solve_or_rollout':False,'nr_search_run':False,
            'continuous_safety':'NOT_CERTIFIED_FINITE_SAMPLING_ONLY','execution_authorized':False}
