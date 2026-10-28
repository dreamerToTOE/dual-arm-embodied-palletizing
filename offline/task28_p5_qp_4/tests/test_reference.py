"""Independent expected values from saved upstream evidence, plus input faults."""
import ast
from dataclasses import asdict, replace
import json
from pathlib import Path
import pytest
import numpy as np
from task28_p5_qp_4 import reference as m
from task28_p5_qp_4.controls import INPUT_FAULTS, mutated_input, negative_controls

@pytest.fixture(scope='module')
def refs():
    return m.build_reference_sequence(m.load_qp3_positive())

def test_archive_q_qdot_object_exact_without_adapter_reconstruction(refs):
    source=json.loads(m.QP3_RECORD.read_text())['positive']
    for i, ref in enumerate(refs):
        step=source['raw_qp2']['steps'][i]
        assert list(ref.left_q_rad+ref.right_q_rad)==step['q_next']
        assert list(ref.left_qdot_rad_s+ref.right_qdot_rad_s)==step['qdot14']
        assert np.array_equal(ref.object_pose_world,step['desired_object_pose_next'])
        assert max(abs(b-a-.02*v) for a,b,v in zip(step['q_initial'],step['q_next'],step['qdot14'])) < 1e-15
        assert ref.left_joint_names==tuple(f'left_fr3_joint{j}' for j in range(1,8))
        assert ref.right_joint_names==tuple(f'right_fr3_joint{j}' for j in range(1,8))
    assert m.validate_sequence(refs,now_s=.1,max_age_s=0)['count']==5

def test_no_false_contact_confirmation_acceleration_or_dynamics(refs):
    for ref in refs:
        assert ref.contact_mode=='bilateral_transitioning'
        assert ref.synchronization['phase_confirmation']=='NOT_RUN'
        assert ref.synchronization['release_authorized'] is False
        assert ref.acceleration_rad_s2 is None
        assert ref.execution_authorized is False
        assert 'tau' not in asdict(ref)
        assert ref.source_model['fixture_manifest_sha256']==m.sha256(m.FIXTURE/'fixture_manifest.json')

@pytest.mark.parametrize('name,path,value,reason',INPUT_FAULTS,ids=[x[0] for x in INPUT_FAULTS])
def test_fault_injection(name,path,value,reason):
    with pytest.raises(ValueError,match=reason):
        m.build_reference_sequence(mutated_input(path,value))

@pytest.mark.parametrize('field,value',[
 ('cube_id','Cube_02'),('frame_id','base'),('source_identity','unknown'),
 ('provenance','REAL_FEEDBACK'),('fixture_id','different'),('phase','CONFIRMED_AIRBORNE'),
 ('contact_mode','supported'),('safety_status','CONTINUOUSLY_SAFE'),
 ('execution_authorized',True),('sequence',True),('source_step_index',1),
 ('time_s',float('nan')),('stamp_s',float('nan')),('time_s',-1.),
 ('left_q_rad',(float('nan'),)*7),('right_qdot_rad_s',(0.,)*7),
 ('acceleration_rad_s2',(0.,)*14),('acceleration_rad_s2',(1.,)*14),
 ('object_pose_world',((2.,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))),
])
def test_typed_reference_cannot_be_mutated_to_false_accepted(field,value,refs):
    with pytest.raises(ValueError):
        m.validate_reference(replace(refs[0],**{field:value}))

def test_pair_async_and_stale(refs):
    bad=dict(refs[0].synchronization,right_stamp_s=.021)
    with pytest.raises(ValueError,match='ASYNC'):
        m.validate_reference(replace(refs[0],synchronization=bad))
    with pytest.raises(ValueError,match='STALE'):
        m.validate_sequence([refs[0],replace(refs[1],stamp_s=.02)])
    with pytest.raises(ValueError,match='STALE'):
        m.validate_sequence(refs,now_s=.2,max_age_s=.01)
    with pytest.raises(ValueError,match='SEQUENCE'):
        m.validate_sequence([refs[1],refs[0]])

def test_original_negative_archive_returns_no_reference():
    neg=m.load_archive()['negative_payload_table']
    assert neg['position_targets']==[]
    assert 'COLLISION:Cube_01/box|task28_table/box' in neg['raw_qp2']['initial_state']['reasons']
    with pytest.raises(ValueError,match='UPSTREAM_STATUS'):
        m.build_reference_sequence(neg)

def test_missing_dependency_and_tampered_archive(tmp_path):
    with pytest.raises(ValueError,match='MISSING_SOURCE'):
        m.source_hashes(tmp_path)
    p=tmp_path/'altered.json';p.write_text(m.QP3_RECORD.read_text()+' ')
    with pytest.raises(ValueError,match='HASH_MISMATCH'):
        m.load_archive(p)

@pytest.mark.parametrize('field,value',[('execution_authorized',True),('acceleration_status','FINITE_DIFFERENCE'),('timestamp_semantics','SERVO'),('joint_units','degrees')])
def test_contract_semantic_changes_rejected(field,value):
    with pytest.raises(ValueError,match='CONTRACT_MISMATCH'):
        replace(m.ReferenceContract(),**{field:value}).validate()

def test_all_saved_control_procedures_reject():
    assert all(c['passed'] for c in negative_controls())

def test_runtime_contract_has_no_actuator_dependencies():
    tree=ast.parse(Path(m.__file__).read_text())
    imports=[n.module for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)]
    imports += [a.name for n in ast.walk(tree) if isinstance(n,ast.Import) for a in n.names]
    assert not any(any(x in (i or '') for x in ('rclpy','rospy','isaac','omni','bridge')) for i in imports)
    calls=[n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)]
    assert not set(calls)&{'publish','apply_action','set_joint_efforts','set_joint_positions'}

def test_wrench_and_kinematic_nullspaces_have_distinct_domains(refs):
    from offline.task28_p2_1.grasp import GraspModel
    from offline.task28_p5_qp_0.core import velocity_maps
    from offline.task28_g1_1.task28_validator.model import RobotModel
    ref=refs[0]; q=np.array(ref.left_q_rad+ref.right_q_rad)
    C=velocity_maps(RobotModel(),q,np.array(ref.object_pose_world))['C']
    G=GraspModel.frozen(np.array(ref.object_pose_world)).matrix()
    assert C.shape==(6,14) and G.shape==(6,12)
    assert np.linalg.matrix_rank(C)==6 and np.linalg.matrix_rank(G)==6
    # G consumes 12 contact-wrench components. A 14-joint velocity cannot be
    # interpreted as contact internal wrench or forwarded to G.
    with pytest.raises(ValueError):
        G@np.array(ref.left_qdot_rad_s+ref.right_qdot_rad_s)

def test_independent_recheck_uses_frozen_geometry_without_solver_or_rollout(monkeypatch):
    from task28_p5_qp_4.recheck import recheck
    from offline.task28_p5_qp_2.driver import FeedbackDriver
    from offline.task28_p5_qp_2.feedback import FeedbackVelocityQP
    def forbidden(*args, **kwargs):
        raise AssertionError('No new solve/rollout is allowed in archive recheck')
    monkeypatch.setattr(FeedbackDriver,'run',forbidden)
    monkeypatch.setattr(FeedbackVelocityQP,'solve',forbidden)
    result=recheck()
    assert result['pass'] and result['source_lineage_exact']
    assert result['negative_payload_table_rejected']
    assert len(result['positive_steps'])==5
    assert all(row['accepted'] for row in result['positive_steps'])
