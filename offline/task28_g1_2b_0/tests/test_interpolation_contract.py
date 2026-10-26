"""Independent polynomial/SO(3) controls plus fail-closed typed API checks."""
from dataclasses import replace
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from task28_edge.checker import SampledEdgeValidator
from task28_edge.experiments import controls
from task28_edge.interpolation import coefficients,evaluate,derivative_bound,reconstruct,subdivisions
from task28_edge.types import (EdgeStatus as S,SamplingPolicy,JointInterpolation,Interpolation,
                              TranslationInterpolation,Motion,Source)
from task28_contact.clearance import ConfirmationEvent as E


@pytest.fixture(scope='module')
def checker():return SampledEdgeValidator()


@pytest.fixture
def trajectory(checker):return controls(checker.model)['positive_hold']


def test_cubic_hermite_matches_independently_expanded_scalar_equation():
    dt=.7;p0=np.array([.2]);p1=np.array([.8]);v0=np.array([2.]);v1=np.array([-1.])
    co=coefficients(p0,p1,v0,v1,dt,True)
    for u in np.linspace(0,1,17):
        expected=(2*u**3-3*u**2+1)*p0+(u**3-2*u**2+u)*dt*v0+(-2*u**3+3*u**2)*p1+(u**3-u**2)*dt*v1
        assert evaluate(co,u)==pytest.approx(expected,abs=1e-14)
    eps=1e-7
    assert ((evaluate(co,eps)-evaluate(co,0))/eps/dt)==pytest.approx(v0,abs=1e-5)
    assert ((evaluate(co,1)-evaluate(co,1-eps))/eps/dt)==pytest.approx(v1,abs=1e-5)
    assert derivative_bound(co)[0]>=max(abs(float((3*co[0]*u*u+2*co[1]*u+co[2])[0])) for u in np.linspace(0,1,101))


def test_shortest_so3_rotation_handles_quaternion_sign_and_halfturn(checker,trajectory):
    a,b=trajectory.waypoints
    b=replace(b,object_quaternion_xyzw=tuple(-np.array(a.object_quaternion_xyzw)))
    middle=reconstruct(a,b,.5,trajectory.interpolation,Motion.GENERIC,checker.contact.collision.support,'quat-sign')
    assert Rotation.from_quat(middle.object_quaternion_xyzw).magnitude()<1e-12
    b=replace(b,object_quaternion_xyzw=tuple(Rotation.from_euler('z',np.pi).as_quat()))
    middle=reconstruct(a,b,.5,trajectory.interpolation,Motion.GENERIC,checker.contact.collision.support,'halfturn')
    assert Rotation.from_quat(middle.object_quaternion_xyzw).magnitude()==pytest.approx(np.pi/2)
    assert middle.provenance.source is Source.MODEL_INTERPOLATION


def test_grid_resolution_bounds_and_exact_midpoint_for_hermite(checker):
    t=controls(checker.model)['C_limit'];a,b=t.waypoints;p=SamplingPolicy(max_joint_step_rad=.1)
    n=subdivisions(a,b,t.interpolation,p)
    assert n%2==0
    rows=[reconstruct(a,b,k/n,t.interpolation,t.motion,checker.contact.collision.support,t.identity) for k in range(n+1)]
    assert max(row.time_s for row in rows)==b.time_s
    assert any(row.time_s==.5 for row in rows)
    for left,right in zip(rows,rows[1:]):
        assert max(abs(x-y) for x,y in zip(left.left_q,right.left_q))<=p.max_joint_step_rad+1e-12
        assert right.time_s-left.time_s<=p.max_dt_s+1e-12


@pytest.mark.parametrize('policy',[None,SamplingPolicy(max_dt_s=0),SamplingPolicy(max_dt_s=float('nan')),
    SamplingPolicy(max_samples=1),SamplingPolicy(max_samples=True),SamplingPolicy(max_samples=100000)])
def test_invalid_sampling_contract_inconclusive(checker,trajectory,policy):
    r=checker.validate(trajectory,policy)
    assert r.status is S.EDGE_INCONCLUSIVE and not r.samples


@pytest.mark.parametrize('change',[
    lambda t:None,
    lambda t:replace(t,waypoints=()),
    lambda t:replace(t,waypoints=list(t.waypoints)),
    lambda t:replace(t,waypoints=(t.waypoints[0],replace(t.waypoints[1],time_s=t.waypoints[0].time_s))),
    lambda t:replace(t,motion='transport'),
    lambda t:replace(t,interpolation=Interpolation(rotation='linear_quaternion')),
    lambda t:replace(t,interpolation=Interpolation(joints=JointInterpolation.CUBIC_HERMITE)),
    lambda t:replace(t,interpolation=Interpolation(translation=TranslationInterpolation.CUBIC_HERMITE)),
    lambda t:replace(t,waypoints=(t.waypoints[0],replace(t.waypoints[1],left_q=(float('nan'),)*7))),
    lambda t:replace(t,waypoints=(t.waypoints[0],replace(t.waypoints[1],object_quaternion_xyzw=(0.,0.,0.,0.)))),
    lambda t:replace(t,waypoints=(t.waypoints[0],replace(t.waypoints[1],provenance=t.waypoints[0].provenance))),
])
def test_malformed_trajectory_returns_inconclusive_without_simulation(checker,trajectory,change):
    r=checker.validate(change(trajectory))
    assert r.status is S.EDGE_INCONCLUSIVE and r.evaluated_sample_count==0


def test_invalid_endpoint_does_not_receive_endpoint_valid_label(checker,trajectory):
    w=trajectory.waypoints[-1];q=list(w.left_q);q[0]=float(checker.model.joint_upper[0]+.01)
    t=replace(trajectory,waypoints=(trajectory.waypoints[0],replace(w,left_q=tuple(q))))
    r=checker.validate(t,endpoint_only=True)
    assert not r.endpoints_valid and r.status is S.SAMPLED_EDGE_REJECTED
    assert 'LEFT_JOINT_LIMIT_VIOLATION' in r.reasons


def test_wrong_phase_event_inside_geometric_hold_is_rejected(checker,trajectory):
    w=replace(trajectory.waypoints[-1],event=E.BEGIN_PLACE)
    r=checker.validate(replace(trajectory,waypoints=(trajectory.waypoints[0],w)))
    assert r.endpoints_valid and all(row['valid'] for row in r.samples)
    assert r.status is S.SAMPLED_EDGE_REJECTED
    assert 'PHASE_EVENT_IN_WRONG_MOTION_OR_LOCATION' in r.reasons


@pytest.mark.parametrize('change',[
    lambda obs:replace(obs,event='observe'),
    lambda obs:replace(obs,now_s=10**1000),
    lambda obs:replace(obs,frame=None),
])
def test_invalid_temporal_prelude_returns_inconclusive(checker,change):
    from task28_edge.experiments import a_stage
    t=a_stage(checker)['attachment_confirmation']
    t=replace(t,temporal_prelude=(change(t.temporal_prelude[0]),*t.temporal_prelude[1:]))
    r=checker.validate(t)
    assert r.status is S.EDGE_INCONCLUSIVE and not r.samples
    assert r.reasons==('INVALID_TEMPORAL_PRELUDE_CONTRACT',)


def test_enormous_integer_timestamp_rejected_without_overflow(checker,trajectory):
    w=replace(trajectory.waypoints[1],time_s=10**1000)
    r=checker.validate(replace(trajectory,waypoints=(trajectory.waypoints[0],w)))
    assert r.status is S.EDGE_INCONCLUSIVE and not r.samples


def test_backend_unavailable_is_inconclusive_not_a_collision_claim(checker,trajectory,monkeypatch,tmp_path):
    monkeypatch.setattr(checker.contact.collision.reference,'backend_path',tmp_path/'not-a-backend')
    r=checker.validate(trajectory)
    assert r.status is S.EDGE_INCONCLUSIVE
    assert any(reason.startswith('COLLISION_BACKEND_FAILURE') for reason in r.reasons)
    assert not r.endpoints_valid
