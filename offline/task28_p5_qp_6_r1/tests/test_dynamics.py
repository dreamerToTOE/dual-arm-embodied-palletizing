import numpy as np
import pytest
from offline.task28_p5_qp_6_r1.dynamics import mimic_basis,reduced_mimic_model,constrained_prediction,arm_effort_with_fingers,reaction_structure,evaluate_R_step
from offline.task28_p5_qp_6_r1.common import config


def test_mimic_relation_virtual_work_and_nontrivial_reduction():
    S,J=mimic_basis(); assert np.max(abs(J@S))==0
    rng=np.random.default_rng(61); A=rng.normal(size=(9,9)); M=A.T@A+np.eye(9); h=rng.normal(size=9)
    Mr,hr,_,_=reduced_mimic_model(M,h); v=rng.normal(size=8); tau=rng.normal(size=9)
    assert np.allclose(Mr,S.T@M@S) and Mr.shape==(8,8)
    assert not np.allclose(Mr[-1,-1],M[-1,-1])
    assert (S@v)@tau==pytest.approx(v@(S.T@tau))
    assert v@Mr@v==pytest.approx((S@v)@M@(S@v))


def test_two_mass_mimic_independent_hand_solution():
    # Fingers mass 2 and 3, same displacement, applied +10 N to master.
    M=np.eye(9); M[7:,7:]=np.diag([2.,3.]); tau=np.zeros(9); tau[7]=10.
    q=np.zeros(9); q[7:]=.02
    a,_=constrained_prediction(M,np.zeros(9),tau,q,np.zeros(9),.01,[0,0],[.04,.04])
    assert a is not None and np.allclose(a[7:],[2,2],atol=1e-7)
    residual=M@a-tau
    assert residual[7]==pytest.approx(-6,abs=1e-7) and residual[8]==pytest.approx(6,abs=1e-7)


@pytest.mark.parametrize('effort,expected',[(2.,0.),(-2.,-2.)])
def test_upper_stop_unilateral_release_not_bilateral(effort,expected):
    M=np.eye(9); q=np.zeros(9); q[7:]=.04; tau=np.zeros(9); tau[7:]=effort
    a,_=constrained_prediction(M,np.zeros(9),tau,q,np.zeros(9),.01,[0,0],[.04,.04])
    assert a is not None and np.allclose(a[7:],expected,atol=1e-7)


def test_reaction_null_force_is_causal_negative_for_free_model():
    M=np.eye(9); q=np.zeros(9); q[7:]=.02; tau=np.zeros(9); tau[7:]=[.1,-.1]
    a,_=constrained_prediction(M,np.zeros(9),tau,q,np.zeros(9),.01,[0,0],[.04,.04])
    assert np.linalg.norm(a)<1e-7
    assert np.max(abs(np.linalg.solve(M,tau)-a))>.09


def test_analytic_arm_cancellation_includes_all_finger_terms():
    M=np.eye(9); b=np.arange(7)*.01
    M[:7,7]=b; M[:7,8]=-b; M[7,:7]=b; M[8,:7]=-b
    af=np.array([.3,.3]); aa=np.arange(7)*.02
    assert np.max(abs(M[:7,7:]@af))<1e-15
    full=M@np.r_[aa,af]
    assert np.allclose(full[:7],M[:7,:7]@aa)
    # Unequal acceleration breaks the cancellation; must remain observable.
    assert np.max(abs(M[:7,7:]@np.array([.3,-.3])))>.01


def test_no_arbitrary_slice_reduction_for_general_mass_matrix():
    M=np.eye(9); M[0,7:]=[.1,.2]; M[7:,0]=[.1,.2]
    assert abs((M[:7,7:]@np.ones(2))[0]-.3)<1e-15
    q=np.zeros(9); q[7:]=.04; pre={'q':q,'dq':np.zeros(9),'M':M,'c':np.zeros(9),'g':np.zeros(9)}
    post={'q':q,'dq':np.zeros(9)}
    gate=evaluate_R_step(pre,post,np.zeros(9),np.zeros(7),.01,[.04,.04],config())
    assert not gate['valid'] and 'STRUCTURAL_ARM_FINGER_CANCELLATION' in gate['reasons']


def test_full_command_finger_policy_units_and_no_implicit_drive():
    M=np.eye(9); q=np.zeros(9); q[7:]=.039999; dq=np.zeros(9); g=np.arange(9)*.01
    tau=arm_effort_with_fingers(M,np.zeros(9),g,q,dq,np.ones(7)*.1,config())
    assert tau.shape==(9,)
    assert np.allclose(tau[:7],g[:7]+.1)
    assert np.allclose(tau[7:],g[7:]+.00002)


def test_mimic_only_fit_does_not_explain_symmetric_upper_reaction():
    M=np.eye(9); q=np.zeros(9); q[7:]=.04; dq=np.zeros(9); postdq=np.zeros(9); tau=np.zeros(9); tau[7:]=.1
    r=reaction_structure(M,np.zeros(9),np.zeros(9),tau,q,dq,q,postdq,.01,[.04,.04],config())
    assert abs(r['mimic_only_permitted_residual'][-1])>.1
    assert np.max(abs(r['constraint_unexplained_residual']))<1e-12
    assert np.all(r['algebraically_fitted_lambda_not_sensor'][1:]>=0)


def test_same_reaction_at_interior_is_not_excused_as_upper_stop():
    M=np.eye(9); q=np.zeros(9); q[7:]=.02; tau=np.zeros(9); tau[7:]=.1
    r=reaction_structure(M,np.zeros(9),np.zeros(9),tau,q,np.zeros(9),q,np.zeros(9),.01,[.04,.04],config())
    assert not np.any(r['upper_active_candidates'])
    assert np.max(abs(r['constraint_unexplained_residual'][7:]))>.09


def test_infeasible_mimic_with_disjoint_finger_ranges_rejected():
    q=np.zeros(9); q[7:]=.02
    a,_=constrained_prediction(np.eye(9),np.zeros(9),np.zeros(9),q,np.zeros(9),.01,[.03,0],[.04,.01])
    assert a is None
