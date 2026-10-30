import numpy as np
from offline.task28_p5_qp_6_r13.audit_isaac import distribution, finite_qp_inputs

def test_p95_interpolates_and_keeps_all_exceedances_in_peak():
    d=distribution([0,1,2,3,100])
    assert d['p95_m']==np.percentile([0,1,2,3,100],95,method='linear')
    assert d['peak_m']==100 and d['n']==5

def test_no_data_is_not_zero_error_pass():
    assert distribution([]) is None

def test_nan_cannot_evade_qp_bound_audit():
    qp={k:np.zeros(14) for k in ('qdot','q_ref','dq_ref')}
    qp['virtual_goal']=np.eye(4)
    assert finite_qp_inputs(qp,np.zeros(14))
    for key in qp:
        bad={k:v.copy() for k,v in qp.items()};bad[key][3]=np.nan
        assert not finite_qp_inputs(bad,np.zeros(14))

def test_wrong_dimension_cannot_enter_qp_activation_audit():
    qp={k:np.zeros(14) for k in ('qdot','q_ref','dq_ref')}
    qp['virtual_goal']=np.eye(4)
    assert not finite_qp_inputs(qp,np.zeros(13))
    qp['qdot']=np.zeros((2,7))
    assert not finite_qp_inputs(qp,np.zeros(14))
