import pytest
from research.stage_b_contact_switch_g1a.scripts.g0r_math import momentum_ledger, force_from_impulse

def test_command_impulse_is_included_once():
    out=momentum_ledger(.8,[0,0,-9.81],.1,[0,0,0],[1.25,0,-.981],[0,0,0],[0,0,0],[10,0,0])
    assert out['applied_force_impulse_Ns']==pytest.approx([1,0,0])
    assert out['unexplained_impulse_Ns']==pytest.approx([0,0,0])

def test_impulse_force_uses_physics_dt():
    assert force_from_impulse([1,0,0],.02)[0]==pytest.approx(50)
    with pytest.raises(ValueError): force_from_impulse([1,0,0],0)
