import json
import numpy as np
import pytest
from offline.task28_r14.no_contact import guard_command, plan, R3


def setup():
    cfg=json.loads((R3/'experiment_preregistration.json').read_text())
    p=plan(); q=np.array(cfg['configuration_rad']+cfg['finger_q_m']); z=np.zeros(9)
    return cfg,p,q,z


def test_first_gravity_and_regular_command():
    cfg,p,q,z=setup(); g=z.copy();g[3]=31
    guard_command(g,q,z,g,None,None,cfg,p)
    guard_command(g,q,z,g,g,z,cfg,p)


def test_rejects_effort_below_legacy_but_above_frozen_research():
    cfg,p,q,z=setup(); u=z.copy();u[5]=5
    with pytest.raises(ValueError, match='EFFORT_LIMIT'): guard_command(u,q,z,u,None,None,cfg,p)


def test_rejects_per_second_rate_despite_legacy_per_step_pass():
    cfg,p,q,z=setup();u=z.copy();u[2]=3
    with pytest.raises(ValueError, match='EFFORT_RATE_LIMIT'): guard_command(u,q,z,u,z,z,cfg,p)


def test_nonfinite_and_non_gravity_bootstrap():
    cfg,p,q,z=setup();u=z.copy();u[1]=float('nan')
    with pytest.raises(ValueError, match='INVALID_PRECOMMAND'): guard_command(u,q,z,z,None,None,cfg,p)
    u=z.copy();u[1]=.1
    with pytest.raises(ValueError, match='INITIAL_COMMAND_NOT_GRAVITY'): guard_command(u,q,z,z,None,None,cfg,p)


def test_acceleration_rejects_before_write():
    cfg,p,q,z=setup();dq=z.copy();dq[0]=2
    with pytest.raises(ValueError, match='ACCELERATION_LIMIT'): guard_command(z,q,dq,z,z,z,cfg,p)
