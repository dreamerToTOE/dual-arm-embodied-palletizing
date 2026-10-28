import numpy as np, pytest
from offline.task28_p5_qp_5.controller import sample_reference, explicit_effort
from offline.task28_p5_qp_4.reference import load_qp3_positive, build_reference_sequence
from offline.task28_p5_qp_5.evaluate import load_config

def test_reference_sampler_boundaries():
 refs=build_reference_sequence(load_qp3_positive())
 q0,qd,k,p=sample_reference(refs,0.0); assert k==0 and p=='interval'
 q,qd,k,p=sample_reference(refs,0.02); assert k==1 and p=='interval'
 q,qd,k,p=sample_reference(refs,0.1); assert k==4 and p=='final_hold' and np.allclose(qd,0)
def test_full_nine_effort_and_no_padding():
 x=explicit_effort(np.zeros(9),np.zeros(9),np.ones(9),np.zeros(9),np.ones(9),np.ones(9),np.ones(9),np.ones(9)*60,np.ones(9)*100)
 assert x['command'].shape==(9,) and np.all(x['command']!=0)
def test_unsupported_acceleration_is_explicit():
 assert load_config()['sampler'].find('interval-start qdot') >= 0
def test_wrong_mapping_rejected():
 refs=build_reference_sequence(load_qp3_positive()); assert tuple(refs[0].left_joint_names)[0]=='left_fr3_joint1'
