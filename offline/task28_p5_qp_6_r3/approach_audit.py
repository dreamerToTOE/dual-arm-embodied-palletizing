"""Offline bounded IK/straight-line FCL screen; no physical execution."""
import json
from dataclasses import asdict
from pathlib import Path
import numpy as np
from offline.task28_g1_1.task28_validator.model import RobotModel,DEFAULT_FIXTURE
from offline.task28_g1_1.task28_validator.closed_chain import desired_tcps
from offline.task28_g1_1.task28_validator.collision import CollisionValidator


def main():
    base=Path(__file__).resolve().parent;raw=json.loads((base/'evidence/dev_03/raw.json').read_text())['rows'];last=raw[-1];obj=np.asarray(last['cube_world'],float);model=RobotModel();collision=CollisionValidator(DEFAULT_FIXTURE);targets=desired_tcps(obj);solutions={};out={'scope':'bounded offline endpoint IK and 51-point straight-joint-space FCL screen; no execution or continuous collision proof','source':'dev_03 last saved state, not qualified for actuation','cube_world':obj.tolist(),'results':{},'validated_continuous_approach':False}
    for side,goal in zip(('left','right'),targets):
        q=np.asarray(last['arms'][side]['q'][:7]);r=model.ik(side,goal,seeds=[q],max_attempts=8,max_nfev=300);out['results'][side]=asdict(r)
        if r.success:solutions[side]=np.asarray(r.q)
    if len(solutions)==2:
        first=None;total=0;ql=np.asarray(last['arms']['left']['q'][:7]);qr=np.asarray(last['arms']['right']['q'][:7])
        for i in range(51):
            t=i/50;a=ql*(1-t)+solutions['left']*t;b=qr*(1-t)+solutions['right']*t
            hit=collision.validate(model.forward(a,b),obj,'detached');unexpected=[r for r in hit.reasons if r!='COLLISION:Cube_01/box|task28_table/box'];total+=1
            if unexpected: first={'sample':i,'fraction':t,'reasons':unexpected};break
        out['straight_joint_interpolation']={'samples_checked':total,'expected_support_exception_is_reporting_classification_not_acm':True,'first_unexpected_collision':first,'all_sampled_clear':first is None,'continuous_and_dynamic_safety_proven':False}
    else:out['straight_joint_interpolation']={'status':'SKIPPED_ENDPOINT_IK_FAILURE'}
    (base/'evidence/approach_audit.json').write_text(json.dumps(out,indent=2)+'\n')
    print({s:out['results'][s]['reason'] for s in out['results']},out['straight_joint_interpolation'])
if __name__=='__main__':main()
