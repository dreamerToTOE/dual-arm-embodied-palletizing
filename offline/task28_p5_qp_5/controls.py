from __future__ import annotations
import numpy as np

def validate_reference_controls(rows):
    """Independent software controls; every fault is fail-closed."""
    reasons=[]; prev_t=None
    for row in rows:
        t=row.get('sim_time_s')
        if not isinstance(t,(int,float)) or not np.isfinite(t): reasons.append('STALE_OR_INVALID_STATE'); continue
        if prev_t is not None and t<=prev_t: reasons.append('ASYNC_OR_OUT_OF_ORDER_REFERENCE')
        prev_t=t
        if row.get('cube_id','Cube_01')!='Cube_01': reasons.append('WRONG_CUBE_ID')
        if row.get('acceleration_rad_s2') is not None: reasons.append('UNSUPPORTED_ACCELERATION_REFERENCE')
        for side in ('left','right'):
            if row.get('arms',{}).get(side,{}).get('dof_names') != ['fr3_joint'+str(i) for i in range(1,8)]+['fr3_finger_joint1','fr3_finger_joint2']:
                reasons.append('WRONG_DOF_MAPPING:'+side)
    return {'valid':not reasons,'reasons':sorted(set(reasons))}

def negative_controls():
    base={'sim_time_s':0.0,'cube_id':'Cube_01','acceleration_rad_s2':None,'arms':{'left':{'dof_names':['fr3_joint'+str(i) for i in range(1,8)]+['fr3_finger_joint1','fr3_finger_joint2']},'right':{'dof_names':['fr3_joint'+str(i) for i in range(1,8)]+['fr3_finger_joint1','fr3_finger_joint2']}}}
    cases=[]
    for name,mut,reason in [('wrong_mapping',lambda x:x['arms']['right'].__setitem__('dof_names',['fr3_joint'+str(i) for i in range(1,8)]+['bad','bad']),'WRONG_DOF_MAPPING:right'),('async_reference',lambda x:x.update(sim_time_s=-1.),'ASYNC_OR_OUT_OF_ORDER_REFERENCE'),('stale_state',lambda x:x.update(sim_time_s=float('nan')),'STALE_OR_INVALID_STATE'),('wrong_cube',lambda x:x.update(cube_id='Cube_02'),'WRONG_CUBE_ID'),('unsupported_acceleration',lambda x:x.update(acceleration_rad_s2=[0.]*14),'UNSUPPORTED_ACCELERATION_REFERENCE')]:
        x={'sim_time_s':base['sim_time_s'],'cube_id':base['cube_id'],'acceleration_rad_s2':base['acceleration_rad_s2'],'arms':{s:dict(base['arms'][s]) for s in ('left','right')}}; mut(x); result=validate_reference_controls([base,x]); cases.append({'id':name,'passed':reason in result['reasons'],'reasons':result['reasons']})
    return cases
