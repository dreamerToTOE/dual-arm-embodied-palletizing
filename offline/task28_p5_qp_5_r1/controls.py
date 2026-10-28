from __future__ import annotations
import numpy as np

def validate_negative(case):
    reasons=[]
    if case.get('joint_sample_time_s') != case.get('object_sample_time_s'): reasons.append('REFERENCE_TIME_MISMATCH')
    if case.get('physics_step_offset_s') not in (0.0,1/120): reasons.append('ONE_STEP_SAMPLING_OFFSET')
    if case.get('interval_step_count') != 3: reasons.append('INCORRECT_20MS_STEP_CONVERSION')
    if case.get('side')!='left' or case.get('dof_names')!=['fr3_joint'+str(i) for i in range(1,8)]+['fr3_finger_joint1','fr3_finger_joint2']: reasons.append('WRONG_SIDE_OR_DOF_MAPPING')
    if case.get('clock_discontinuous'): reasons.append('STALE_OR_DISCONTINUOUS_CLOCK')
    if case.get('closure_translation_m',0)>1e-4: reasons.append('POST_STEP_CLOSURE_FAILURE')
    if case.get('position_error_rad',0)>0.05: reasons.append('TRACKING_ERROR_LIMIT')
    if case.get('saturated'): reasons.append('COMMAND_SATURATION')
    if case.get('rate_violation'): reasons.append('EFFORT_RATE_VIOLATION')
    if case.get('continued_after_failure'): reasons.append('ILLEGAL_CONTINUATION_AFTER_FAILURE')
    if case.get('qdd_status')!='UNAVAILABLE_NO_VALID_TIME_PARAMETERIZATION': reasons.append('UNSUPPORTED_ACCELERATION_PROMOTION')
    return sorted(set(reasons))

def negative_controls():
    base={'joint_sample_time_s':0.0,'object_sample_time_s':0.0,'physics_step_offset_s':0.0,'interval_step_count':3,'side':'left','dof_names':['fr3_joint'+str(i) for i in range(1,8)]+['fr3_finger_joint1','fr3_finger_joint2'],'clock_discontinuous':False,'closure_translation_m':0.,'position_error_rad':0.,'saturated':False,'rate_violation':False,'continued_after_failure':False,'qdd_status':'UNAVAILABLE_NO_VALID_TIME_PARAMETERIZATION'}
    cases=[]
    for name,key,value,reason in [('time_mismatch','object_sample_time_s',0.008333333333333,'REFERENCE_TIME_MISMATCH'),('one_step_offset','physics_step_offset_s',0.02,'ONE_STEP_SAMPLING_OFFSET'),('wrong_conversion','interval_step_count',2,'INCORRECT_20MS_STEP_CONVERSION'),('wrong_mapping','dof_names',['bad']*9,'WRONG_SIDE_OR_DOF_MAPPING'),('stale_clock','clock_discontinuous',True,'STALE_OR_DISCONTINUOUS_CLOCK'),('closure','closure_translation_m',0.00011,'POST_STEP_CLOSURE_FAILURE'),('tracking','position_error_rad',0.06,'TRACKING_ERROR_LIMIT'),('saturation','saturated',True,'COMMAND_SATURATION'),('rate','rate_violation',True,'EFFORT_RATE_VIOLATION'),('continuation','continued_after_failure',True,'ILLEGAL_CONTINUATION_AFTER_FAILURE'),('qdd','qdd_status','VALIDATED','UNSUPPORTED_ACCELERATION_PROMOTION')]:
        x=dict(base); x[key]=value; reasons=validate_negative(x); cases.append({'id':name,'passed':reason in reasons,'reasons':reasons})
    return cases
