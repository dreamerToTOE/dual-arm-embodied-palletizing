"""Assemble final status from independent evaluations; no simulation imports."""
import hashlib,json
from pathlib import Path
HERE=Path(__file__).resolve().parent

def write(name,value):
 (HERE/'evidence'/name).write_text(json.dumps(value,indent=2,sort_keys=True)+'\n')

def main():
 e=json.loads((HERE/'evidence/evaluation.json').read_text());p=json.loads((HERE/'evidence/process_isolation_audit.json').read_text());reg=json.loads((HERE/'evidence/regressions/summary.json').read_text());r7=json.loads((HERE/'evidence/regressions/r7_focused.json').read_text());focus=(HERE/'evidence/focused_final.log').read_text()
 assert reg['total_passed']==1015 and reg['status']=='PASS' and r7['passed']==4 and r7['pass']
 assert '15 passed' in focus
 for n in ('D3','D4','FORMAL','MOVING'):assert e['cells'][n]['pass'] and p[n]['verified'] and p[n]['normal_exit']
 assert e['cells']['D3']['sides']==e['cells']['D4']['sides']
 preservation=json.loads((HERE/'evidence/frozen_preservation_verification.json').read_text());assert preservation['status']=='PASS'
 statuses={'ORIGINAL_SCENE_PRESERVED':'PASS','ISAAC_PROCESS_ISOLATION_VERIFIED':'PASS_RECOVERED_D3_D4_FORMAL_MOVING','R7_DRIFT_CAUSE_STATUS':'WEAK_GAIN_AND_INHERITED_VELOCITY_SUPPORTED_COMBINED_RECOVERY','INITIAL_STATE_AND_OWNERSHIP_STATUS':'SOURCE_VALID_NONZERO_NATIVE_DQ_C2_TRANSITION','DYNAMICS_MODEL_SCOPE':'FROZEN_MEASURED_M_GAIN_METRIC_NOT_CERTIFIED_INVERSE_DYNAMICS','FEEDBACK_DIAGNOSTIC_STABILIZATION_STATUS':'PASS_D3_D4_INDEPENDENT_REPEAT','FORMAL_STATIC_HOLD_STATUS':'PASS_ONE_60_STEP_RUN','SHORT_TRACKING_STATUS':'PASS_ONE_210_STEP_IMPEDANCE_RUN','COMPUTED_TORQUE_STATUS':'SKIPPED_APPLICABLE_MODEL_NOT_INDEPENDENTLY_QUALIFIED','COOPERATIVE_QP_STATUS':'OFFLINE_READINESS_EVALUATED_NOT_READY','CUBE_D6_UNTESTED':True}
 write('final_status.json',statuses)
 write('computed_torque_skip.json',{'status':statuses['COMPUTED_TORQUE_STATUS'],'reason':'Observed SPD M/c/g and successful small impedance tracking do not independently certify mounted-tool inverse dynamics or constraint terms. End-step dq derivative is not certified acceleration.','commands':0})
 write('focused_tests.json',{'passed':15,'status':'PASS','command':'PYTHONDONTWRITEBYTECODE=1 python3 -m pytest offline/task28_p5_qp_6_r8/tests -q -p no:cacheprovider --import-mode=importlib','log_sha256':hashlib.sha256(focus.encode()).hexdigest(),'historical_1015_separate':True,'R7_4_separate':True})
 write('summary.json',{'status':'G2_P5_QP_6_R8_STATIC_AND_SHORT_TRACKING_PASS_COOPERATIVE_NOT_READY','statuses':statuses,'cells':{n:{'verdict':c.get('verdict'),'commands_per_arm':c.get('commands',0),'robot_writes':c.get('writes',0),'duration_s':c.get('actual_physics_duration_s'),'sides':c.get('sides',{})} for n,c in e['cells'].items()},'total_bilateral_command_intervals':sum(c.get('commands',0) for c in e['cells'].values()),'total_full_nine_DOF_robot_writes':sum(c.get('writes',0) for c in e['cells'].values()),'test_counts':{'previous':1015,'R7_separate':4,'R8_focused':15},'repeat_D3_D4_metrics_exact':True,'failed_initialization_cells':['D1','D2'],'no_development_cell_retry':True,'formal_runs':1,'moving_runs':1,'stop_after_R8':True})
 print(json.dumps(statuses,indent=2))
if __name__=='__main__':main()
