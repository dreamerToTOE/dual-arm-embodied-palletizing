from __future__ import annotations
import json
from pathlib import Path
HERE=Path(__file__).resolve().parent

def main():
    robust=json.loads((HERE/'evidence/robustness_replay.json').read_text())
    safety=json.loads((HERE/'evidence/continuous_safety_audit.json').read_text())
    evald=json.loads((HERE/'evidence/r12_baseline/evaluation.json').read_text())['cells']
    cells={}
    for c in ('V10-A','V10-B','V10-C'):
        det=robust['cells'][c]['deterministic_repetition']
        worst=robust['cells'][c]['combined_extremes']['worst_case']
        cells[c]={
            'frozen_discrete_gate': all(evald[c]['accuracy'][k]['pass'] for k in ('virtual_object_tracking_peak_m','relative_tcp_translation_peak_m')),
            'deterministic_repetition_20_pass': det['all_acceptance_pass'] and det['identical_record_fingerprint'],
            'bounded_perturbation_envelope_pass': robust['cells'][c]['combined_extremes']['all_acceptance_pass'],
            'bounded_perturbation_worst_peak_mm': worst['object_peak_mm'],
            'bounded_perturbation_worst_over_limit_count': worst['object_over_limit_count'],
        }
    review={
      'schema':'TASK28_R13_PHASE_ADMISSION_REVIEW_V1',
      'baseline_head':'52d07e6a9c4ac482db4887483cbf8ab0cbf510d1',
      'controller_frozen':True,
      'cells':cells,
      'continuous_safety':{
        'status':safety['status'],
        'sampled_candidate_only':True,
        'collision_continuous_proof':False,
        'closed_chain_continuous_proof':False,
        'direct_negative_present':True,
        'unresolved_interval_counts':[r['unresolved_interval_count'] for r in safety['depth_runs']],
      },
      'V10-D':{
        'preconditions_discrete_A_B_C':True,
        'preconditions_deterministic_repetition':True,
        'preconditions_bounded_perturbation_envelope':False,
        'continuous_safety_required':False,
        'execution_authorized':False,
        'status':'BLOCKED_R13_SCOPE_NO_EXECUTION_AND_ROBUSTNESS_ENVELOPE_NOT_ESTABLISHED'
      },
      'FORMAL':{
        'preconditions_discrete_A_B_C':True,
        'preconditions_deterministic_repetition':True,
        'preconditions_bounded_perturbation_envelope':False,
        'continuous_safety_required':False,
        'execution_authorized':False,
        'status':'BLOCKED_R13_SCOPE_NO_EXECUTION_AND_ROBUSTNESS_ENVELOPE_NOT_ESTABLISHED'
      },
      'next_phase_auto_start':False,
      'physical_execution':False,
    }
    (HERE/'evidence/phase_admission_review.json').write_text(json.dumps(review,indent=2,sort_keys=True)+'\n')
    print(json.dumps(review,indent=2))
if __name__=='__main__': main()
