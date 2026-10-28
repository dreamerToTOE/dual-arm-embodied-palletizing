"""Reproducible archive conversion and CPU recheck; refuses existing output."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
from .reference import ReferenceContract, BINDING, load_archive, build_reference_sequence, source_hashes, validate_sequence
from .controls import negative_controls
from .recheck import recheck
from offline.task28_p5_qp_1.driver import _jsonable


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    source=load_archive(); refs=build_reference_sequence(source['positive'])
    cases=negative_controls(); independent=recheck()
    valid=validate_sequence(refs)
    if not all(c['passed'] for c in cases) or not independent['pass']:
        raise ValueError('QP4_EVIDENCE_ACCEPTANCE_FAILED')
    data={
        'reference_contract.json':{'schema':'task28.g2_p5_qp_4.contract.v1','contract':asdict(ReferenceContract()),
                                  'scope':'EXACT_REGISTERED_FIVE_STEP_ARCHIVE_ONLY','source_binding':json.loads(BINDING.read_text()),
                                  'interpolation':'NONE_PROVIDED','acceleration_rad_s2':None,
                                  'missing_future_time_parameterization':True,'execution_authorized':False},
        'accepted_reference_sequence.json':{'schema':'task28.g2_p5_qp_4.sequence.v1','validation':valid,
                                           'references':[asdict(r) for r in refs],'execution_authorized':False},
        'negative_controls.json':{'cases':cases,'passed':len(cases),'scope':'CONTRACT_FAULT_INJECTION_PLUS_ORIGINAL_ARCHIVED_NEGATIVE'},
        'original_negative.json':source['negative_payload_table'],
        'independent_recheck.json':independent,
        'source_hashes.json':source_hashes(),
    }
    rows=independent['positive_steps']
    summary={'status':'G2_P5_QP_4_TORQUE_REFERENCE_PASS_OFFLINE','reference_count':len(refs),'negative_count':len(cases),
             'exact_q_qdot_copy':all(r['exact_q_copy'] and r['exact_qdot_copy'] for r in rows),
             'max_euler_error_rad':max(r['euler_max_error_rad'] for r in rows),
             'max_affine_equality_residual':max(r['hard_affine_residual_inf'] for r in rows),
             'max_tracking_translation_m_s':max(r['tracking_translation_error_m_s'] for r in rows),
             'max_tracking_rotation_rad_s':max(r['tracking_rotation_error_rad_s'] for r in rows),
             'minimum_position_margin_rad':min(r['minimum_position_margin_rad'] for r in rows),
             'minimum_velocity_margin_rad_s':min(r['minimum_velocity_margin_rad_s'] for r in rows),
             'max_closure_translation_m':max(r['predicted_state']['closure']['translation_error_m'] for r in rows),
             'max_closure_rotation_rad':max(r['predicted_state']['closure']['rotation_error_rad'] for r in rows),
             'sampled_edge_evaluations':sum(r['sampled_edge']['evaluated_sample_count'] for r in rows),
             'source_lineage_exact':independent['source_lineage_exact'],'negative_payload_table_rejected':independent['negative_payload_table_rejected'],
             'continuous_safety':'NOT_CERTIFIED_FINITE_SAMPLING_ONLY','execution_authorized':False}
    data['summary.json']=summary
    for name,value in data.items():
        (args.output/name).write_text(json.dumps(_jsonable(value),indent=2,sort_keys=True,allow_nan=False)+'\n')
    print(json.dumps(summary,sort_keys=True))
    return 0

if __name__=='__main__':
    raise SystemExit(main())
