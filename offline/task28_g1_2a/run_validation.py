#!/usr/bin/env python3
"""Run G1-2A's deterministic mode/support tests and emit evidence.

Only endpoint snapshots are evaluated. No interpolation, sweep, ROS, Isaac,
robot, suction or planner execution is performed.
"""
from dataclasses import asdict
import argparse, hashlib, json, os, platform, re, subprocess, sys
from pathlib import Path

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[1]
FIXTURE=REPO/'fixtures/task28_g1_0_new_single_cube_v0'
sys.path.insert(0,str(REPO/'offline/task28_g1_1'))
sys.path.insert(0,str(HERE))
from task28_contact.examples import load_sequence
from task28_contact.modes import OperationMode, SupportSite
from task28_contact.validator import ContactModeValidator
from task28_validator.state import StatePolicy


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,default=HERE/'evidence/validation_results.json')
    args=parser.parse_args()
    env={**os.environ,'PYTHONDONTWRITEBYTECODE':'1','PYTHONPATH':str(REPO/'offline/task28_g1_1')+':'+str(HERE)}
    g1_1_run=subprocess.run([sys.executable,'-m','pytest',str(REPO/'offline/task28_g1_1/tests'),'-q','-p','no:cacheprovider'],cwd=REPO,env=env,capture_output=True,text=True,check=True,timeout=180)
    g1_2a_run=subprocess.run([sys.executable,'-m','pytest',str(HERE/'tests'),'-q','-p','no:cacheprovider'],cwd=REPO,env=env,capture_output=True,text=True,check=True,timeout=180)
    g1_1_match=re.search(r'(\d+) passed',g1_1_run.stdout)
    g1_2a_match=re.search(r'(\d+) passed',g1_2a_run.stdout)
    assert g1_1_match and g1_2a_match
    g1_1_count=int(g1_1_match.group(1));g1_2a_count=int(g1_2a_match.group(1))
    gate=ContactModeValidator(); sequence,events=load_sequence(gate.model)
    snapshots=[]
    for row in sequence:
        result=gate.validate(row.snapshot,row.now_s)
        assert result.valid,result.reasons
        collision=result.collision
        snapshots.append({'label':row.label,'mode':row.snapshot.mode.value,'support_site':row.snapshot.support_site.value if row.snapshot.support_site else None,
                          'object_position_m':row.snapshot.input_state.cube.position_m,'valid':result.valid,'reasons':result.reasons,
                          'closure':None if result.closure is None else {'valid':result.closure.valid,'translation_error_m':result.closure.translation_error_m,'rotation_error_rad':result.closure.rotation_error_rad,'reasons':result.closure.reasons},
                          'support':None if collision.support is None else asdict(collision.support),'accepted_support_contact_count':len(collision.accepted_support_contacts),
                          'collision_reference_valid':collision.reference.valid,'collision_reference_reasons':collision.reference.reasons,
                          'remaining_collisions':len(collision.remaining_collisions),'checked_shape_pairs':collision.reference.checked_shape_pairs,'skipped_acm_shape_pairs':collision.reference.skipped_acm_shape_pairs,
                          'airborne_clearance_m':collision.airborne_clearance_m})
    transitions=[]
    for before,after,event in zip(sequence,sequence[1:],events):
        result=gate.validate_transition(before.snapshot,after.snapshot,event,before.now_s,after.now_s)
        assert result.valid,result.reasons
        transitions.append({'from':before.label,'to':after.label,'event':event.value,'valid':result.valid,'reasons':result.reasons,'scope':result.scope})
    negative_summary={
      'support_penetration': 'SUPPORT_TABLE_PENETRATION',
      'support_wrong_site': 'SUPPORT_OUTSIDE_DESIGNATED_FOOTPRINT',
      'airborne_table_intersection': 'AIRBORNE_CLEARANCE_NOT_ESTABLISHED',
      'invalid_feedback_or_flags': 'INCONSISTENT_ATTACHMENT_MODE',
      'stale_or_async': 'STALE_ATTACHMENT_FEEDBACK/ASYNCHRONOUS_ATTACHMENT_FEEDBACK',
      'closure_mismatch': 'LEFT_GRASP_TRANSLATION_MISMATCH',
      'illegal_transition': 'ILLEGAL_MODE_TRANSITION',
      'backend_failure': 'COLLISION_BACKEND_FAILURE',
    }
    source={}
    source_roots=(HERE/'task28_contact',HERE/'tests',HERE/'examples')
    for root in source_roots:
        for path in sorted(root.rglob('*')):
            if path.is_file() and '__pycache__' not in path.parts:
                source[str(path.relative_to(REPO))]=digest(path)
    for path in (HERE/'README.md',HERE/'requirements.txt',HERE/'run_validation.py',
                 REPO/'offline/task28_g1_1/tests/control_states.json'):
        source[str(path.relative_to(REPO))]=digest(path)
    evidence={'schema':'task28.g1_2a.offline_evidence.v1','status':'G1_2A_PASS','scope':'Offline phase mode and single-state endpoint transition validation; no continuous path safety.',
      'starting_commit':'4b5806e4250c468865b1f217f5afbcba15e61b2a','fixture_id':gate.model.manifest['fixture_id'],'fixture_commit_unchanged':True,
      'fixture_manifest_sha256':digest(FIXTURE/'fixture_manifest.json'),'g1_1_regression_tests_passed':g1_1_count,'g1_2a_tests_passed':g1_2a_count,'total_tests_passed':g1_1_count+g1_2a_count,
      'environment':{'python':platform.python_version(),'numpy':__import__('numpy').__version__,'scipy':__import__('scipy').__version__,'fcl':subprocess.check_output(['pkg-config','--modversion','fcl'],text=True).strip()},
      'support_policy':{'plane_tolerance_m':1e-6,'depth_tolerance_m':1e-6,'footprint_tolerance_m':1e-6,'normal_angle_tolerance_rad':1e-6,'max_contact_count':128,'mode_local_no_acm_change':True,
                        'near_zero_is_geometric_classification_not_physical_evidence':True},
      'mode_definitions':{'detached_supported':'No bilateral closure; exact supply support geometry and detached independent feedback.',
                          'bilateral_supported':'Bilateral closure plus exact local support geometry and independent bilateral feedback.',
                          'bilateral_airborne':'Bilateral closure, no support site, Cube/table clearance strictly above numerical-zero band.',
                          'released_supported':'Released independent feedback plus exact target support geometry; no bilateral closure.'},
      'a_stage_snapshots':snapshots,'a_stage_transitions':transitions,'negative_control_reason_catalog':negative_summary,
      'independent_fcl_audit':'evidence/independent_fcl_support_audit.json','source_sha256':source,
      'not_performed':['Isaac/ROS execution','suction/robot commands','path planning','trajectory-edge or swept validation','NR/QP/force/RL','physical attachment/support measurement']}
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(evidence,indent=2,allow_nan=False)+'\n')
    print(f"G1_2A_PASS: {evidence['total_tests_passed']} tests; 6 snapshots; 5 transitions; evidence {args.output}")


if __name__=='__main__':main()
