"""Small R14 G2 closeout sensitivity check using the unchanged R13 CPU plant.

This performs fresh QP/controller computations, never Isaac or robot commands.
The approximate plant was falsified for the C margin in R13. Its observations
cannot establish perturbed-runtime acceptance, robustness, or contact safety.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import gzip
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import time

import numpy as np
import scipy

from offline.task28_p5_qp_6_r13 import closed_loop_replay as replay

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
EVIDENCE = HERE / 'evidence'
R13_HEAD = '72fbb8d5ea09ac9edb0a3869a07bed01e243741c'
CASES = ('V10-B', 'V10-C')
SCENARIOS = ('nominal', 'initial_ulp_plus',
             'observed_mass_delta_minus', 'observed_mass_delta_plus')
MANIFEST = EVIDENCE / 'g2_preregistration.json'
RESULTS = EVIDENCE / 'g2_results.json'


def read(path):
    return json.loads(Path(path).read_text())


def validate_plan(plan):
    """Reject unregistered scenarios/parameters before any QP is evaluated."""
    if plan['cases'] != list(CASES) or plan['scenarios'] != list(SCENARIOS):
        raise ValueError('R14_SMALL_PLAN_CHANGED')
    if plan['episodes_per_case_scenario'] != 1 or plan['planned_episodes'] != 8:
        raise ValueError('R14_NO_LARGE_SWEEP')
    if plan['controller'] != {'feedback_gain_scale': .75, 'tau_s': .10, 'changed': False}:
        raise ValueError('FROZEN_CONTROLLER_CHANGED')
    if plan['acceptance'] != {'object_peak_m': .00025, 'relative_translation_peak_m': .00015}:
        raise ValueError('FROZEN_THRESHOLDS_CHANGED')
    source = read(replay.EVIDENCE / 'closed_loop_manifest.json')
    if plan['perturbations'] != {case: source['perturbations'][case] for case in CASES}:
        raise ValueError('PERTURBATIONS_NOT_FROZEN_R13_VALUES')
    for name, expected in plan['sources_sha256'].items():
        if replay.digest(ROOT / name) != expected:
            raise ValueError('SOURCE_HASH_CHANGED:' + name)
    # RobotModel checks every frozen fixture byte against its bound manifest.
    replay.RobotModel()


def freeze():
    if MANIFEST.exists() or RESULTS.exists():
        raise FileExistsError('R14 plan/results already exist; do not retune after results')
    head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    if head != R13_HEAD:
        raise ValueError('UNEXPECTED_R14_STARTING_HEAD')
    source = read(replay.EVIDENCE / 'closed_loop_manifest.json')
    hashes = dict(source['sources_sha256'])
    for path in (Path(__file__), replay.EVIDENCE / 'closed_loop_manifest.json',
                 replay.EVIDENCE / 'closed_loop_model_validation.json',
                 ROOT / 'offline/task28_g1_1/task28_validator/fixture_binding.json',
                 replay.RobotModel().fixture_dir / 'fixture_manifest.json'):
        hashes[str(path.relative_to(ROOT))] = replay.digest(path)
    plan = {
        'schema': 'TASK28_R14_G2_SMALL_PREREGISTRATION_V1',
        'baseline_head': head,
        'frozen_utc': datetime.now(timezone.utc).isoformat(),
        'cases': list(CASES), 'scenarios': list(SCENARIOS),
        'episodes_per_case_scenario': 1, 'planned_episodes': 8,
        'perturbed_episodes': 6,
        'controller': source['controller'], 'acceptance': source['acceptance'],
        'perturbations': {case: source['perturbations'][case] for case in CASES},
        'selection_before_r14_results': True,
        'selection_rationale': {
            'nominal': 'One freshly reset comparator for each B/C case; no A rerun or repeatability campaign.',
            'initial_ulp_plus': 'Positive adjacent float32 value chosen by fixed signed convention for both cases, not by observed performance. Numerical-resolution sensitivity only; not an initial-state uncertainty bound.',
            'observed_mass_delta_minus': 'Negative largest archived spectral-norm M departure; paired sign prevents a one-sided mass sensitivity claim.',
            'observed_mass_delta_plus': 'Positive same archived M departure; SPD checks preserved. This is posture variation, not calibrated model uncertainty.',
            'omissions': 'No timestep sweep or delayed-feedback experiment. R13 timestamp deviation is not a scheduler-jitter bound. This limited closeout is not a coverage claim.',
        },
        'plant': source['plant'], 'safety_scope': source['safety_scope'],
        'model_qualification': source['model_qualification'],
        'sources_sha256': hashes,
        'environment': {'python': platform.python_version(), 'numpy': np.__version__,
                        'scipy': scipy.__version__},
        'continuous_approach_safety': 'CONTINUOUS_SAFETY_NOT_ESTABLISHED',
        'physical_execution': False, 'robot_command_writes': 0,
    }
    validate_plan(plan)
    replay.write(MANIFEST, plan)
    print('FROZEN', replay.digest(MANIFEST), flush=True)


def observation(summary):
    """An approximate-plant observation is never a runtime/robustness verdict."""
    complete = summary['frames'] == 210 and summary['plant_status'] == 'COMPLETE'
    obj = len(summary['object_exceedance_frames'])
    rel = len(summary['relative_exceedance_frames'])
    return {
        'approximate_model_observation': ('INCOMPLETE_OR_GATE_REJECTED' if not complete
            else 'TRACKING_THRESHOLD_EXCEEDED' if obj or rel
            else 'NO_TRACKING_THRESHOLD_EXCEEDANCE_OBSERVED'),
        'object_exceedance_count': obj, 'relative_exceedance_count': rel,
        'runtime_acceptance_claim': False, 'robustness_status': 'NOT_ESTABLISHED',
        'execution_authorized': False,
    }


def run():
    if RESULTS.exists():
        raise FileExistsError('R14 results already exist; preserve the recorded experiment')
    plan = read(MANIFEST)
    validate_plan(plan)
    plan_hash = replay.digest(MANIFEST)
    episodes = []
    qualification = {}
    trace_dir = EVIDENCE / 'g2_traces'
    trace_dir.mkdir(parents=True, exist_ok=True)
    # Both nominal qualification checks precede every sensitivity episode.
    jobs = [(case, 'nominal') for case in CASES]
    jobs += [(case, scenario) for case in CASES for scenario in SCENARIOS[1:]]
    for case, scenario in jobs:
        validate_plan(plan)
        if replay.digest(MANIFEST) != plan_hash:
            raise ValueError('PREREGISTRATION_CHANGED_DURING_RUN')
        started = time.monotonic()
        summary, records = replay.simulate_episode(case, scenario, plan)
        data = replay.encoded(records)
        sha = hashlib.sha256(data).hexdigest()
        path = trace_dir / (case + '_' + scenario + '.json.gz')
        with path.open('xb') as raw:
            with gzip.GzipFile(fileobj=raw, mode='wb', mtime=0) as stream:
                stream.write(data)
        row = {
            'case': case, 'scenario': scenario,
            'independent_state_reset': True, 'fresh_qp_and_controller_recomputation': True,
            'elapsed_s': time.monotonic() - started,
            'trace': str(path.relative_to(ROOT)),
            'trace_sha256_uncompressed': sha, 'trace_sha256_gzip': replay.digest(path),
            'computed_qp_calls': summary['qp_solves'],
            'summary': summary, **observation(summary),
        }
        episodes.append(row)
        if scenario == 'nominal':
            delta = summary['archive_comparison']['same_index_tcp_discrepancy_peak_m']
            margin = plan['model_qualification']['v10c_remaining_margin_m']
            qualification[case] = {
                'same_index_tcp_discrepancy_peak_m': delta,
                'v10c_remaining_margin_m': margin,
                'discrepancy_to_margin_ratio': delta / margin if delta is not None else None,
                'status': ('MODEL_NOT_QUALIFIED_FOR_MARGIN' if delta is None or
                           summary['frames'] != 210 or delta > margin else
                           'NECESSARY_DISCREPANCY_TEST_ONLY_PASSED'),
                'independent_plant_validation': False,
            }
        print(case, scenario, json.dumps({
            'peak_mm': 1000 * summary['tracking_peak_m'],
            'relative_mm': 1000 * summary['relative_tcp_max_m'],
            'observation': row['approximate_model_observation']}), flush=True)
    result = {
        'schema': 'TASK28_R14_G2_SMALL_RESULTS_V1', 'baseline_head': R13_HEAD,
        'preregistration_sha256': plan_hash, 'controller_changed': False,
        'episode_count': len(episodes), 'perturbed_episode_count': 6,
        'computed_qp_calls': sum(row['computed_qp_calls'] for row in episodes),
        'episodes': episodes, 'nominal_model_qualification': qualification,
        'robustness_status': 'NOT_ESTABLISHED', 'runtime_acceptance_claim': False,
        'g2_closeout': 'FROZEN_R12_R13_NOMINAL_RESULTS_RETAINED_NO_NEW_ROBUSTNESS_CLAIM',
        'independent_g3_offline_research_blocked_by_new_perturbations': False,
        'risks': [
            'C nominal Isaac margin is 0.002383426963275 mm; CPU model discrepancy exceeds that margin.',
            'No threshold exceedance in these few approximate-model episodes does not bound untested disturbances or Isaac results.',
            'Initial ULP is numerical resolution, and observed M departure is posture sensitivity; neither is calibrated physical uncertainty.',
            'Archive-driven g cancellation, omitted constraints/friction/finger coupling, and TGS velocity semantics limit transfer.',
            'No contact or suction observation was tested. Original Cube approach continuous safety remains unestablished.',
        ],
        'continuous_approach_safety': 'CONTINUOUS_SAFETY_NOT_ESTABLISHED',
        'isaac_processes_launched': 0, 'robot_command_writes': 0,
        'cube_contact_or_motion_commands': 0, 'suction_or_D6_commands': 0,
    }
    validate_plan(plan)
    replay.write(RESULTS, result)
    print('COMPLETE', len(episodes), result['computed_qp_calls'], flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--freeze', action='store_true')
    parser.add_argument('--run', action='store_true')
    args = parser.parse_args()
    if args.freeze:
        freeze()
    if args.run:
        run()


if __name__ == '__main__':
    main()
