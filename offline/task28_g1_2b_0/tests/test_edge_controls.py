"""Actual frozen FK/FCL trajectories: endpoint validity precedes each failure."""
from dataclasses import replace
import numpy as np
import pytest
from task28_edge.checker import SampledEdgeValidator
from task28_edge.experiments import controls,a_stage
from task28_edge.types import EdgeStatus as S,SamplingPolicy,Interpolation,JointInterpolation,Motion


@pytest.fixture(scope='module')
def checker():return SampledEdgeValidator()


@pytest.fixture(scope='module')
def cases(checker):return controls(checker.model)


@pytest.fixture(scope='module')
def results(checker,cases):
    policy=SamplingPolicy(max_dt_s=.25,max_joint_step_rad=.2,max_object_step_m=.01)
    return {key:checker.validate(t,policy) for key,t in cases.items()}


def test_endpoint_only_is_explicitly_distinct(checker,cases):
    for t in cases.values():
        r=checker.validate(t,endpoint_only=True)
        assert r.status is S.ENDPOINT_VALID
        assert r.endpoints_valid and all(row['valid'] for row in r.endpoint_results)
        assert r.continuous_safety=='NOT_ESTABLISHED_FINITE_SAMPLING_ONLY'


def test_positive_all_sample_gates_and_payload_present(results):
    r=results['positive_hold']
    assert r.status is S.SAMPLED_EDGE_VALID and r.evaluated_sample_count==3
    assert r.maxima['max_closure_translation_m']<1e-4
    for row in r.samples:
        assert row['valid'] and row['closure']['valid']
        assert row['collision']['reference']['checked_shape_pairs']==414
        assert row['collision']['reference']['skipped_acm_shape_pairs']==40
        assert len(row['independent_fk'])==20
        assert row['geometry']['clearance_m']==pytest.approx(.004)


def test_A_endpoint_valid_joint_linear_intermediate_closure_failure(results):
    r=results['A_closure']
    assert r.endpoints_valid and r.status is S.SAMPLED_EDGE_REJECTED
    middle=next(row for row in r.samples if row['time_s']==.5)
    assert not middle['closure']['valid']
    assert r.maxima['max_closure_translation_m']>.01
    assert r.maxima['max_closure_rotation_rad']>.01
    assert any('GRASP_TRANSLATION' in reason for reason in middle['reasons'])
    assert middle['collision']['reference']['checked_shape_pairs']==414


def test_B_endpoint_valid_actual_fk_closed_chain_payload_table_collision(results):
    r=results['B_payload_collision']
    assert r.endpoints_valid and r.status is S.SAMPLED_EDGE_REJECTED
    middle=next(row for row in r.samples if row['time_s']==1.)
    assert middle['closure']['valid']
    assert middle['minimum_joint_limit_margin_rad']>0
    collisions=middle['collision']['remaining_collisions']
    assert any({record['owner_a'],record['owner_b']}=={'Cube_01','task28_table'} for record in collisions)
    assert max(contact['depth_m'] for record in collisions for contact in record['contacts'])==pytest.approx(.02)


def test_C_endpoint_valid_nonlinear_hermite_limit_overshoot(results):
    r=results['C_limit']
    assert r.endpoints_valid and r.status is S.SAMPLED_EDGE_REJECTED
    middle=next(row for row in r.samples if row['time_s']==.5)
    assert middle['minimum_joint_limit_margin_rad']==pytest.approx(-.01)
    assert 'LEFT_JOINT_LIMIT_VIOLATION' in middle['reasons']
    # Full collision/FK diagnostics are retained for this rejected algebraic
    # configuration; out-of-range angles never acquire execution permission.
    assert middle['collision']['reference']['checked_shape_pairs']==414
    assert len(middle['independent_fk'])==20


def test_C_control_cannot_be_falsely_described_as_linear_overshoot(checker,cases):
    t=replace(cases['C_limit'],interpolation=Interpolation())
    r=checker.validate(t)
    assert r.status is S.SAMPLED_EDGE_VALID
    assert r.maxima['minimum_joint_limit_margin_rad']>0


def test_D_every_geometric_sample_valid_but_interior_mode_change_rejected(results):
    r=results['D_mode']
    assert r.endpoints_valid and all(row['valid'] for row in r.samples)
    assert r.status is S.SAMPLED_EDGE_REJECTED
    assert 'ILLEGAL_INTERIOR_CONTACT_MODE_TRANSITION' in r.reasons
    assert 'ATTACHMENT_CHANGED_WITHOUT_EXPLICIT_EVENT' in r.reasons
    assert any(location.get('waypoint_index')==1 for location in r.rejection_locations)


def test_budget_exhaustion_preserves_endpoint_result_and_is_inconclusive(checker,cases):
    r=checker.validate(cases['A_closure'],SamplingPolicy(max_samples=2))
    assert r.endpoints_valid and r.status is S.EDGE_INCONCLUSIVE
    assert r.reasons==('SAMPLE_BUDGET_EXCEEDED',)
    assert r.planned_sample_count>r.evaluated_sample_count==2


def test_all_7_representative_segments_sampled_and_temporal_context_valid(checker):
    trajectories=a_stage(checker)
    assert len(trajectories)==7
    for name,t in trajectories.items():
        r=checker.validate(t)
        if name=='vertical_placement':
            assert r.status is S.EDGE_INCONCLUSIVE and r.reasons==('TERMINAL_PHASE_NOT_CONFIRMED',)
            assert not r.resolution['temporal_end_confirmed']
        else:assert r.status is S.SAMPLED_EDGE_VALID,(name,r.reasons,r.rejection_locations[:1])
        assert r.endpoints_valid and r.evaluated_sample_count==r.planned_sample_count
        assert r.resolution['interpolated_points_used_for_temporal_confirmation']==0
        assert len(r.temporal_evidence)==len(t.temporal_prelude)+len(t.waypoints)
        assert all(e['result']['valid'] for e in r.temporal_evidence)
        if name=='release_boundary':
            assert r.resolution['temporal_end_phase']=='released_supported_confirmed'
            assert r.resolution['temporal_end_confirmed']
        for row in r.samples:
            assert row['collision']['reference']['checked_shape_pairs']==414
            assert row['minimum_joint_limit_margin_rad']>=0
            if row['closure']:assert row['closure']['valid']


def test_sample_density_cannot_manufacture_temporal_confirmation(checker):
    t=a_stage(checker)['grasp_boundary']
    one=checker.validate(t,SamplingPolicy(max_dt_s=.1))
    dense=checker.validate(t,SamplingPolicy(max_dt_s=.001))
    assert one.evaluated_sample_count<dense.evaluated_sample_count
    assert len(one.temporal_evidence)==len(dense.temporal_evidence)==3
    assert one.resolution['temporal_end_phase']==dense.resolution['temporal_end_phase']


def test_only_two_explicit_observations_cannot_finish_required_phase(checker):
    t=a_stage(checker)['grasp_boundary']
    r=checker.validate(replace(t,waypoints=t.waypoints[:2]),SamplingPolicy(max_dt_s=.001))
    assert r.endpoints_valid and all(row['valid'] for row in r.samples)
    assert r.evaluated_sample_count>3
    assert len(r.temporal_evidence)==2
    assert not r.resolution['temporal_end_confirmed']
    assert r.status is S.EDGE_INCONCLUSIVE
    assert r.reasons==('TERMINAL_PHASE_NOT_CONFIRMED',)


def test_incomplete_source_confirmation_cannot_be_repaired_by_dense_edge_samples(checker):
    t=a_stage(checker)['attachment_confirmation']
    r=checker.validate(replace(t,temporal_prelude=t.temporal_prelude[:1]),SamplingPolicy(max_dt_s=.001))
    assert r.endpoints_valid and all(row['valid'] for row in r.samples)
    assert r.status is S.SAMPLED_EDGE_REJECTED
    assert any('ATTACH_REQUIRES_CONFIRMED_DETACHED_SUPPORT' in reason for reason in r.reasons)


def test_placement_confirmation_requires_explicit_stationary_observations(checker):
    from task28_edge.experiments import placement_with_support_hold
    r=checker.validate(placement_with_support_hold(checker))
    assert r.status is S.SAMPLED_EDGE_VALID
    assert r.resolution['temporal_end_phase']=='bilateral_supported_confirmed'
    assert r.resolution['temporal_end_confirmed']
    assert r.temporal_evidence[-1]['result']['can_release']
