import json
from pathlib import Path
import pytest
from offline.task28_p5_qp_6_r5.source_gate import R5Gate,Rejected,torque_attempt
ROOT=Path(__file__).resolve().parents[1]; E=ROOT/'evidence'
def result(n):return json.loads((E/f'run_{n}'/'result.json').read_text())
def rows(n):
 p=E/f'run_{n}'/'frames.jsonl';return [json.loads(x) for x in p.read_text().splitlines()] if p.exists() else []

def test_preregistration_and_cells_are_frozen_before_run():
 p=json.loads((ROOT/'preregistration.json').read_text());assert p['created_before_new_isaac_experiment'] is True;assert set(p['cells'])==set('ABCDEF')
def test_original_cpu_cells_complete_and_write_free():
 for n in 'ABCE':
  d=result(n);assert d['status']=='BOUNDED_OBSERVATION_COMPLETE_PENDING_EVALUATION';assert d['explicit_physics_steps']==150;assert len(rows(n))==151;assert d['robot_command_writes']==0;assert d['suction_commands']==0
def test_c_flag_is_installed_and_nonbaseline_only():
 d=result('C');assert d['tgs_force_flag_supported'] is True;assert d['tgs_force_flag_before']=='False';assert d['tgs_force_flag_after_context']=='True';assert d['non_baseline_diagnostic'] is True;assert d['physics_after_context']['physxScene:enableExternalForcesEveryIteration']=='True'
def test_baseline_and_flag_show_the_preregistered_semantic_difference():
 s=json.loads((E/'summary.json').read_text());b=s['baseline_vs_flag'];assert b['baseline_tail_mean_rad_s']>0.002;assert b['flag_tail_mean_rad_s']<0.0004;assert b['reduction_ratio']>2
def test_a_and_e_reproduce_signed_motion_and_settle():
 for n in 'AE':
  rs=rows(n)
  for side in ('left','right'):
   ds=[x['derived'][side]['delta_q'][:7] for x in rs[1:]];flat=[v for row in ds for v in row];assert any(x>1e-4 for x in flat) and any(x<-1e-6 for x in flat);assert sum(max(abs(v) for v in row)>1e-4 for row in ds)>=90
def test_fk_and_collision_checks_hold_for_all_cpu_traces():
 for n in 'ABCE':
  for x in rows(n):
   assert x['collision']['unexpected']==[]
   for side in ('left','right'):
    assert x['arms'][side]['tcp_fk_error_m']<1e-3;assert x['arms'][side]['tcp_fk_error_rad']<2e-3
def test_b_and_e_same_backend_dynamic_control_is_corrobation_only():
 for n in 'BE':
  for x in rows(n):
   for side in ('left','right'):
    assert x['arms'][side]['dynamic_control_q']==x['arms'][side]['q'];assert x['arms'][side]['dynamic_control_dq']==x['arms'][side]['dq']
def test_solver_and_drive_parameters_are_recorded():
 d=result('B');assert d['physics_after_context']['physxScene:solverType']=='TGS';assert d['physics_after_context']['physxScene:minPositionIterationCount']=='1';assert d['physics_after_context']['physxScene:maxPositionIterationCount']=='255';a=rows('B')[0]['arms']['left'];assert a['drive_position_targets'][1]==pytest.approx(-0.7853981852531433);assert a['drive_kp'][0][0]>3e6
def test_gpu_diagnostic_is_explicitly_rejected_without_frames_or_commands():
 d=result('D');assert d['status']=='REJECTED';assert d['explicit_physics_steps']==0;assert d['recorded_frames']==0;assert 'CUDA error' in d['first_failure']['message'];assert d['robot_command_writes']==0
def test_negative_controls_are_all_fail_closed():
 d=json.loads((E/'negative_controls.json').read_text());assert d['status']=='ALL_R5_NEGATIVE_CONTROLS_REJECTED';assert len(d['cases'])==13;assert d['robot_command_writes']==0;assert all(x['outcome']=='REJECTED' and x['reason']==x['expected'] for x in d['cases'])
def test_latched_failure_blocks_actuation():
 g=R5Gate()
 with pytest.raises(Rejected,match='PRE'):g.reject('PRE')
 with pytest.raises(Rejected,match='LATCHED'):torque_attempt(g)
def test_r4_is_preserved_and_no_torque_cells_started():
 s=json.loads((E/'summary.json').read_text());assert s['r4_preserved'] is True;assert s['torque_commands']==0;st=json.loads((E/'experiment_status.json').read_text());assert st['STATE_SOURCE_VALID_FOR_CONTROL']=='PARTIAL';assert st['STATIC_TORQUE_HOLD']=='SKIPPED';assert st['CUBE_D6_NOT_TESTED'] is True
def test_source_binding_and_installed_probe_are_present():
 b=json.loads((ROOT/'source_binding.json').read_text());assert len(b['files'])>=7;p=json.loads((E/'installed_api_probe.json').read_text());assert p['physx_scene_api']['GetEnableExternalForcesEveryIterationAttr'] is True;assert p['r5_c_support'].startswith('stage')
