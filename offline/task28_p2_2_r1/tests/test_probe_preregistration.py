"""Structural checks for the frozen, independent preregistrations; no Isaac launch."""
import json
from pathlib import Path

BASE = Path(__file__).resolve().parents[2]
PREVIOUS = BASE / 'task28_p2_2/evidence/static_probe_preregistration.json'
NEW = BASE / 'task28_p2_2_r1/evidence'


def load(path):
    return json.loads(path.read_text(encoding='utf-8'))


def test_physical_setup_and_prior_thresholds_unchanged():
    old = load(PREVIOUS)
    for name in ('static_preregistration.json', 'sign_preregistration.json'):
        new = load(NEW / name)
        for key in ('fixture_usd_sha256', 'joint_names', 'base_world_m', 'physics_dt_s',
                    'gravity_m_s2', 'control', 'settle_steps_each_condition',
                    'observation_window_steps', 'thresholds', 'flags'):
            assert new[key] == old[key], key
        for key, value in new['configurations_rad'].items():
            assert value == old['configurations_rad'][key]
        assert new['max_steps'] == 2 * len(new['configurations_rad']) * len(new['loads']) * new['settle_steps_each_condition']


def test_six_independent_nonzero_wrench_components_and_opposite_controls():
    config = load(NEW / 'static_preregistration.json')
    loads = {item['id']: item for item in config['loads']}
    assert len(loads) == len(config['loads'])
    for i, name in enumerate(('fx', 'fy', 'fz', 'mx', 'my', 'mz')):
        case = loads['positive_' + name]
        vec = case['force_world_n'] + case['torque_world_nm']
        magnitude = 1.0 if i < 3 else 0.3
        assert vec == [magnitude if k == i else 0 for k in range(6)]
        assert case['offset_from_virtual_tcp_world_m'] == [0, 0, 0]
    assert 0.3 > 3 * config['thresholds']['max_moment_error_nm']
    for name in ('fx', 'fz', 'mz'):
        positive = loads['positive_' + name]
        negative = loads['negative_' + name]
        for component in ('force_world_n', 'torque_world_nm'):
            assert negative[component] == [-x for x in positive[component]]
    assert len(config['configurations_rad']) == 2


def test_sign_control_is_separate_and_bidirectional():
    config = load(NEW / 'sign_preregistration.json')
    assert set(config['configurations_rad']) == {'home'}
    loads = {item['id']: item for item in config['loads']}
    assert set(loads) == {'positive_fx', 'negative_fx', 'positive_fy', 'negative_fy'}
    for axis in ('fx', 'fy'):
        for component in ('force_world_n', 'torque_world_nm'):
            assert loads['negative_' + axis][component] == [-x for x in loads['positive_' + axis][component]]
    assert config['independent_sign_control']['min_abs_moment_nm'] > 0
    assert config['independent_sign_control']['max_q1_effort_plus_moment_error_nm'] > 0
