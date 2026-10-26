"""Independent expected frame counts for bounded threshold/noise experiments."""
import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from r1_experiments import sensitivity,support_noise
from task28_contact.validator import ContactModeValidator


def test_threshold_noise_grid_matches_independent_run_length_expectations():
    rows=sensitivity(ContactModeValidator())
    assert len(rows)==15
    high_expected={(.0025,0.):3,(.0025,.0001):3,(.0025,.0005):3,
                   (.003,0.):3,(.003,.0001):9,(.003,.0005):9,
                   (.0035,0.):12,(.0035,.0001):12,(.0035,.0005):9}
    low_expected={( .00075,0.):12,(.00075,.0001):12,
                  (.001,0.):3,(.001,.0001):9,(.00125,0.):3,(.00125,.0001):3}
    for row in rows:
        amplitude=row['noise_amplitude_m']
        if row['direction']=='lift':
            assert row['first_confirmation_frame']==high_expected[(row['d_high_m'],amplitude)]
            assert all(frame['support_records']==0 for frame in row['frames'])
        else:
            assert row['first_confirmation_frame']==low_expected[(row['d_low_m'],amplitude)]
            assert all(not frame['can_release'] for frame in row['frames'])


def test_millimetre_hysteresis_never_inflates_strict_support_noise_band():
    rows=support_noise(ContactModeValidator())
    assert len(rows)==14
    for row in rows:
        assert row['valid_supported_state']==(abs(row['cube_observation_z_noise_m'])<=.000001)
        if not row['valid_supported_state']:assert row['accepted_support_records']==0
