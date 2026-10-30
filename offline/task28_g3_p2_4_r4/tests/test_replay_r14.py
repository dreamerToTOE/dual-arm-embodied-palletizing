import json
from pathlib import Path

from offline.task28_g3_p2_4_r4.replay_r14_sources import replay


def test_r14_no_contact_sources_replay_strictly():
    root = Path(__file__).parents[3]
    raw = root / "offline/task28_r14/evidence/no_contact/attempt_01/raw/zero_load_hold_observations.json"
    result = replay(raw)
    assert result["status"] == "R14_SOURCES_STRICTLY_ADAPTED"
    assert result["sample_count"] == 240
    assert result["accepted_counts"] == {
        "simulation_drive_effort": 240,
        "simulation_applied_effort": 240,
        "simulation_projected_effort": 240,
    }
    assert result["source_validation"] == "SIMULATION_CANDIDATE"
    assert result["simulation_tau_J_equivalence"] is False
    assert result["candidate_external_torque_validated"] is False
    assert result["hardware_or_ros"] is False
