"""Evaluate synthetic records only; never evidence of simulator operation."""
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from task28_validator.model import RobotModel
from offline.task28_p2_2.evaluate_probe import evaluate, ROOT


@pytest.fixture
def synthetic(tmp_path):
    config = json.loads((ROOT / "offline/task28_p2_2/evidence/static_probe_preregistration.json").read_text())
    config["configurations_rad"] = {"home": config["configurations_rad"]["home"]}
    config["loads"] = [config["loads"][1]]
    config["observation_window_steps"] = 3
    config_path = tmp_path / "synthetic_config.json"
    config_path.write_text(json.dumps(config))
    conf_hash = hashlib.sha256(config_path.read_bytes()).hexdigest()
    input_dir = tmp_path / "synthetic_input"
    input_dir.mkdir()
    names = config["joint_names"]
    state = {"status": "COLLECTION_COMPLETE_NOT_EVALUATED", "source_kind": "SYNTHETIC_TEST",
        "config_sha256": conf_hash, "asset_sha256": config["fixture_usd_sha256"],
        "collector_sha256": hashlib.sha256((ROOT / "offline/task28_p2_2/static_isaac_probe.py").read_bytes()).hexdigest(),
        "runtime": {"ordered_joint_names": names, "all_dof_names": names,
                    "ordered_dof_indices": list(range(7)), "dof_stiffness": [1.] * 7,
                    "dof_damping": [1.] * 7, "physics_dt_s": config["physics_dt_s"],
                    "physics_gravity": [[0, 0, -1], 9.81],
                    "observation_source": "SingleArticulation.get_measured_joint_efforts only"}}
    (input_dir / "collector_status.json").write_text(json.dumps(state))
    model = RobotModel()
    q = np.array(config["configurations_rad"]["home"])
    J = model.jacobian("left", q)
    force = np.array([1., 0., 0.])
    link = model._chain("left", q)[0]["left_fr3_link7"]
    quat_xyzw = Rotation.from_matrix(link[:3, :3]).as_quat()
    quat = quat_xyzw[[3, 0, 1, 2]]
    for condition in ("unloaded", "loaded"):
        f = force if condition == "loaded" else np.zeros(3)
        tau = J.T @ np.r_[f, np.zeros(3)]
        rows = [{"physics_step": i + 1, "simulation_time_s": (i+1)*config["physics_dt_s"],
            "q_rad": q.tolist(), "dq_rad_s": [0.]*7, "effort_nm": tau.tolist(),
            "link7_position_world_m": link[:3, 3].tolist(), "link7_orientation_world_wxyz": quat.tolist(),
            "applied_force_world_n": f.tolist(), "application_point_world_m": model.fk("left", q)[:3, 3].tolist(),
            "q_pre_step_rad": q.tolist()} for i in range(3)]
        doc = {"configuration": "home", "load": config["loads"][0], "condition": condition,
               "observations": rows, "config_sha256": conf_hash}
        (input_dir / f"home_positive_x_{condition}.json").write_text(json.dumps(doc))
    return input_dir, config_path


def test_synthetic_pair_passes_without_claiming_actual_measurement(synthetic):
    input_dir, config_path = synthetic
    result = evaluate(input_dir, config_path)
    assert result["status"] == "SYNTHETIC_GATES_PASS_NOT_ACTUAL_EVIDENCE"
    case = result["cases"][0]
    assert case["status"] == "GATES_PASS"
    assert max(abs(x) for x in case["tau_error_nm"]) < 1e-14
    assert max(abs(x) for x in case["wrench_error_n_nm"]) < 1e-12
    assert case["applied_sign_automatically_flipped"] is False


def test_reverse_sign_is_rejected_never_flipped(synthetic):
    input_dir, config_path = synthetic
    path = input_dir / "home_positive_x_loaded.json"
    doc = json.loads(path.read_text())
    for row in doc["observations"]:
        row["effort_nm"] = [-x for x in row["effort_nm"]]
    path.write_text(json.dumps(doc))
    result = evaluate(input_dir, config_path)
    assert result["status"] == "REJECTED"
    assert "POSSIBLE_ACTION_REACTION_SIGN_MISMATCH_NO_FLIP" in result["cases"][0]["failures"]
    assert result["cases"][0]["applied_sign_automatically_flipped"] is False


def test_missing_and_invalid_observations(synthetic):
    input_dir, config_path = synthetic
    path = input_dir / "home_positive_x_loaded.json"
    path.unlink()
    assert evaluate(input_dir, config_path)["status"] == "INCOMPLETE"


def test_steady_state_failure_and_wrong_applied_force(synthetic):
    input_dir, config_path = synthetic
    path = input_dir / "home_positive_x_loaded.json"
    doc = json.loads(path.read_text())
    doc["observations"][0]["dq_rad_s"][0] = .02
    doc["observations"][0]["applied_force_world_n"] = [0, 1, 0]
    path.write_text(json.dumps(doc))
    result = evaluate(input_dir, config_path)
    assert result["status"] == "REJECTED"
    assert "LOADED_MAX_ABS_VELOCITY_RAD_S" in result["cases"][0]["failures"]
    assert "LOADED_APPLIED_FORCE_MISMATCH" in result["cases"][0]["failures"]


def test_independently_reconstructed_fk_rejects_mismatched_pose(synthetic):
    input_dir, config_path = synthetic
    path = input_dir / "home_positive_x_loaded.json"
    doc = json.loads(path.read_text())
    doc["observations"][0]["link7_position_world_m"][0] += .001
    path.write_text(json.dumps(doc))
    result = evaluate(input_dir, config_path)
    assert result["status"] == "REJECTED"
    assert "LOADED_MAX_LINK7_FK_POSITION_DIFFERENCE_M" in result["cases"][0]["failures"]
