#!/usr/bin/env python3
"""Read-only static-pair evaluation; creates a new result directory.

Inputs are recorded Isaac observations, or explicit synthetic test fixtures.
The estimator sees only q, paired torque difference, J and metadata; the
registered known load is read *after* estimation as a separate oracle. This
program never launches Isaac, commands an actuator, or changes the collector.
"""
from __future__ import annotations
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "offline/task28_g1_1"))
from task28_validator.model import RobotModel
from offline.task28_p2_2.estimator import Contract, Observation, estimate


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _read_condition(path, config_hash, config_id, load_id, condition, n, dt, model):
    doc = json.loads(path.read_text())
    if (doc.get("config_sha256") != config_hash or doc.get("configuration") != config_id
            or doc.get("condition") != condition or doc.get("load", {}).get("id") != load_id):
        raise ValueError("IDENTITY_OR_CONFIG_HASH_MISMATCH")
    rows = doc["observations"]
    if len(rows) != n:
        raise ValueError("INCOMPLETE_OBSERVATION_WINDOW")
    def array(key, width):
        value = np.asarray([r[key] for r in rows], float)
        if value.shape != (n, width) or not np.isfinite(value).all():
            raise ValueError("MALFORMED_" + key.upper())
        return value
    q, dq, tau = array("q_rad", 7), array("dq_rad_s", 7), array("effort_nm", 7)
    t = np.asarray([r["simulation_time_s"] for r in rows], float)
    step = np.asarray([r["physics_step"] for r in rows])
    observed_p = array("link7_position_world_m", 3)
    observed_quat = array("link7_orientation_world_wxyz", 4)
    if not np.allclose(np.linalg.norm(observed_quat, axis=1), 1, rtol=0, atol=1e-5):
        raise ValueError("INVALID_LINK_QUATERNION")
    expected = [model._chain("left", qi)[0]["left_fr3_link7"] for qi in q]
    fk_p = np.asarray([np.linalg.norm(observed_p[i] - T[:3, 3]) for i, T in enumerate(expected)])
    fk_r = np.asarray([np.linalg.norm(Rotation.from_matrix(T[:3, :3].T @
        Rotation.from_quat(observed_quat[i][[1, 2, 3, 0]]).as_matrix()).as_rotvec()) for i, T in enumerate(expected)])
    if (not all(np.isfinite(x).all() for x in (t, step, fk_p, fk_r))
            or np.any(t < 0) or np.any(np.diff(step) != 1)
            or not np.allclose(np.diff(t), dt, rtol=0, atol=1e-9)
            or np.any(fk_p < 0) or np.any(fk_r < 0)):
        raise ValueError("INVALID_OBSERVATION_TIMING_OR_FK")
    return {"q": q, "dq": dq, "tau": tau, "time": t, "fk_p": fk_p, "fk_r": fk_r,
            "rows": rows, "sha256": digest(path)}


def evaluate(input_dir, config_path):
    config = json.loads(config_path.read_text())
    config_hash = digest(config_path)
    result = {"schema": "task28.p2_2.static_evaluation.v1", "status": "INCOMPLETE",
              "config_sha256": config_hash, "input_directory": str(input_dir.resolve()),
              "estimator_sha256": digest(ROOT / "offline/task28_p2_2/estimator.py"),
              "evaluator_sha256": digest(Path(__file__)), "flags": config["flags"],
              "oracle_access": "after estimator returns; never passed to estimator",
              "reference_method": "paired steady-state mean efforts; no online observer implied",
              "cases": [], "global_failures": []}
    status_path = input_dir / "collector_status.json"
    if not status_path.is_file():
        result["global_failures"].append("MISSING_COLLECTOR_STATUS")
        return result
    state = json.loads(status_path.read_text())
    result["collector_status_sha256"] = digest(status_path)
    if state.get("config_sha256") != config_hash:
        result["global_failures"].append("COLLECTOR_CONFIG_HASH_MISMATCH")
    if state.get("asset_sha256") != config["fixture_usd_sha256"]:
        result["global_failures"].append("COLLECTOR_ASSET_HASH_MISMATCH")
    if state.get("collector_sha256") != digest(ROOT / "offline/task28_p2_2/static_isaac_probe.py"):
        result["global_failures"].append("COLLECTOR_SOURCE_HASH_MISMATCH")
    result["source_kind"] = state.get("source_kind", "MISSING")
    if result["source_kind"] not in ("ISAAC_SINGLE_ARTICULATION_STANDALONE", "SYNTHETIC_TEST"):
        result["global_failures"].append("SOURCE_KIND_NOT_VERIFIED")
    runtime = state.get("runtime", {})
    names = config["joint_names"]
    all_names, indices = runtime.get("all_dof_names", []), runtime.get("ordered_dof_indices", [])
    if (runtime.get("ordered_joint_names") != names or len(indices) != 7 or len(set(indices)) != 7
            or any(not isinstance(i, int) or i < 0 or i >= len(all_names) or all_names[i] != n
                   for i, n in zip(indices, names))
            or runtime.get("observation_source") != "SingleArticulation.get_measured_joint_efforts only"):
        result["global_failures"].append("JOINT_ORDER_OR_EFFORT_SOURCE_UNVERIFIED")
    if state.get("status") != "COLLECTION_COMPLETE_NOT_EVALUATED":
        result["global_failures"].append("COLLECTION_NOT_COMPLETE")
    if len(set(all_names)) != len(all_names):
        result["global_failures"].append("AMBIGUOUS_ARTICULATION_DOF_NAMES")
    try:
        stiffness = np.asarray(runtime["dof_stiffness"], float)
        damping = np.asarray(runtime["dof_damping"], float)
        gravity = runtime["physics_gravity"]
        gravity_vec = np.asarray(gravity[0], float) * float(gravity[1])
        if (stiffness.shape != (len(all_names),) or damping.shape != (len(all_names),)
                or not np.isfinite(np.r_[stiffness, damping, gravity_vec]).all()
                or np.any(stiffness[indices] <= 0)
                or abs(float(runtime["physics_dt_s"]) - config["physics_dt_s"]) > 1e-12
                or not np.allclose(gravity_vec, config["gravity_m_s2"], rtol=0, atol=1e-6)):
            raise ValueError("RUNTIME_SETTINGS_MISMATCH")
    except (KeyError, ValueError, TypeError, IndexError):
        result["global_failures"].append("RUNTIME_SETTINGS_UNVERIFIED")
    model = RobotModel()
    th = config["thresholds"]
    n = config["observation_window_steps"]
    missing = False
    for config_id in config["configurations_rad"]:
        for load_record in config["loads"]:
            load_id = load_record["id"]
            case = {"configuration": config_id, "load_id": load_id, "status": "INCOMPLETE", "failures": []}
            result["cases"].append(case)
            paths = {kind: input_dir / f"{config_id}_{load_id}_{kind}.json" for kind in ("unloaded", "loaded")}
            if not all(path.is_file() for path in paths.values()):
                missing = True
                case["failures"].append("MISSING_CONDITION_FILE")
                continue
            try:
                pair = {k: _read_condition(p, config_hash, config_id, load_id, k, n, config["physics_dt_s"], model)
                        for k, p in paths.items()}
                u, l = pair["unloaded"], pair["loaded"]
                q = l["q"].mean(axis=0)
                delta = l["tau"].mean(axis=0) - u["tau"].mean(axis=0)
                J = model.jacobian("left", q)
                if any(not model.within_limits(qi) for item in pair.values() for qi in item["q"]):
                    case["failures"].append("OBSERVATION_Q_OUTSIDE_LIMITS")
                # Measured simulation timestamps represent the observation window;
                # averages and paired reset trials are offline, not synchronous feedback.
                t_mean = float(l["time"].mean())
                contract = Contract(tuple(names), "world", "left_fr3_side_suction_tcp", 0, 0)
                obs = Observation(q, delta, J, q.copy(), tuple(names), tuple(names),
                    "world", "left_fr3_side_suction_tcp", t_mean, t_mean, t_mean, t_mean,
                    "paired_static_load_difference", "mean loaded minus mean unloaded; file hashes in record",
                    "frozen G1 left URDF world-frame virtual TCP Jacobian")
                estimated = estimate(obs, contract)
                # Oracle is introduced only after estimate(), never as an input.
                F = np.asarray(load_record["force_world_n"], float)
                r = np.asarray(load_record["offset_from_virtual_tcp_world_m"], float)
                oracle = np.r_[F, np.cross(r, F)]
                expected_tau = J.T @ oracle
                tau_error = delta - expected_tau
                wrench_error = np.asarray(estimated.wrench_minimum_norm) - oracle
                opposite_error = delta + expected_tau
                # Check actual force and point logs independently against registration,
                # using pre-step q recorded by the collector. Oracle remains excluded
                # from estimate() and reaches this logic only after estimation.
                for name, item in pair.items():
                    expected_force = F if name == "loaded" else np.zeros(3)
                    offset = r if name == "loaded" else np.zeros(3)
                    for row in item["rows"]:
                        logged_force = np.asarray(row["applied_force_world_n"], float)
                        logged_point = np.asarray(row["application_point_world_m"], float)
                        q_pre = np.asarray(row["q_pre_step_rad"], float)
                        if (logged_force.shape != (3,) or logged_point.shape != (3,) or q_pre.shape != (7,)
                                or not np.isfinite(np.r_[logged_force, logged_point, q_pre]).all()):
                            raise ValueError("MALFORMED_FORCE_APPLICATION_LOG")
                        expected_point = model.fk("left", q_pre)[:3, 3] + offset
                        if not np.allclose(logged_force, expected_force, rtol=0, atol=1e-12):
                            case["failures"].append(name.upper() + "_APPLIED_FORCE_MISMATCH")
                            break
                        if not np.allclose(logged_point, expected_point, rtol=0, atol=1e-10):
                            case["failures"].append(name.upper() + "_APPLICATION_POINT_MISMATCH")
                            break
                condition_stats = {}
                for name, item in pair.items():
                    stats = {"max_abs_velocity_rad_s": float(np.max(np.abs(item["dq"]))),
                        "max_window_q_range_rad": float(np.max(np.ptp(item["q"], axis=0))),
                        "max_window_effort_std_nm": float(np.max(np.std(item["tau"], axis=0))),
                        "max_link7_fk_position_difference_m": float(np.max(item["fk_p"])),
                        "max_link7_fk_rotation_difference_rad": float(np.max(item["fk_r"]))}
                    condition_stats[name] = stats
                    for key, value in stats.items():
                        if value > th[key]:
                            case["failures"].append(name.upper() + "_" + key.upper())
                pair_q = float(np.max(np.abs(l["q"].mean(axis=0) - u["q"].mean(axis=0))))
                if pair_q > th["max_pair_mean_q_difference_rad"]:
                    case["failures"].append("PAIR_Q_MISMATCH")
                if estimated.rank != 6 or min(estimated.singular_values) < th["minimum_jacobian_singular_value"]:
                    case["failures"].append("JACOBIAN_RANK_OR_CONDITION_GATE")
                if np.max(np.abs(tau_error)) > th["max_abs_torque_error_nm"]:
                    case["failures"].append("JOINT_TORQUE_ORACLE_MISMATCH")
                if np.linalg.norm(wrench_error[:3]) > th["max_force_error_n"]:
                    case["failures"].append("FORCE_ORACLE_MISMATCH")
                if np.linalg.norm(wrench_error[3:]) > th["max_moment_error_nm"]:
                    case["failures"].append("MOMENT_ORACLE_MISMATCH")
                if not np.any(F) and np.max(np.abs(delta)) > th["zero_delta_tau_inf_nm"]:
                    case["failures"].append("ZERO_LOAD_TORQUE_MISMATCH")
                if np.any(F) and np.linalg.norm(opposite_error) < np.linalg.norm(tau_error):
                    case["failures"].append("POSSIBLE_ACTION_REACTION_SIGN_MISMATCH_NO_FLIP")
                case.update({"input_sha256": {k: v["sha256"] for k, v in pair.items()},
                    "loaded_mean_q_rad": q.tolist(), "unloaded_mean_q_rad": u["q"].mean(axis=0).tolist(),
                    "loaded_mean_effort_nm": l["tau"].mean(axis=0).tolist(),
                    "unloaded_mean_effort_nm": u["tau"].mean(axis=0).tolist(),
                    "delta_tau_nm": delta.tolist(), "condition_stats": condition_stats,
                    "max_pair_mean_q_difference_rad": pair_q,
                    "loaded_mean_q_minus_target_rad": (q - np.asarray(config["configurations_rad"][config_id])).tolist(),
                    "estimate": asdict(estimated),
                    "oracle_wrench_on_robot_world_n_nm": oracle.tolist(),
                    "expected_delta_tau_nm": expected_tau.tolist(), "tau_error_nm": tau_error.tolist(),
                    "wrench_error_n_nm": wrench_error.tolist(),
                    "direct_torque_error_norm_nm": float(np.linalg.norm(tau_error)),
                    "opposite_sign_torque_error_norm_nm": float(np.linalg.norm(opposite_error)),
                    "applied_sign_automatically_flipped": False})
                case["status"] = "REJECTED" if case["failures"] else "GATES_PASS"
            except (KeyError, json.JSONDecodeError) as exc:
                missing = True
                case["status"] = "INCOMPLETE"
                case["failures"].append("MISSING_OR_TRUNCATED_DATA:" + str(exc))
            except (ValueError, TypeError, OverflowError) as exc:
                if str(exc) == "INCOMPLETE_OBSERVATION_WINDOW":
                    missing = True
                    case["status"] = "INCOMPLETE"
                else:
                    case["status"] = "REJECTED"
                case["failures"].append("INVALID_OR_NUMERICAL:" + str(exc))
    if missing or state.get("status") != "COLLECTION_COMPLETE_NOT_EVALUATED":
        result["status"] = "INCOMPLETE"
    elif result["global_failures"] or any(c["status"] != "GATES_PASS" for c in result["cases"]):
        result["status"] = "REJECTED"
    elif result["source_kind"] == "SYNTHETIC_TEST":
        result["status"] = "SYNTHETIC_GATES_PASS_NOT_ACTUAL_EVIDENCE"
    else:
        result["status"] = "ALL_PREREGISTERED_STATIC_GATES_PASS_PENDING_INDEPENDENT_REVIEW"
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=ROOT / "offline/task28_p2_2/evidence/static_probe_preregistration.json")
    args = parser.parse_args()
    args.output.mkdir(parents=False, exist_ok=False)
    result = evaluate(args.input, args.config)
    with (args.output / "evaluation.json").open("x") as f:
        json.dump(result, f, indent=2, sort_keys=True, allow_nan=False)
        f.write("\n")
    print(result["status"])
    return 0 if result["status"].startswith("ALL_PREREGISTERED") else 1


if __name__ == "__main__":
    raise SystemExit(main())
