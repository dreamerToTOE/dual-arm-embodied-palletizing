#!/usr/bin/env python3
"""Task27 torque-owned bridge for the Issue #1 integration attempt.

The original Task27 feed/gripper/rail bridge is loaded from the absolute source
selected by the existing wrapper.  Its arm position sink is replaced at runtime
with a fail-closed seven-DOF effort controller; finger targets, feed, rails,
Surface Grippers and D6 lifecycle remain owned by the original bridge.

This file is a new execution entry.  It does not edit or export the Task27
scene, and it never starts the historical position sink as an arm actuator.
"""
from __future__ import annotations

import builtins
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import types

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

ARM_DOF_NAMES = tuple(f"fr3_joint{i}" for i in range(1, 8))
FINGER_DOF_NAMES = ("fr3_finger_joint1", "fr3_finger_joint2")
SIDES = ("left", "right")
TORQUE_FRAME = "task28_t0_dual_sync"
BRIDGE_SOURCE = Path("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task26_truck_box_bridge.py")


def _finite(value, size, code):
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.shape != (size,) or not np.isfinite(arr).all():
        raise RuntimeError(code)
    return arr.copy()


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


class Task27TorqueBridge:
    """Patch the original feed bridge while retaining its scene semantics."""

    def __init__(self, feed_bridge, output: Path):
        from isaacsim.core.utils.types import ArticulationAction
        import omni.physx

        self.feed = feed_bridge
        self.output_path = Path(output)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.output_path.open("w", encoding="utf-8")
        self.articulations = feed_bridge.articulations
        self._log({
            "event": "TASK27_TORQUE_BRIDGE_BOOT",
            "scene_profile": "task27",
            "scene_root": "/World/Task27",
            "original_feed_bridge_source": str(BRIDGE_SOURCE),
            "original_feed_bridge_sha256": _sha256(BRIDGE_SOURCE),
            "position_arm_sink_replaced": True,
            "finger_sink_preserved": True,
            "torque_frame": TORQUE_FRAME,
            "arm_dof_names": list(ARM_DOF_NAMES),
            "real_robot": False,
        })
        self._old_apply_joint_command = feed_bridge._apply_joint_command
        self._old_physics_step = feed_bridge._physics_step
        self._qref = {}
        self._dqref = {side: np.zeros(7) for side in SIDES}
        self._qref_stamp = {side: None for side in SIDES}
        self._q0 = {}
        self._previous_effort = {side: np.zeros(7) for side in SIDES}
        self._step = 0
        self._time_s = 0.0
        self._latched_error = None
        self._writes = 0
        self._position_arm_write_count = 0
        self._target_count = {side: 0 for side in SIDES}
        self._last_tcp = {}
        self._interface = None

        # Snapshot and take over only the seven arm DOFs.  Finger policy is
        # checked before and after and remains untouched.
        for side, articulation in self.articulations.items():
            view = articulation._articulation_view
            names = tuple(articulation.dof_names)
            if names != ARM_DOF_NAMES + FINGER_DOF_NAMES:
                raise RuntimeError(f"DOF_ORDER:{side}")
            q = _finite(articulation.get_joint_positions(), 9, f"Q0:{side}")
            self._q0[side] = q[:7].copy()
            self._qref[side] = q[:7].copy()
            modes = list(view.get_effort_modes()[0])
            kp, kd = (np.asarray(x, dtype=float).reshape(-1) for x in view.get_gains())
            if len(modes) != 9 or kp.shape != (9,) or kd.shape != (9,):
                raise RuntimeError(f"DRIVE_SNAPSHOT:{side}")
            self._finger_signature_before = getattr(self, "_finger_signature_before", {})
            self._finger_signature_before[side] = (
                tuple(modes[7:]), tuple(float(x) for x in kp[7:]), tuple(float(x) for x in kd[7:])
            )
            view.set_effort_modes("force", joint_indices=np.arange(7, dtype=np.int32))
            view.set_gains(kps=np.zeros(7), kds=np.zeros(7),
                           joint_indices=np.arange(7, dtype=np.int32), save_to_usd=False)
            modes_after = list(view.get_effort_modes()[0])
            kp_after, kd_after = (np.asarray(x, dtype=float).reshape(-1) for x in view.get_gains())
            if modes_after[:7] != ["force"] * 7 or np.any(kp_after[:7] != 0) or np.any(kd_after[:7] != 0):
                raise RuntimeError(f"DRIVE_TAKEOVER:{side}")
            if (tuple(modes_after[7:]), tuple(float(x) for x in kp_after[7:]), tuple(float(x) for x in kd_after[7:])) != (
                    self._finger_signature_before[side][0],
                    self._finger_signature_before[side][1],
                    self._finger_signature_before[side][2]):
                raise RuntimeError(f"FINGER_POLICY_CHANGED:{side}")

        from offline.task28_t0.torque_interface import TorqueInterface
        self._interface = TorqueInterface()
        for side, articulation in self.articulations.items():
            view = articulation._articulation_view
            modes = tuple(view.get_effort_modes()[0][:7])
            kp, kd = (np.asarray(x, dtype=float).reshape(-1)[:7] for x in view.get_gains())
            self._interface.activate(
                dof_names=tuple(articulation.dof_names), position_sink_active=False,
                arm_effort_modes=modes, arm_kp=kp, arm_kd=kd,
                finger_signature={"side": side, "signature": self._finger_signature_before[side]},
                finger_signature_after={"side": side, "signature": self._finger_signature_before[side]},
            )
            break

        self._ArticulationAction = ArticulationAction
        # Existing callback calls this method; it now stores arm targets and
        # forwards only finger targets to the historical writer.
        feed_bridge._apply_joint_command = types.MethodType(self._apply_joint_command, feed_bridge)

        # Replace the callback subscription so effort is applied before the
        # original feed/gripper/rail callback for the same physics step.
        try:
            feed_bridge.physics_sub.unsubscribe()
        except Exception:
            pass
        feed_bridge.physics_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(
            self._physics_step
        )
        feed_bridge._task28_torque_owner = self
        self._log({
            "event": "TASK27_TORQUE_BRIDGE_ACTIVE",
            "arm_effort_modes": {side: ["force"] * 7 for side in SIDES},
            "arm_kp_zero": True,
            "arm_kd_zero": True,
            "position_arm_write_count": 0,
            "suction_or_d6_commands_owned_here": False,
            "qp_status": "NOT_IMPLEMENTED_TASK27_LIVE_QP",
            "observer_status": "MODEL_ONLY_CANDIDATE_NO_PHYSICAL_TAU_J",
        })

    def _log(self, row):
        self.stream.write(json.dumps(row, sort_keys=True, allow_nan=False,
                                     default=lambda x: x.tolist() if isinstance(x, np.ndarray) else x.item() if isinstance(x, np.generic) else str(x)) + "\n")
        self.stream.flush()

    def _apply_joint_command(self, feed, side, command):
        stamp, names, positions = command
        positions = np.asarray(positions, dtype=float).reshape(-1)
        if len(names) != len(positions):
            raise RuntimeError("COMMAND_LENGTH")
        arm = self._qref[side].copy()
        arm_seen = False
        finger_names, finger_values = [], []
        for name, value in zip(names, positions):
            if name in ARM_DOF_NAMES:
                arm[ARM_DOF_NAMES.index(name)] = float(value)
                arm_seen = True
            elif name in FINGER_DOF_NAMES:
                finger_names.append(name)
                finger_values.append(float(value))
            else:
                raise RuntimeError(f"UNKNOWN_DOF:{side}:{name}")
        if arm_seen:
            if not np.isfinite(arm).all():
                raise RuntimeError("NONFINITE_QREF")
            previous = self._qref[side]
            self._dqref[side] = (arm - previous) * 60.0
            self._qref[side] = arm
            self._qref_stamp[side] = tuple(stamp)
            self._target_count[side] += 1
            # This is a target capture, never a position actuator write.
            self._position_arm_write_count = 0
        if finger_names:
            self._old_apply_joint_command(
                side, (stamp, tuple(finger_names), np.asarray(finger_values, dtype=float))
            )

    def _state(self, side):
        articulation = self.articulations[side]
        q = _finite(articulation.get_joint_positions(), 9, f"Q:{side}")
        dq = _finite(articulation.get_joint_velocities(), 9, f"DQ:{side}")
        gravity = _finite(articulation._articulation_view.get_generalized_gravity_forces(), 9, f"G:{side}")
        applied = _finite(articulation.get_applied_joint_efforts(), 9, f"APPLIED:{side}")
        projected = _finite(articulation.get_measured_joint_efforts(), 9, f"PROJECTED:{side}")
        mass = _finite(articulation._articulation_view.get_mass_matrices(), 81, f"M:{side}").reshape(9, 9)
        coriolis = _finite(articulation._articulation_view.get_coriolis_and_centrifugal_forces(), 9, f"C:{side}")
        return q, dq, gravity, applied, projected, mass, coriolis

    @staticmethod
    def _tcp(stage, side):
        from pxr import Usd, UsdGeom
        path = f"/World/{side}_fr3/fr3_link8/side_suction_tool/side_suction_tcp"
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            raise RuntimeError(f"TCP_MISSING:{side}")
        t = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        return [float(t[i]) for i in range(3)]

    def _torque_step(self, dt):
        from offline.task28_p5_qp_6_r8.controller import gains, torque
        if abs(float(dt) - 1.0 / 60.0) > 1e-6:
            raise RuntimeError("PHYSICS_DT")
        if self._step > 0:
            self._interface.post_step(physics_step=self._step, time_s=self._time_s)
        states = {side: self._state(side) for side in SIDES}
        efforts, components = {}, {}
        clipped = {}
        for side in SIDES:
            q, dq, gravity, _, _, mass, _ = states[side]
            kp, kd = gains(mass, "frozen_mass_hold")
            raw, comp = torque({"q": q, "dq": dq, "g": gravity}, self._qref[side], self._dqref[side], kp, kd)
            limits = np.asarray(self._interface.limits.abs_Nm, dtype=float)
            rate = np.asarray(self._interface.limits.rate_Nm_s, dtype=float) * (1.0 / 60.0)
            bounded = np.clip(raw, -limits, limits)
            bounded = self._previous_effort[side] + np.clip(bounded - self._previous_effort[side], -rate, rate)
            clipped[side] = bool(np.any(np.abs(bounded - raw) > 1e-9))
            efforts[side] = bounded
            components[side] = comp
        command = self._interface.submit_pair(
            physics_step=self._step, time_s=self._time_s,
            left_Nm=efforts["left"], right_Nm=efforts["right"],
            frame_id=TORQUE_FRAME, units="N*m",
        )
        self._interface.take_pending(physics_step=self._step, time_s=self._time_s)
        for side in SIDES:
            self.articulations[side].apply_action(self._ArticulationAction(
                joint_efforts=efforts[side], joint_indices=np.arange(7, dtype=np.int64)
            ))
        applied = {side: _finite(self.articulations[side].get_applied_joint_efforts(), 9, f"APPLIED_AFTER:{side}")[:7] for side in SIDES}
        self._interface.record_write(command=command, applied_left=applied["left"], applied_right=applied["right"], physics_step=self._step, time_s=self._time_s)
        self._writes += 1
        self._log({
            "event": "TORQUE_FRAME", "physics_step": self._step, "time_s": self._time_s, "dt_s": float(dt),
            "q_ref_rad": {side: self._qref[side].tolist() for side in SIDES},
            "dq_ref_rad_s": {side: self._dqref[side].tolist() for side in SIDES},
            "q_rad": {side: states[side][0][:7].tolist() for side in SIDES},
            "dq_rad_s": {side: states[side][1][:7].tolist() for side in SIDES},
            "requested_effort_Nm": {side: efforts[side].tolist() for side in SIDES},
            "applied_effort_command_Nm": {side: applied[side].tolist() for side in SIDES},
            "projected_effort_not_sensor_Nm": {side: states[side][4][:7].tolist() for side in SIDES},
            "gravity_Nm": {side: states[side][2][:7].tolist() for side in SIDES},
            "tcp_world_m": {side: self._tcp(self.feed.stage, side) for side in SIDES},
            "qp": {"status": "NOT_IMPLEMENTED_TASK27_LIVE_QP", "solve_count": 0},
            "observer": {"status": "MODEL_ONLY_CANDIDATE_NO_PHYSICAL_TAU_J", "contact_oracle": "NOT_CONNECTED"},
            "position_arm_write_count": 0, "torque_write_count": self._writes,
            "torque_rate_limited": clipped,
        })
        self._previous_effort = {side: efforts[side].copy() for side in SIDES}
        self._step += 1
        self._time_s += 1.0 / 60.0

    def _physics_step(self, dt):
        try:
            self._torque_step(dt)
            self._old_physics_step(dt)
        except Exception as exc:
            self._latched_error = repr(exc)
            self._log({"event": "FAIL_CLOSED", "error": self._latched_error,
                       "physics_step": self._step, "position_arm_write_count": 0})
            try:
                self.feed.physics_sub.unsubscribe()
            except Exception:
                pass

    def close(self):
        try:
            self.feed.physics_sub.unsubscribe()
        except Exception:
            pass
        try:
            self.feed.shutdown()
        except Exception as exc:
            self._log({"event": "FEED_BRIDGE_SHUTDOWN_ERROR", "error": repr(exc)})
        try:
            self._interface.stop()
        except Exception:
            pass
        self._log({
            "event": "TASK27_TORQUE_BRIDGE_STOPPED",
            "physics_steps": self._step,
            "torque_writes": self._writes,
            "position_arm_write_count": self._position_arm_write_count,
            "target_count": self._target_count,
            "latched_error": self._latched_error,
            "qp_status": "NOT_IMPLEMENTED_TASK27_LIVE_QP",
            "observer_status": "MODEL_ONLY_CANDIDATE_NO_PHYSICAL_TAU_J",
        })
        self.stream.close()


def start_task27_torque_bridge(output=None):
    if getattr(builtins, "_task28_task27_torque_bridge", None) is not None:
        raise RuntimeError("TASK27_TORQUE_BRIDGE_ALREADY_REGISTERED")
    if not BRIDGE_SOURCE.is_file():
        raise RuntimeError(f"ORIGINAL_BRIDGE_SOURCE_MISSING:{BRIDGE_SOURCE}")
    if getattr(builtins, "_task26_batched_feed_bridge", None) is not None:
        try:
            builtins._task26_batched_feed_bridge.shutdown()
        except Exception:
            pass
        delattr(builtins, "_task26_batched_feed_bridge")
    namespace = {"__name__": "task27_original_feed_bridge", "_SIDE_SUCTION_SCENARIO": "task27"}
    exec(compile(BRIDGE_SOURCE.read_text(encoding="utf-8"), str(BRIDGE_SOURCE), "exec"), namespace, namespace)
    feed = getattr(builtins, "_task26_batched_feed_bridge", None)
    if feed is None:
        raise RuntimeError("ORIGINAL_FEED_BRIDGE_NOT_STARTED")
    path = Path(output) if output else REPO_ROOT / "offline/task28_task27_integration/evidence/task27_torque_bridge.jsonl"
    bridge = Task27TorqueBridge(feed, path)
    builtins._task28_task27_torque_bridge = bridge
    return bridge


if __name__ == "__main__":
    raise SystemExit("Run start_task27_torque_bridge(...) from the Isaac Script Editor or integration runner")
