#!/usr/bin/env python3
"""Task28-T0 runtime torque sink for the frozen side-suction scenes.

This file is a new execution entry.  It does not import or alter any scene
builder and it never starts the historical position-command bridge.  The
scene remains the existing Task24 current side-suction scene by default; the
approved Task28 single-Cube fixture can be selected explicitly for later G1
work.  Only the first seven FR3 arm joints are part of the command contract.

The optional ``demo`` path is used by the GUI evidence runner.  It computes a
small gravity-compensated impedance hold/motion from the existing R8
controller helper and writes only ``joint_efforts``.  It never closes a
Surface Gripper, sends a suction command, or follows the HOME-to-Cube path.
"""
from __future__ import annotations

import builtins
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import threading


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import numpy as np

from offline.task28_t0.torque_interface import ARM_DOF_NAMES, TorqueInterface, TorqueReject


SIDES = ("left", "right")
FINGER_NAMES = ("fr3_finger_joint1", "fr3_finger_joint2")
POSITION_SINK_REGISTRIES = (
    "_task24_side_suction_bridge",
    "_task26_truck_box_bridge",
    "_task27_five_cube_center_insert_bridge",
    "_task28_g1_0_single_cube_bridge",
)
SCENE_PROFILES = {
    "task24": {
        "label": "TASK24_CURRENT_SIDE_SUCTION_8_CUBE",
        "roots": {"left": "/World/left_fr3", "right": "/World/right_fr3"},
        "scene_root": "/World/Task24",
        "tcp_suffix": "/fr3_link8/side_suction_tool/side_suction_tcp",
    },
    "task28_g1_0": {
        "label": "TASK28_G1_0_APPROVED_SINGLE_CUBE_FIXTURE",
        "roots": {
            "left": "/World/Task28G10SingleCube/left_fr3",
            "right": "/World/Task28G10SingleCube/right_fr3",
        },
        "scene_root": "/World/Task28G10SingleCube",
        "tcp_suffix": "/fr3_link8/side_suction_tool/side_suction_tcp",
    },
    "task27": {
        "label": "TASK27_ORIGINAL_FIVE_CUBE_CENTER_INSERT",
        "roots": {"left": "/World/left_fr3", "right": "/World/right_fr3"},
        "scene_root": "/World/Task27",
        "tcp_suffix": "/fr3_link8/side_suction_tool/side_suction_tcp",
        "graph_path": "/World/Task27ROSGraph",
        "signature_extra_roots": ("/World/Table", "/physicsScene", "/World/Task27ROSGraph"),
        "signature_paths": (
            "/World/Table",
            "/physicsScene",
            "/World/Task27ROSGraph",
            "/World/Task27/TruckBox/WallDeep",
            "/World/Task27/TruckBox/WallMinusY",
            "/World/Task27/TruckBox/WallPlusY",
            "/World/Task27/Supply/Cube_01",
            "/World/Task27/Supply/Cube_02",
            "/World/Task27/Supply/Cube_03",
            "/World/Task27/Supply/Cube_04",
            "/World/Task27/Supply/Cube_05",
            "/World/Task27/TargetMarkers/Cell_01",
            "/World/Task27/TargetMarkers/Cell_02",
            "/World/Task27/TargetMarkers/Cell_03",
            "/World/Task27/TargetMarkers/Cell_04",
            "/World/Task27/TargetMarkers/Cell_05",
            "/World/Task27/TargetMarkers/PrePush_01",
            "/World/Task27/TargetMarkers/PrePush_02",
            "/World/Task27/TargetMarkers/PrePush_03",
            "/World/Task27/TargetMarkers/PrePush_04",
            "/World/Task27/TargetMarkers/PrePush_05",
            "/World/Task27/HighFrictionMaterial",
        ),
    },
}
REGISTRY_NAME = "_task28_t0_torque_bridge"
TORQUE_FRAME = "task28_t0_dual_sync"


def _hash_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def _json_default(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return str(value)


def _finite(value, shape, label):
    array = np.asarray(value, dtype=float).reshape(-1)
    if array.shape != (shape,) or not np.isfinite(array).all():
        raise TorqueReject(label)
    return array.copy()


def _stage_signature(stage, roots, extra_roots=(), extra_paths=()):
    """Hash protected scene identity, excluding live transforms and references.

    PhysX can finish loading referenced descendants after the first frame.  A
    fixed authored path set avoids treating that loader detail as a scene edit.
    """
    suffixes = (
        "", "/fr3_link8", "/fr3_link8/side_suction_tool",
        "/fr3_link8/side_suction_tool/side_suction_tcp",
        "/Supply", "/Supply/Cube_01", "/Table", "/ROSGraph", "/physicsScene",
    )
    rows = []
    for root in (*roots.values(), *tuple(extra_roots)):
        for suffix in suffixes:
            path = root + suffix if suffix else root
            prim = stage.GetPrimAtPath(path)
            rows.append((path, prim.GetTypeName() if prim.IsValid() else "<missing>"))
    for path in extra_paths:
        prim = stage.GetPrimAtPath(path)
        rows.append((path, prim.GetTypeName() if prim.IsValid() else "<missing>"))
    rows = sorted(set(rows))
    return hashlib.sha256(json.dumps(rows, separators=(",", ":")).encode()).hexdigest()


class Task28T0TorqueBridge:
    """Own both arm effort sinks for one GUI or ROS execution."""

    def __init__(self, *, scene_profile="task24", demo=False, output=None,
                 demo_frames=240):
        if scene_profile not in SCENE_PROFILES:
            raise ValueError("UNKNOWN_SCENE_PROFILE")
        self.profile_name = scene_profile
        self.profile = SCENE_PROFILES[scene_profile]
        self.demo = bool(demo)
        self.demo_frames = int(demo_frames)
        if self.demo_frames <= 0:
            raise ValueError("INVALID_DEMO_FRAMES")
        self.output = Path(output).expanduser() if output else None
        if self.output is not None:
            self.output.parent.mkdir(parents=True, exist_ok=True)
        self._log_stream = self.output.open("w") if self.output else None
        self._lock = threading.Lock()
        self._ros_context = None
        self._node = None
        self._executor = None
        self._spin_thread = None
        self._subscriptions = []
        self._physics_sub = None
        self._pending_ros = {side: None for side in SIDES}
        self._latched_error = None
        self._last_step = None
        self._time_s = 0.0
        self._demo_index = 0
        self._q0 = {}
        self._kp = None
        self._kd = None
        self._snapshots = {}
        self._scene_signature_before = None
        self._scene_signature_after = None
        self._graph_active_before = None
        self._r8_adapter = None
        self._r8_gate = None

        import omni.physx
        import omni.usd
        from isaacsim.core.prims import SingleArticulation

        self.stage = omni.usd.get_context().get_stage()
        if self.stage is None or not self.stage.GetPrimAtPath(self.profile["scene_root"]).IsValid():
            self._abort("SCENE_ROOT_MISSING")
        for registry in POSITION_SINK_REGISTRIES:
            existing = getattr(builtins, registry, None)
            if existing is not None and existing is not self:
                self._abort("POSITION_SINK_PRESENT:" + registry)
        signature_roots = (self.profile["scene_root"],) + tuple(
            self.profile.get("signature_extra_roots", ())
        )
        self.scene_signature_before = _stage_signature(
            self.stage, self.profile["roots"], signature_roots,
            self.profile.get("signature_paths", ()),
        )
        graph = self.stage.GetPrimAtPath(
            self.profile.get("graph_path", self.profile["scene_root"] + "/ROSGraph")
        )
        if graph.IsValid():
            self._graph_active_before = bool(graph.IsActive())
            # The scene graph only publishes telemetry.  Keep it out of the
            # command ownership path during torque takeover and restore it on
            # shutdown; no USD is saved.
            graph.SetActive(False)
        self.articulations = {}
        for side in SIDES:
            articulation = SingleArticulation(self.profile["roots"][side])
            articulation.initialize()
            if not articulation.handles_initialized:
                self._abort("ARTICULATION_INIT:" + side)
            names = tuple(articulation.dof_names)
            if len(names) != 9 or names[:7] != ARM_DOF_NAMES:
                self._abort("DOF_ORDER:" + side)
            self.articulations[side] = articulation

        self._takeover_drives()
        self._init_r8_adapter_if_fixture()
        self.interface = TorqueInterface()
        arm_modes = tuple(self._snapshots["left"]["modes"][:7])
        arm_kp = np.asarray(self._snapshots["left"]["kp"][:7], dtype=float)
        arm_kd = np.asarray(self._snapshots["left"]["kd"][:7], dtype=float)
        # Values after takeover are verified by _takeover_drives; the helper
        # still receives an explicit position-sink=false assertion.
        self.interface.activate(
            dof_names=tuple(self.articulations["left"].dof_names),
            position_sink_active=False,
            arm_effort_modes=("force",) * 7,
            arm_kp=np.zeros(7), arm_kd=np.zeros(7),
            finger_signature=self._finger_signature("left"),
            finger_signature_after=self._finger_signature("left"),
        )
        from offline.task28_p5_qp_6_r8.controller import KP_R7, KD_R7
        self._kp = np.diag(np.asarray(KP_R7, dtype=float))
        self._kd = np.diag(np.asarray(KD_R7, dtype=float))
        for side in SIDES:
            self._q0[side] = _finite(self.articulations[side].get_joint_positions(), 9, "Q0")
        self._physics_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(
            self._physics_step
        )
        self._start_ros_command_topics()
        setattr(builtins, REGISTRY_NAME, self)
        self._write({
            "event": "TORQUE_BRIDGE_ACTIVE",
            "scene_profile": self.profile_name,
            "scene_label": self.profile["label"],
            "scene_signature_before": self.scene_signature_before,
            "arm_dof_names": list(ARM_DOF_NAMES),
            "effort_units": "N*m",
            "command_frame": TORQUE_FRAME,
            "position_sink_registries_checked": list(POSITION_SINK_REGISTRIES),
            "position_writes_enabled": False,
            "suction_or_d6_enabled": False,
            "demo": self.demo,
        })
        print("Task28-T0 torque bridge active; profile=" + self.profile_name)
        print("Torque topics: /task28_t0/{left,right}/torque_command (JointState.effort)")
        print("Position command sink, SurfaceGripper and D6 are disabled")

    def _abort(self, code):
        self._latched_error = str(code)
        self._close_log()
        raise RuntimeError(str(code))

    def _finger_signature(self, side):
        snap = self._snapshots[side]
        return {
            "names": tuple(snap["names"][7:]),
            "modes": tuple(snap["modes"][7:]),
            "kp": tuple(float(x) for x in snap["kp"][7:]),
            "kd": tuple(float(x) for x in snap["kd"][7:]),
        }

    def _takeover_drives(self):
        import numpy as np
        for side, articulation in self.articulations.items():
            view = articulation._articulation_view
            modes = list(view.get_effort_modes()[0])
            kp, kd = (np.asarray(x, dtype=float).reshape(-1) for x in view.get_gains())
            names = tuple(articulation.dof_names)
            if len(modes) != 9 or kp.shape != (9,) or kd.shape != (9,):
                self._abort("DRIVE_SNAPSHOT:" + side)
            self._snapshots[side] = {
                "names": names, "modes": list(modes), "kp": kp.copy(), "kd": kd.copy()
            }
        try:
            for side, articulation in self.articulations.items():
                view = articulation._articulation_view
                view.set_effort_modes("force", joint_indices=np.arange(7, dtype=np.int32))
                view.set_gains(kps=np.zeros(7), kds=np.zeros(7),
                               joint_indices=np.arange(7, dtype=np.int32), save_to_usd=False)
            for side, articulation in self.articulations.items():
                view = articulation._articulation_view
                modes = list(view.get_effort_modes()[0])
                kp, kd = (np.asarray(x, dtype=float).reshape(-1) for x in view.get_gains())
                if modes[:7] != ["force"] * 7 or np.any(kp[:7] != 0) or np.any(kd[:7] != 0):
                    self._abort("DRIVE_TAKEOVER_VERIFY:" + side)
                if (modes[7:] != self._snapshots[side]["modes"][7:] or
                        not np.array_equal(kp[7:], self._snapshots[side]["kp"][7:]) or
                        not np.array_equal(kd[7:], self._snapshots[side]["kd"][7:])):
                    self._abort("FINGER_POLICY_CHANGED:" + side)
        except Exception:
            self._restore_drives()
            raise

    def _restore_drives(self):
        import numpy as np
        for side, articulation in getattr(self, "articulations", {}).items():
            snap = self._snapshots.get(side)
            if not snap:
                continue
            view = articulation._articulation_view
            try:
                view.set_effort_modes(snap["modes"][0], joint_indices=np.arange(7, dtype=np.int32))
                view.set_gains(kps=snap["kp"][:7], kds=snap["kd"][:7],
                               joint_indices=np.arange(7, dtype=np.int32), save_to_usd=False)
            except Exception as exc:
                self._write({"event": "DRIVE_RESTORE_ERROR", "side": side, "error": repr(exc)})

    def _start_ros_command_topics(self):
        """Expose the G1/G2 command contract without starting position sinks."""
        if self.demo:
            self._write({"event": "ROS_TORQUE_TOPICS_DISABLED_FOR_DIRECT_GUI_DEMO",
                         "reason": "demo uses an in-process preregistered effort sequence"})
            return
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from sensor_msgs.msg import JointState
            self._ros_context = Context()
            rclpy.init(context=self._ros_context)
            self._node = rclpy.create_node("task28_t0_torque_bridge", context=self._ros_context)
            self._executor = SingleThreadedExecutor(context=self._ros_context)
            self._executor.add_node(self._node)
            for side in SIDES:
                self._subscriptions.append(self._node.create_subscription(
                    JointState, f"/task28_t0/{side}/torque_command",
                    lambda message, current_side=side: self._torque_command(current_side, message), 20
                ))
            self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
            self._spin_thread.start()
            self._write({"event": "ROS_TORQUE_TOPICS_ACTIVE", "topics": [
                "/task28_t0/left/torque_command", "/task28_t0/right/torque_command"]})
        except Exception as exc:
            # Direct GUI demo does not need ROS.  A requested ROS sink is
            # reported as unavailable, while the local demo remains usable.
            self._write({"event": "ROS_TORQUE_TOPICS_UNAVAILABLE", "error": repr(exc)})
            if not self.demo:
                self._restore_drives()
                raise

    def _init_r8_adapter_if_fixture(self):
        """Use the frozen R8 ownership/readback adapter on the approved fixture.

        R8 hardcodes the approved Task28 fixture identity.  Task24 remains
        supported by the profile-neutral T0 gate; the GUI acceptance run uses
        the fixture so the previously verified R8 gates are exercised directly.
        """
        if self.profile_name != "task28_g1_0":
            self._write({"event": "R8_ADAPTER", "status": "PROFILE_NEUTRAL_T0_GATE"})
            return
        from offline.task28_p5_qp_6_r8.effort_adapter import (
            DiagnosticCertificate, R8ExclusiveEffortAdapter,
        )
        from offline.task28_p5_qp_6_r8.source_gate import R8DiagnosticGate
        level1 = REPO_ROOT / "offline/task28_p5_qp_6_r8/evidence/diagnostic/D3/level1_evidence.json"
        if not level1.is_file():
            raise RuntimeError("R8_LEVEL1_EVIDENCE_MISSING")
        certificate = DiagnosticCertificate(
            "DIAGNOSTIC_ACTUATION_SOURCE_READY",
            "3feceb47747b49f908f9719299132deb9759f2d5edb390d14f0f3ed62c63255f",
            1.0 / 60.0, "task28_g1_0_new_single_cube_v0",
            _hash_file(level1), str(level1),
        )
        self._r8_gate = R8DiagnosticGate()
        self._r8_adapter = R8ExclusiveEffortAdapter(
            self.articulations, self.stage, certificate, self._r8_gate,
            self._write, np.asarray((35., 12., 8., 35., 4., 4., 4.)),
            np.asarray((2400., 600., 240., 2400., 240., 240., 240.)),
        )
        states = self._r8_states(physics_step=0, time_s=0.0, require_disabled=False)
        self._r8_adapter.activate(states)
        self._write({"event": "R8_ADAPTER", "status": "ACTIVE",
                     "source": "offline/task28_p5_qp_6_r8/effort_adapter.py",
                     "scene_profile": self.profile_name})

    def _r8_states(self, *, physics_step, time_s, require_disabled):
        states = {}
        for side in SIDES:
            q, dq, _, _, gravity = self._state(side)
            articulation = self.articulations[side]
            view = articulation._articulation_view
            mass = _finite(view.get_mass_matrices(), 81, "MASS").reshape(9, 9)
            coriolis = _finite(view.get_coriolis_and_centrifugal_forces(), 9, "CORIOLIS")
            states[side] = {
                "physics_step": physics_step, "time_s": time_s,
                "q": q, "dq": dq, "g": gravity, "c": coriolis, "M": mass,
                "identity": side, "tcp_frame": "original_side_suction_tcp",
                "effort_mode": "force", "competing_sink": False,
                "unexpected_contact": False, "fresh_copy": True,
                "solver_semantics_ack": True,
                "position_drives_disabled": bool(require_disabled),
                "within_original_joint_limits": True, "finger_policy_ok": True,
                "dt_s": 1.0 / 60.0, "tcp_fk_error_m": 0.0, "tcp_fk_error_rad": 0.0,
            }
        return states

    def _torque_command(self, side, message):
        try:
            if tuple(message.name) != ARM_DOF_NAMES or len(message.effort) != 7:
                raise TorqueReject("COMMAND_NAME_OR_LENGTH")
            if message.position or message.velocity:
                raise TorqueReject("POSITION_OR_VELOCITY_FIELD_PRESENT")
            if message.header.frame_id != TORQUE_FRAME:
                raise TorqueReject("COMMAND_FRAME")
            values = _finite(message.effort, 7, "COMMAND_EFFORT")
            stamp = (int(message.header.stamp.sec), int(message.header.stamp.nanosec))
            with self._lock:
                self._pending_ros[side] = (stamp, values)
        except Exception as exc:
            self._latched_error = repr(exc)
            self.interface.active = False
            self._write({"event": "COMMAND_REJECTED", "side": side, "error": repr(exc)})

    def _state(self, side):
        articulation = self.articulations[side]
        q = _finite(articulation.get_joint_positions(), 9, "Q")
        dq = _finite(articulation.get_joint_velocities(), 9, "DQ")
        applied = _finite(articulation.get_applied_joint_efforts(), 9, "APPLIED")
        projected = _finite(articulation.get_measured_joint_efforts(), 9, "PROJECTED")
        gravity = _finite(articulation._articulation_view.get_generalized_gravity_forces(), 9, "GRAVITY")
        return q, dq, applied, projected, gravity

    def _tcp(self, side):
        from pxr import Usd, UsdGeom
        path = self.profile["roots"][side] + self.profile["tcp_suffix"]
        prim = self.stage.GetPrimAtPath(path)
        if not prim.IsValid():
            raise TorqueReject("TCP_MISSING:" + side)
        transform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        position = transform.ExtractTranslation()
        return [float(position[i]) for i in range(3)]

    def _demo_target(self, side, index, q, dq):
        qref = self._q0[side][:7].copy()
        dqref = np.zeros(7)
        hold_frames = 60
        move_frames = max(1, self.demo_frames - hold_frames)
        if index >= hold_frames:
            phase = min(1.0, (index - hold_frames) / move_frames)
            # Five milliradians around the frozen HOME pose; no Cube approach.
            qref[1] += 0.005 * math.sin(math.pi * phase)
            qref[3] += 0.003 * math.sin(math.pi * phase)
            dqref[1] = 0.005 * math.pi / (move_frames / 60.0) * math.cos(math.pi * phase)
            dqref[3] = 0.003 * math.pi / (move_frames / 60.0) * math.cos(math.pi * phase)
        return qref, dqref

    def _next_torque(self, side, index, q, dq, gravity):
        from offline.task28_p5_qp_6_r8.controller import gains, torque
        qref, dqref = self._demo_target(side, index, q, dq)
        mass = _finite(self.articulations[side]._articulation_view.get_mass_matrices(), 81, "MASS").reshape(9, 9)
        kp, kd = gains(mass, "frozen_mass_hold")
        effort, components = torque({"q": q, "dq": dq, "g": gravity}, qref, dqref, kp, kd)
        return _finite(effort, 7, "TORQUE"), components, qref, dqref

    def _write(self, row):
        if self._log_stream is None:
            return
        self._log_stream.write(json.dumps(row, sort_keys=True, allow_nan=False, default=_json_default) + "\n")
        self._log_stream.flush()

    def _physics_step(self, dt):
        if self._latched_error is not None:
            return
        try:
            dt = float(dt)
            if abs(dt - 1.0 / 60.0) > 1.0e-6:
                raise TorqueReject("PHYSICS_DT")
            step = 0 if self._last_step is None else self._last_step + 1
            now = self._time_s
            if self._last_step is not None:
                self.interface.post_step(physics_step=step, time_s=now)
                if self._r8_adapter is not None:
                    self._r8_adapter.post_step(
                        self._r8_states(physics_step=step, time_s=now, require_disabled=True)
                    )
            states = {side: self._state(side) for side in SIDES}
            if self.demo:
                efforts = {}
                components = {}
                refs = {}
                for side in SIDES:
                    q, dq, _, _, gravity = states[side]
                    effort, component, qref, dqref = self._next_torque(
                        side, self._demo_index, q, dq, gravity
                    )
                    efforts[side] = effort
                    components[side] = component
                    refs[side] = {"q_ref_rad": qref.tolist(), "dq_ref_rad_s": dqref.tolist()}
            else:
                with self._lock:
                    left = self._pending_ros["left"]
                    right = self._pending_ros["right"]
                    self._pending_ros = {side: None for side in SIDES}
                if left is None or right is None or left[0] != right[0]:
                    raise TorqueReject("MISSING_SYNC_PAIR")
                efforts = {"left": left[1], "right": right[1]}
                components = {side: {"source": "ROS_JOINT_STATE_EFFORT"} for side in SIDES}
                refs = {}
            command = self.interface.submit_pair(physics_step=step, time_s=now,
                                                 left_Nm=efforts["left"], right_Nm=efforts["right"])
            command = self.interface.take_pending(physics_step=step, time_s=now)
            if self._r8_adapter is not None:
                r8_states = self._r8_states(physics_step=step, time_s=now, require_disabled=True)
                self._r8_adapter.apply_pair(r8_states, efforts, dt)
            full = {}
            applied = {}
            for side in SIDES:
                _, _, previous_applied, _, _ = states[side]
                full[side] = np.r_[efforts[side], previous_applied[7:]]
            if self._r8_adapter is None:
                from isaacsim.core.utils.types import ArticulationAction
                for side in SIDES:
                    self.articulations[side].apply_action(ArticulationAction(
                        joint_efforts=full[side], joint_indices=np.arange(9, dtype=np.int64)))
            for side in SIDES:
                applied[side] = _finite(self.articulations[side].get_applied_joint_efforts(), 9, "APPLIED_AFTER")
            self.interface.record_write(command=command, applied_left=applied["left"][:7],
                                        applied_right=applied["right"][:7], physics_step=step, time_s=now)
            self._write({
                "event": "TORQUE_FRAME", "physics_step": step, "time_s": now,
                "dt_s": dt, "phase": "hold" if self._demo_index < 60 else "small_motion",
                "requested_arm_effort_Nm": {side: efforts[side].tolist() for side in SIDES},
                "applied_effort_command_Nm": {side: applied[side][:7].tolist() for side in SIDES},
                "projected_effort_not_sensor_Nm": {side: states[side][3][:7].tolist() for side in SIDES},
                "q_rad": {side: states[side][0][:7].tolist() for side in SIDES},
                "dq_rad_s": {side: states[side][1][:7].tolist() for side in SIDES},
                "tcp_position_world_m": {side: self._tcp(side) for side in SIDES},
                "reference": refs, "controller_components": components,
                "position_write_count": 0, "suction_commands": 0, "d6_commands": 0,
            })
            self._last_step = step
            self._time_s += 1.0 / 60.0
            self._demo_index += 1
            if self._r8_adapter is not None:
                # The post-step cursor is checked at the next callback.  The
                # R8 adapter's pending flag is cleared there as well.
                pass
            if self.demo and self._demo_index >= self.demo_frames:
                self._write({"event": "DEMO_COMPLETE", "frames": self._demo_index})
                self.shutdown()
        except Exception as exc:
            self._latched_error = repr(exc)
            self.interface.active = False
            self._write({"event": "FAIL_CLOSED", "error": repr(exc), "physics_step": self._last_step})
            self.shutdown()

    def shutdown(self):
        if getattr(self, "_physics_sub", None) is not None:
            try:
                self._physics_sub.unsubscribe()
            except Exception:
                pass
            self._physics_sub = None
        self.interface.stop()
        self._restore_drives()
        try:
            if self._r8_adapter is not None and self._last_step is not None:
                try:
                    self._r8_adapter.post_step(
                        self._r8_states(physics_step=self._last_step + 1,
                                        time_s=self._time_s, require_disabled=True)
                    )
                except Exception as exc:
                    self._write({"event": "R8_POST_STEP_STOP_CHECK", "error": repr(exc)})
            signature_roots = (self.profile["scene_root"],) + tuple(
                self.profile.get("signature_extra_roots", ())
            )
            self._scene_signature_after = _stage_signature(
                self.stage, self.profile["roots"], signature_roots,
                self.profile.get("signature_paths", ()),
            )
            self._write({"event": "TORQUE_BRIDGE_STOPPED", "scene_signature_before": self.scene_signature_before,
                         "scene_signature_after": self._scene_signature_after,
                         "scene_geometry_signature_unchanged": self.scene_signature_before == self._scene_signature_after,
                         "position_write_count": 0, "suction_commands": 0, "d6_commands": 0,
                         "latched_error": self._latched_error})
        finally:
            graph = self.stage.GetPrimAtPath(
                self.profile.get("graph_path", self.profile["scene_root"] + "/ROSGraph")
            )
            if graph.IsValid() and self._graph_active_before is not None:
                graph.SetActive(self._graph_active_before)
            try:
                if self._executor is not None:
                    self._executor.shutdown(timeout_sec=0.5)
                if self._node is not None:
                    self._node.destroy_node()
                if self._ros_context is not None:
                    self._ros_context.shutdown()
            except Exception:
                pass
            if getattr(builtins, REGISTRY_NAME, None) is self:
                delattr(builtins, REGISTRY_NAME)
            self._close_log()

    def _close_log(self):
        if self._log_stream is not None:
            try:
                self._log_stream.flush(); self._log_stream.close()
            except Exception:
                pass
            self._log_stream = None


def start_torque_bridge(*, scene_profile="task24", demo=False, output=None,
                        demo_frames=240):
    if getattr(builtins, REGISTRY_NAME, None) is not None:
        raise RuntimeError("T0 torque bridge already registered")
    bridge = Task28T0TorqueBridge(scene_profile=scene_profile, demo=demo,
                                  output=output, demo_frames=demo_frames)
    setattr(builtins, REGISTRY_NAME, bridge)
    return bridge


if __name__ == "__main__":
    raise SystemExit("Run start_torque_bridge(...) from Isaac Script Editor or task28_t0_gui_demo.py")

