"""Live, bounded diagnostic effort adapter for a fresh original-scene stage.

The full-buffer write and finger-preservation behavior follows the R3 adapter,
but its certificate is deliberately the R6 Level-1 diagnostic certificate.  It
does not claim calibrated torque sensing or complete inverse dynamics.
"""
from __future__ import annotations
import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
import numpy as np

from offline.task28_p5_qp_6_r2.common import DOF_NAMES, FR3_SHA256, LEFT_ROOT, RIGHT_ROOT, TASK_ROOT


class Reject(RuntimeError):
    pass


def vec(value, shape, label):
    x = np.asarray(value, dtype=float)
    if x.shape != shape or not np.isfinite(x).all():
        raise Reject("INVALID_" + label)
    return x


@dataclass(frozen=True)
class DiagnosticCertificate:
    status: str
    asset_sha256: str
    dt_s: float
    runtime_scene_identity: str
    evidence_sha256: str
    evidence_path: str

    def require_level1(self):
        if self.status != "DIAGNOSTIC_ACTUATION_SOURCE_READY":
            raise Reject("LEVEL1_REQUIRED")
        if self.asset_sha256 != FR3_SHA256 or abs(self.dt_s - 1 / 60) > 1e-8:
            raise Reject("CERTIFICATE_ID_OR_DT")
        if self.runtime_scene_identity != "task28_g1_0_new_single_cube_v0":
            raise Reject("CERTIFICATE_SCENE")
        if len(self.evidence_sha256) != 64 or any(c not in "0123456789abcdef" for c in self.evidence_sha256):
            raise Reject("CERTIFICATE_HASH")
        try:
            raw = Path(self.evidence_path).read_bytes()
            evidence = json.loads(raw)
        except (OSError, ValueError, TypeError) as exc:
            raise Reject("CERTIFICATE_EVIDENCE") from exc
        if hashlib.sha256(raw).hexdigest() != self.evidence_sha256:
            raise Reject("CERTIFICATE_EVIDENCE_HASH")
        if evidence.get("status") != self.status or evidence.get("level") != 1:
            raise Reject("CERTIFICATE_EVIDENCE_STATUS")


class R7ExclusiveEffortAdapter:
    """Own both arm command sinks with per-joint source and research limits."""

    def __init__(self, arms, stage, certificate, gate, journal, limits, rates):
        self.arms = arms
        self.stage = stage
        self.certificate = certificate
        self.gate = gate
        self.journal = journal
        self.limits = vec(limits, (7,), "LIMITS")
        self.rates = vec(rates, (7,), "RATES")
        self.source_limits = np.asarray([87.0,87.0,87.0,87.0,12.0,12.0,12.0])
        if np.any(self.limits > self.source_limits):
            raise Reject("RESEARCH_LIMIT_EXCEEDS_SOURCE")
        if np.any(self.limits <= 0) or np.any(self.rates <= 0):
            raise Reject("INVALID_LIMITS")
        self.enabled = False
        self.pending = False
        self.finger_signature = {}
        self.pre_cursor = None

    def _identity(self):
        self.certificate.require_level1()
        if set(self.arms) != {"left", "right"}:
            raise Reject("PAIR_REQUIRED")
        for side, path in (("left", LEFT_ROOT), ("right", RIGHT_ROOT)):
            arm = self.arms[side]
            if str(arm.prim_path) != path or list(arm.dof_names) != DOF_NAMES or not arm.handles_initialized:
                raise Reject("ARTICULATION_IDENTITY:" + side)
        root = self.stage.GetPrimAtPath(TASK_ROOT)
        if (not root.IsValid() or root.GetCustomDataByKey("task28_g1_0_fixture_id") != "task28_g1_0_new_single_cube_v0"
                or root.GetCustomDataByKey("task28_g1_0_fr3_usd_sha256") != FR3_SHA256):
            raise Reject("SCENE_IDENTITY")
        graph = self.stage.GetPrimAtPath(TASK_ROOT + "/ROSGraph")
        if not graph.IsValid() or graph.IsActive():
            raise Reject("ROS_GRAPH_ACTIVE")

    def _pair(self, states, require_disabled=False):
        if set(states) != {"left", "right"}:
            raise Reject("STATE_PAIR")
        a, b = states["left"], states["right"]
        if a.get("physics_step") != b.get("physics_step") or a.get("time_s") != b.get("time_s"):
            raise Reject("STATE_SYNCHRONIZATION")
        for side in ("left", "right"):
            s = states[side]
            vec(s.get("q"), (9,), "Q")
            vec(s.get("dq"), (9,), "DQ")
            vec(s.get("g"), (9,), "G")
            vec(s.get("c"), (9,), "C")
            mass = vec(s.get("M"), (9, 9), "MASS")
            if not np.allclose(mass, mass.T, atol=1e-5, rtol=0) or np.linalg.eigvalsh((mass + mass.T) / 2).min() <= 0:
                raise Reject("STATE_MASS_MATRIX")
            if s.get("identity") != side or s.get("tcp_frame") != "original_side_suction_tcp":
                raise Reject("STATE_IDENTITY")
            if s.get("effort_mode") != "force":
                raise Reject("INVALID_EFFORT_MODE")
            if s.get("competing_sink") or s.get("unexpected_contact"):
                raise Reject("SOURCE_OR_CONTACT")
            if s.get("fresh_copy") is not True or s.get("solver_semantics_ack") is not True:
                raise Reject("STATE_COPY_OR_SEMANTICS")
            if require_disabled and not s.get("position_drives_disabled"):
                raise Reject("POSITION_DRIVE_ACTIVE")
        verdict = self.gate.validate_pair(states, require_disabled=require_disabled)
        if verdict.get("valid") is not True:
            raise Reject("PRE_COMMAND_GATE")
        return verdict

    def activate(self, states):
        if self.enabled:
            raise Reject("ALREADY_ACTIVE")
        self._identity()
        self._pair(states, require_disabled=False)
        before = {}
        for side in ("left", "right"):
            view = self.arms[side]._articulation_view
            modes = list(view.get_effort_modes()[0])
            kp, kd = (np.asarray(x, dtype=float).reshape(-1) for x in view.get_gains())
            if len(modes) != 9 or modes[:7] != ["force"] * 7 or kp.shape != (9,) or kd.shape != (9,):
                raise Reject("DRIVE_MODE_OR_GAINS")
            before[side] = {"kp": kp.copy(), "kd": kd.copy(), "modes": modes}
        self.journal({"event": "OWNERSHIP_PREPARED", "states": states, "before": before,
                      "level": "DIAGNOSTIC_ACTUATION"})
        try:
            for side in ("left", "right"):
                self.arms[side]._articulation_view.set_gains(
                    kps=np.zeros(7), kds=np.zeros(7),
                    joint_indices=np.arange(7, dtype=np.int32), save_to_usd=False)
            for side in ("left", "right"):
                view = self.arms[side]._articulation_view
                kp, kd = (np.asarray(x, dtype=float).reshape(-1) for x in view.get_gains())
                modes = list(view.get_effort_modes()[0])
                if np.any(kp[:7] != 0) or np.any(kd[:7] != 0) or modes[:7] != ["force"] * 7:
                    raise Reject("ARM_DRIVE_NOT_EXCLUSIVE")
                if not np.array_equal(kp[7:], before[side]["kp"][7:]) or not np.array_equal(kd[7:], before[side]["kd"][7:]) or modes[7:] != before[side]["modes"][7:]:
                    raise Reject("FINGER_DRIVE_CHANGED")
                self.finger_signature[side] = {"kp": kp[7:].copy(), "kd": kd[7:].copy(), "modes": modes[7:]}
        except BaseException:
            self.journal({"event": "OWNERSHIP_TRANSITION_FAILED_ABORT_WITHOUT_STEP"})
            raise
        self.enabled = True
        self.journal({"event": "OWNERSHIP_ACTIVE", "finger_signature": self.finger_signature})

    def apply_pair(self, states, arm_efforts, dt):
        if not self.enabled or self.pending:
            raise Reject("ADAPTER_NOT_READY")
        self._identity()
        self._pair(states, require_disabled=True)
        if abs(float(dt) - 1 / 60) > 1e-8:
            raise Reject("PHYSICS_DT")
        full, previous = {}, {}
        for side in ("left", "right"):
            view = self.arms[side]._articulation_view
            kp, kd = (np.asarray(x, dtype=float).reshape(-1) for x in view.get_gains())
            modes = list(view.get_effort_modes()[0])
            if np.any(kp[:7] != 0) or np.any(kd[:7] != 0) or modes[:7] != ["force"] * 7:
                raise Reject("ARM_DRIVE_NOT_EXCLUSIVE")
            sig = self.finger_signature[side]
            if not np.array_equal(kp[7:], sig["kp"]) or not np.array_equal(kd[7:], sig["kd"]) or modes[7:] != sig["modes"]:
                raise Reject("FINGER_DRIVE_CHANGED")
            prev = vec(self.arms[side].get_applied_joint_efforts(), (9,), "APPLIED_READBACK")
            u = vec(arm_efforts[side], (7,), "ARM_EFFORT")
            if np.any(np.abs(u) > self.limits) or np.any(np.abs(u - prev[:7]) > self.rates * dt):
                raise Reject("EFFORT_OR_RATE")
            previous[side] = prev.copy()
            full[side] = np.r_[u, prev[7:]]
        # This journal is durable before either bilateral write.
        self.journal({"event": "PRE_COMMAND", "states": states, "previous_applied": previous,
                      "full_effort": full, "dt_s": float(dt)})
        try:
            for side in ("left", "right"):
                self.arms[side].set_joint_efforts(full[side], joint_indices=np.arange(9, dtype=np.int32))
            immediate = {}
            for side in ("left", "right"):
                immediate[side] = vec(self.arms[side].get_applied_joint_efforts(), (9,), "IMMEDIATE_READBACK")
                if not np.allclose(immediate[side], full[side], atol=1e-6, rtol=0):
                    raise Reject("IMMEDIATE_READBACK_MISMATCH:" + side)
            self.journal({"event": "IMMEDIATE_READBACK", "readback": immediate, "match": True})
        except BaseException:
            self.enabled = False
            self.journal({"event": "PARTIAL_WRITE_ABORT_NO_STEP"})
            raise
        self.pending = True
        self.pre_cursor = (states["left"]["physics_step"], states["left"]["time_s"])
        return full

    def post_step(self, states):
        if not self.pending:
            raise Reject("NO_PENDING_STEP")
        try:
            self._pair(states, require_disabled=True)
            step, t = self.pre_cursor
            if states["left"]["physics_step"] != step + 1 or abs(states["left"]["time_s"] - t - 1 / 60) > 1e-8:
                raise Reject("POST_STEP_CURSOR")
        except BaseException:
            self.enabled = False
            self.journal({"event": "POST_STEP_GATE_FAILURE", "states": states})
            raise
        self.journal({"event": "POST_STEP", "states": states})
        self.pending = False
