"""Small, deterministic source-validity gate used by R4 negative controls.

It is intentionally a policy module: it does not issue actuator commands and
does not substitute finite differences for a reported velocity.
"""
from __future__ import annotations

import numpy as np


class SourceRejected(RuntimeError):
    pass


class LatchedSourceGate:
    def __init__(self):
        self.rejected = False
        self.reason = None

    def reject(self, reason):
        self.rejected = True
        self.reason = reason
        raise SourceRejected(reason)

    def validate(self, current, previous=None, expected_identity=None, expected_dt=1.0 / 60.0):
        if self.rejected:
            raise SourceRejected("LATCHED_REJECTION:" + str(self.reason))
        if expected_identity and current.get("identity") != expected_identity:
            self.reject("WRONG_ROBOT_IDENTITY")
        if current.get("position_sink_active"):
            self.reject("POSITION_SINK_ACTIVE")
        if current.get("effort_mode") not in (None, "force"):
            self.reject("JOINT_EFFORT_MODE_MISMATCH")
        if current.get("unexpected_collision"):
            self.reject("UNEXPECTED_COLLISION")
        M = np.asarray(current.get("M", []), float)
        if M.ndim != 2 or M.shape[0] != M.shape[1] or not np.isfinite(M).all() or np.any(np.diag(M) <= 0):
            self.reject("INVALID_MASS_SOURCE")
        if current.get("tcp_frame") != "original_side_suction_tcp":
            self.reject("WRONG_TCP_FRAME")
        step = current.get("physics_step")
        time = current.get("time_s")
        if previous is not None:
            if step != previous.get("physics_step", -1) + 1:
                self.reject("WRONG_PHYSICS_STEP")
            dt = float(time) - float(previous.get("time_s"))
            if dt <= 0 or abs(dt - expected_dt) > 1e-8:
                self.reject("WRONG_PHYSICS_TIME")
            if np.array_equal(np.asarray(current.get("q")), np.asarray(previous.get("q"))) and np.max(np.abs(np.asarray(current.get("dq", []), float))) > 0.01:
                self.reject("STALE_Q_NONZERO_DQ")
            if np.array_equal(np.asarray(current.get("dq")), np.asarray(previous.get("dq"))) and np.max(np.abs(np.asarray(current.get("q", []), float) - np.asarray(previous.get("q", []), float))) > 1e-8:
                self.reject("STALE_DQ")
        q = np.asarray(current.get("q", []), float)
        dq = np.asarray(current.get("dq", []), float)
        if q.size == 0 or dq.size != q.size or not np.isfinite(q).all() or not np.isfinite(dq).all():
            self.reject("STALE_OR_INVALID_STATE")
        if current.get("shared_mutable_buffer"):
            self.reject("SHARED_MUTABLE_BUFFER")
        return True

    def require_not_rejected(self):
        if self.rejected:
            raise SourceRejected("LATCHED_REJECTION:" + str(self.reason))
        return True


def torque_attempt(gate: LatchedSourceGate):
    """The only sanctioned torque test hook: a rejected gate writes nothing."""
    gate.require_not_rejected()
    return {"commands": 0, "executed": False}
