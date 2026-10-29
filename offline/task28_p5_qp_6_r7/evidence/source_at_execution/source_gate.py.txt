"""R6 task-specific Level-1 gates and fault-latch behavior."""
from __future__ import annotations
import numpy as np


class Rejected(RuntimeError):
    pass


class R6DiagnosticGate:
    def __init__(self, dt=1 / 60):
        self.dt = float(dt)
        self.latched = False
        self.reason = None

    def reject(self, reason):
        self.latched = True
        self.reason = str(reason)
        raise Rejected(self.reason)

    def validate_pair(self, states, require_disabled=True):
        if self.latched:
            raise Rejected("LATCHED:" + str(self.reason))
        if set(states) != {"left", "right"}:
            self.reject("STATE_PAIR")
        left, right = states["left"], states["right"]
        if left.get("physics_step") != right.get("physics_step") or left.get("time_s") != right.get("time_s"):
            self.reject("TIME_SYNC")
        for side in ("left", "right"):
            s = states[side]
            if s.get("identity") != side:
                self.reject("WRONG_ARM_IDENTITY")
            if s.get("competing_sink"):
                self.reject("COMPETING_POSITION_SINK")
            if s.get("unexpected_contact"):
                self.reject("UNEXPECTED_COLLISION")
            if s.get("effort_mode") != "force":
                self.reject("INVALID_EFFORT_MODE")
            if s.get("tcp_frame") != "original_side_suction_tcp":
                self.reject("WRONG_TCP_FRAME")
            if s.get("fresh_copy") is not True:
                self.reject("STALE_OR_ALIASED_STATE")
            if s.get("solver_semantics_ack") is not True:
                self.reject("TGS_SEMANTICS_UNDECLARED")
            if abs(float(s.get("dt_s", self.dt)) - self.dt) > 1e-8:
                self.reject("PHYSICS_DT")
            if require_disabled and not s.get("position_drives_disabled", False):
                self.reject("POSITION_DRIVE_ACTIVE")
            if s.get("finger_policy_ok") is not True:
                self.reject("FINGER_POLICY_CHANGED")
            if s.get("tcp_fk_error_m", 1.0) > 2e-5 or s.get("tcp_fk_error_rad", 1.0) > 2e-5:
                self.reject("FK_TCP_MISMATCH")
            for key in ("q", "dq", "g", "c"):
                x = np.asarray(s.get(key), dtype=float)
                if x.shape != (9,) or not np.isfinite(x).all():
                    self.reject("INVALID_" + key.upper())
            m = np.asarray(s.get("M"), dtype=float)
            if m.shape != (9, 9) or not np.isfinite(m).all() or not np.allclose(m, m.T, atol=1e-5, rtol=0):
                self.reject("INVALID_MASS")
            if np.linalg.eigvalsh((m + m.T) / 2).min() <= 0:
                self.reject("NON_POSITIVE_MASS")
        return {"valid": True, "level": "DIAGNOSTIC_ACTUATION_SOURCE_READY", "fd_role": "ESTIMATOR_ONLY", "velocity_semantics": "END_STEP_SOLVER_VELOCITY"}

    def require(self):
        if self.latched:
            raise Rejected("LATCHED:" + str(self.reason))
        return True

    @staticmethod
    def validate_effort_request(requested, previous, expected_sign, dt=1 / 60, limits=45.0, rates=2400.0, units="Nm"):
        if units != "Nm":
            raise Rejected("INVALID_EFFORT_UNITS")
        u = np.asarray(requested, dtype=float)
        p = np.asarray(previous, dtype=float)
        sign = np.asarray(expected_sign, dtype=float)
        if u.shape != (7,) or p.shape != (7,) or sign.shape != (7,) or not np.isfinite(u).all():
            raise Rejected("INVALID_EFFORT_VECTOR")
        if np.any(np.sign(u - p) * sign < 0):
            raise Rejected("INCORRECT_TORQUE_SIGN")
        if np.any(np.abs(u) > limits) or np.any(np.abs(u - p) > rates * dt):
            raise Rejected("EFFORT_OR_RATE")
        return True
