"""Fail-closed, seven-arm-DOF torque handoff used by the T0 bridge.

The interface deliberately has no Isaac, ROS, USD, or hardware dependency.
It is the boundary used by the GUI bridge and by future G1/G2 callers.  A
command is a pair of seven-joint arm torques in N*m.  Finger DOFs remain under
their native scene policy and are never part of this public algorithm API.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Sequence

import numpy as np


ARM_DOF_NAMES = tuple(f"fr3_joint{i}" for i in range(1, 8))
FINGER_DOF_NAMES = ("fr3_finger_joint1", "fr3_finger_joint2")


class TorqueReject(RuntimeError):
    """A source, ownership, timing, or command gate rejected the operation."""


def _vector(value: object, size: int, code: str) -> np.ndarray:
    try:
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError) as exc:
        raise TorqueReject(code) from exc
    if result.shape != (size,) or not np.isfinite(result).all():
        raise TorqueReject(code)
    return result.copy()


def _scalar(value: object, code: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise TorqueReject(code) from exc
    if not math.isfinite(result):
        raise TorqueReject(code)
    return result


@dataclass(frozen=True)
class TorqueLimits:
    """Frozen R8 arm limits, applied without altering scene/USD limits."""

    abs_Nm: tuple[float, ...] = (35.0, 12.0, 8.0, 35.0, 4.0, 4.0, 4.0)
    rate_Nm_s: tuple[float, ...] = (2400.0, 600.0, 240.0, 2400.0, 240.0, 240.0, 240.0)
    dt_s: float = 1.0 / 60.0
    dt_tolerance_s: float = 1.0e-8

    def validate(self) -> None:
        abs_limit = _vector(self.abs_Nm, 7, "INVALID_ABS_LIMIT")
        rate = _vector(self.rate_Nm_s, 7, "INVALID_RATE_LIMIT")
        dt = _scalar(self.dt_s, "INVALID_DT")
        tol = _scalar(self.dt_tolerance_s, "INVALID_DT_TOLERANCE")
        if (np.any(abs_limit <= 0.0) or np.any(rate <= 0.0) or dt <= 0.0 or
                tol < 0.0 or tol >= dt):
            raise TorqueReject("INVALID_LIMITS")


@dataclass(frozen=True)
class TorqueCommand:
    """A synchronized pair accepted for one future physics step."""

    physics_step: int
    time_s: float
    left_Nm: tuple[float, ...]
    right_Nm: tuple[float, ...]
    frame_id: str
    units: str = "N*m"
    source: str = "task28_t0_torque_interface"


class TorqueInterface:
    """Stateful pair gate; every failed operation is fail-closed."""

    def __init__(self, *, limits: TorqueLimits | None = None,
                 arm_dof_names: Sequence[str] = ARM_DOF_NAMES):
        self.limits = limits or TorqueLimits()
        self.limits.validate()
        self.arm_dof_names = tuple(arm_dof_names)
        if self.arm_dof_names != ARM_DOF_NAMES:
            raise TorqueReject("ARM_DOF_ORDER")
        self.active = False
        self.latched_failure: str | None = None
        self._pending: TorqueCommand | None = None
        self._last_written_command: TorqueCommand | None = None
        self._last_command: np.ndarray | None = None
        self._last_step: int | None = None
        self._last_time: float | None = None
        self._writes = 0

    @property
    def last_command(self) -> TorqueCommand | None:
        return self._last_written_command

    @property
    def pending_command(self) -> TorqueCommand | None:
        return self._pending

    @property
    def last_cursor(self) -> tuple[int, float] | None:
        if self._last_step is None or self._last_time is None:
            return None
        return self._last_step, self._last_time

    @property
    def writes(self) -> int:
        return self._writes

    def _fail(self, code: str):
        self.latched_failure = str(code)
        self._pending = None
        self.active = False
        raise TorqueReject(code)

    def activate(self, *, dof_names: Sequence[str], position_sink_active: bool,
                 arm_effort_modes: Sequence[str], arm_kp: object, arm_kd: object,
                 finger_signature: Mapping[str, object] | None = None,
                 finger_signature_after: Mapping[str, object] | None = None) -> None:
        """Open the runtime handoff after ownership checks.

        ``position_sink_active`` must already be false.  The bridge changes
        arm drive type/gains in memory and passes the resulting values here;
        no stage save is performed.  Finger signatures are compared when both
        snapshots are provided.
        """
        if self.latched_failure is not None:
            raise TorqueReject("LATCHED:" + self.latched_failure)
        names = tuple(dof_names)
        if names != ARM_DOF_NAMES + FINGER_DOF_NAMES:
            self._fail("DOF_ORDER")
        if bool(position_sink_active):
            self._fail("POSITION_SINK_ACTIVE")
        modes = tuple(arm_effort_modes)
        if modes != ("force",) * 7:
            self._fail("ARM_EFFORT_MODE")
        kp = _vector(arm_kp, 7, "ARM_KP")
        kd = _vector(arm_kd, 7, "ARM_KD")
        if np.any(kp != 0.0) or np.any(kd != 0.0):
            self._fail("ARM_DRIVE_ACTIVE")
        if (finger_signature is not None and finger_signature_after is not None and
                dict(finger_signature) != dict(finger_signature_after)):
            self._fail("FINGER_POLICY_CHANGED")
        self.active = True
        self._pending = None

    def submit_pair(self, *, physics_step: int, time_s: float,
                    left_Nm: object, right_Nm: object,
                    frame_id: str = "task28_t0_dual_sync",
                    units: str = "N*m") -> TorqueCommand:
        if self.latched_failure is not None:
            raise TorqueReject("LATCHED:" + self.latched_failure)
        if not self.active:
            raise TorqueReject("INTERFACE_INACTIVE")
        if self._pending is not None:
            self._fail("PENDING_COMMAND")
        if not isinstance(physics_step, (int, np.integer)) or physics_step < 0:
            self._fail("PHYSICS_STEP")
        t = _scalar(time_s, "TIME")
        if t < 0.0:
            self._fail("TIME")
        if frame_id != "task28_t0_dual_sync":
            self._fail("SYNC_FRAME")
        if units != "N*m":
            self._fail("UNITS")
        left = _vector(left_Nm, 7, "LEFT_TORQUE")
        right = _vector(right_Nm, 7, "RIGHT_TORQUE")
        limits = np.asarray(self.limits.abs_Nm, dtype=float)
        if np.any(np.abs(left) > limits) or np.any(np.abs(right) > limits):
            self._fail("TORQUE_ABS_LIMIT")
        if self._last_step is not None:
            expected_step = self._last_step + 1
            expected_time = self._last_time + self.limits.dt_s
            if int(physics_step) != expected_step:
                self._fail("COMMAND_CURSOR")
            if abs(t - expected_time) > self.limits.dt_tolerance_s:
                self._fail("COMMAND_TIME")
            previous = self._last_command
            if previous is None:
                self._fail("COMMAND_STATE")
            rate = np.asarray(self.limits.rate_Nm_s, dtype=float)
            if (np.any(np.abs(left - previous[:7]) > rate * self.limits.dt_s) or
                    np.any(np.abs(right - previous[7:]) > rate * self.limits.dt_s)):
                self._fail("TORQUE_RATE_LIMIT")
        command = TorqueCommand(int(physics_step), t, tuple(left), tuple(right), frame_id, units)
        self._pending = command
        return command

    def take_pending(self, *, physics_step: int, time_s: float) -> TorqueCommand:
        if self.latched_failure is not None:
            raise TorqueReject("LATCHED:" + self.latched_failure)
        if not self.active:
            raise TorqueReject("INTERFACE_INACTIVE")
        if self._pending is None:
            self._fail("MISSING_PAIR")
        t = _scalar(time_s, "TIME")
        if int(physics_step) != self._pending.physics_step or abs(t - self._pending.time_s) > self.limits.dt_tolerance_s:
            self._fail("PENDING_CURSOR")
        command = self._pending
        self._pending = None
        return command

    def record_write(self, *, command: TorqueCommand, applied_left: object,
                     applied_right: object, physics_step: int, time_s: float) -> None:
        if self.latched_failure is not None:
            raise TorqueReject("LATCHED:" + self.latched_failure)
        if not isinstance(physics_step, (int, np.integer)):
            self._fail("WRITE_CURSOR")
        step = int(physics_step)
        t = _scalar(time_s, "WRITE_TIME")
        if step != command.physics_step or abs(t - command.time_s) > self.limits.dt_tolerance_s:
            self._fail("WRITE_CURSOR")
        left = _vector(applied_left, 7, "APPLIED_LEFT")
        right = _vector(applied_right, 7, "APPLIED_RIGHT")
        requested = np.r_[command.left_Nm, command.right_Nm]
        applied = np.r_[left, right]
        if not np.allclose(applied, requested, atol=1.0e-6, rtol=0.0):
            self._fail("APPLIED_READBACK")
        self._last_command = requested.copy()
        self._last_written_command = command
        self._last_step = command.physics_step
        self._last_time = command.time_s
        self._writes += 1

    def post_step(self, *, physics_step: int, time_s: float) -> None:
        if self.latched_failure is not None:
            raise TorqueReject("LATCHED:" + self.latched_failure)
        if self._last_step is None or self._last_time is None:
            self._fail("POST_WITHOUT_WRITE")
        if not isinstance(physics_step, (int, np.integer)):
            self._fail("POST_STEP_CURSOR")
        step = int(physics_step)
        t = _scalar(time_s, "POST_STEP_TIME")
        if step != self._last_step + 1 or abs(t - (self._last_time + self.limits.dt_s)) > self.limits.dt_tolerance_s:
            self._fail("POST_STEP_CURSOR")

    def stop(self) -> None:
        self.active = False
        self._pending = None

