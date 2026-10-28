"""CPU-only, disabled torque adapter prototype for the Task28 scene.

The adapter has no position-command path.  It accepts synchronized full-state
snapshots and returns full nine-DOF effort vectors while the research task
controls only the seven arm joints. Native fingers retain an explicit zero
acceleration assumption is only a mathematical candidate, not a validated
finger hold policy. No Isaac sink exists in this prototype.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping

import numpy as np

from .common import ARM_DOF_NAMES, DOF_NAMES, FINGER_DOF_NAMES, TORQUE_SYNC_FRAME


class AdapterRejected(RuntimeError):
    pass


@dataclass(frozen=True)
class ActuatorOwnership:
    enabled: bool = False
    position_sink_active: bool = False
    owner: str = "none"

    def activate_torque(self) -> "ActuatorOwnership":
        if self.position_sink_active:
            raise AdapterRejected("POSITION_SINK_ACTIVE")
        raise AdapterRejected("ARM_DYNAMICS_SOURCE_UNQUALIFIED")


def _vector(value, size, name):
    array = np.asarray(value, dtype=float)
    if array.shape != (size,) or not np.isfinite(array).all():
        raise AdapterRejected(f"INVALID_{name}")
    return array


def _matrix(value, shape, name):
    array = np.asarray(value, dtype=float)
    if array.shape != shape or not np.isfinite(array).all():
        raise AdapterRejected(f"INVALID_{name}")
    if not np.allclose(array, array.T, atol=1e-8):
        raise AdapterRejected("MASS_NOT_SYMMETRIC")
    if float(np.linalg.eigvalsh(array).min()) <= 1e-8:
        raise AdapterRejected("MASS_NOT_POSITIVE_DEFINITE")
    return array


def validate_snapshot(snapshot: Mapping, *, expected_time=None, expected_step=None) -> dict:
    if list(snapshot.get("names", ())) != DOF_NAMES:
        raise AdapterRejected("DOF_ORDER_OR_NAMES")
    if snapshot.get("effort_modes") != ["force"] * 9:
        raise AdapterRejected("FORCE_MODE_REQUIRED")
    kp = np.asarray(snapshot.get("drive_kp"), dtype=float).reshape(-1)
    kd = np.asarray(snapshot.get("drive_kd"), dtype=float).reshape(-1)
    if kp.shape != (9,) or kd.shape != (9,) or not np.isfinite(kp).all() or not np.isfinite(kd).all():
        raise AdapterRejected("INVALID_DRIVE_GAINS")
    if np.max(np.abs(kp)) > 0:
        raise AdapterRejected("POSITION_KP_NONZERO")
    if np.max(np.abs(kd)) > 0:
        raise AdapterRejected("POSITION_KD_NONZERO")
    t = float(snapshot.get("time_s"))
    step = int(snapshot.get("physics_step"))
    if not math.isfinite(t) or step < 0:
        raise AdapterRejected("INVALID_STATE_CURSOR")
    if expected_time is not None and t != float(expected_time):
        raise AdapterRejected("STATE_TIME_MISMATCH")
    if expected_step is not None and step != int(expected_step):
        raise AdapterRejected("STATE_STEP_MISMATCH")
    q = _vector(snapshot.get("q"), 9, "Q")
    dq = _vector(snapshot.get("dq"), 9, "DQ")
    M = _matrix(snapshot.get("M"), (9, 9), "M")
    c = _vector(snapshot.get("c"), 9, "C")
    g = _vector(snapshot.get("g"), 9, "G")
    return {"time_s": t, "physics_step": step, "q": q, "dq": dq, "M": M, "c": c, "g": g}


def coordinated_effort(snapshot: Mapping, arm_qdd: Mapping[str, float] | np.ndarray,
                       *, finger_qdd=(0.0, 0.0), ownership: ActuatorOwnership | None = None,
                       effort_limits=None, rate_limits=None, previous_effort=None,
                       dt=1.0 / 60.0) -> np.ndarray:
    ownership = ownership or ActuatorOwnership()
    # This version cannot qualify the actual mounted-tool/finger dynamics or
    # enforce the original collision/closure gates in a live scene.
    # It must not expose a bypass through a forged ownership value.
    raise AdapterRejected("ARM_DYNAMICS_SOURCE_UNQUALIFIED")


def synchronized_effort_messages(left: np.ndarray, right: np.ndarray, *, stamp: tuple[int, int]) -> dict:
    """Create the adapter's torque payload; position fields are deliberately absent."""
    left = _vector(left, 9, "LEFT_EFFORT")
    right = _vector(right, 9, "RIGHT_EFFORT")
    if not (isinstance(stamp, tuple) and len(stamp) == 2 and all(isinstance(x, int) for x in stamp)):
        raise AdapterRejected("INVALID_SYNC_STAMP")
    return {
        "frame_id": TORQUE_SYNC_FRAME,
        "stamp": {"sec": stamp[0], "nanosec": stamp[1]},
        "left": {"names": list(DOF_NAMES), "effort": left.tolist()},
        "right": {"names": list(DOF_NAMES), "effort": right.tolist()},
        "actuation_field": "effort",
        "position_field_present": False,
        "ownership": "exclusive_torque_adapter",
    }


def validate_effort_payload(payload: Mapping) -> dict:
    if payload.get("frame_id") != TORQUE_SYNC_FRAME:
        raise AdapterRejected("SYNC_FRAME")
    if payload.get("position_field_present") is not False:
        raise AdapterRejected("POSITION_FIELD_PRESENT")
    for side in ("left", "right"):
        part = payload.get(side, {})
        if "position" in part or "joint_positions" in part:
            raise AdapterRejected("POSITION_FIELD_PRESENT")
        if list(part.get("names", ())) != DOF_NAMES:
            raise AdapterRejected("PAYLOAD_DOF_ORDER")
        _vector(part.get("effort"), 9, side.upper() + "_EFFORT")
    return {"valid": True, "frame_id": payload["frame_id"], "position_field_present": False}
