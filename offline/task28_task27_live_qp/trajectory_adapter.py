"""Timestamped Task27 joint-target adapter for the live torque QP.

The Isaac feed bridge exposes each incoming Task27 point as a timestamped
``(q,)`` command.  This small adapter keeps the points in source time and
provides a bounded, deterministic ``q/dq/ddq`` sample for the physics clock.
It deliberately has no actuator or ROS dependencies; missing or unilateral
arms remain independently representable and the caller decides the task mode.
"""
from __future__ import annotations

from dataclasses import dataclass
import bisect
import math

import numpy as np


def _vector(value, shape, code):
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.shape != shape or not np.isfinite(arr).all():
        raise ValueError(code)
    return arr.copy()


@dataclass(frozen=True)
class JointTargetSample:
    q_rad: tuple[float, ...]
    dq_rad_s: tuple[float, ...]
    ddq_rad_s2: tuple[float, ...]
    source_time_s: float
    clamped: bool


class Task27JointTrajectoryAdapter:
    """Interpolate one arm's Task27 target stream in timestamp order.

    Task27's existing Isaac topic carries positions.  Velocities and
    accelerations are therefore estimated from neighboring position samples;
    when richer fields are added later, callers can pass them directly to
    ``push`` without changing the QP interface.  A repeated timestamp replaces
    the old point, while a non-monotonic timestamp is rejected so stale ROS
    traffic cannot move the target backwards in time.
    """

    def __init__(self, *, dof: int = 7, max_points: int = 512):
        if type(dof) is not int or dof <= 0 or type(max_points) is not int or max_points < 2:
            raise ValueError("INVALID_TRAJECTORY_ADAPTER_CONFIG")
        self.dof = dof
        self.max_points = max_points
        self._points: list[tuple[float, np.ndarray, np.ndarray | None, np.ndarray | None]] = []

    @property
    def point_count(self) -> int:
        return len(self._points)

    @property
    def first_time_s(self) -> float | None:
        return None if not self._points else float(self._points[0][0])

    @property
    def last_time_s(self) -> float | None:
        return None if not self._points else float(self._points[-1][0])

    def push(self, timestamp_s, q_rad, *, dq_rad_s=None, ddq_rad_s2=None):
        stamp = float(timestamp_s)
        if not math.isfinite(stamp):
            raise ValueError("INVALID_TRAJECTORY_TIMESTAMP")
        q = _vector(q_rad, (self.dof,), "INVALID_TRAJECTORY_POSITION")
        dq = None if dq_rad_s is None else _vector(dq_rad_s, (self.dof,), "INVALID_TRAJECTORY_VELOCITY")
        ddq = None if ddq_rad_s2 is None else _vector(ddq_rad_s2, (self.dof,), "INVALID_TRAJECTORY_ACCELERATION")
        times = [point[0] for point in self._points]
        index = bisect.bisect_left(times, stamp)
        row = (stamp, q, dq, ddq)
        if index < len(self._points) and abs(self._points[index][0] - stamp) <= 1e-12:
            self._points[index] = row
        else:
            # Out-of-order points are not silently re-timed.  A small bounded
            # reorder window is unnecessary because ROS source stamps are the
            # synchronization contract for this bridge.
            if self._points and stamp < self._points[-1][0] - 1e-12:
                raise ValueError("NONMONOTONIC_TRAJECTORY_TIMESTAMP")
            self._points.insert(index, row)
        if len(self._points) > self.max_points:
            del self._points[:len(self._points) - self.max_points]

    def _estimated_velocity(self, index):
        stamp, q, dq, ddq = self._points[index]
        if dq is not None:
            return dq.copy()
        if len(self._points) == 1:
            return np.zeros(self.dof)
        if index == 0:
            nxt = self._points[1]
            return (nxt[1] - q) / max(float(nxt[0] - stamp), 1e-9)
        if index == len(self._points) - 1:
            prev = self._points[index - 1]
            return (q - prev[1]) / max(float(stamp - prev[0]), 1e-9)
        prev, nxt = self._points[index - 1], self._points[index + 1]
        return (nxt[1] - prev[1]) / max(float(nxt[0] - prev[0]), 1e-9)

    def sample(self, source_time_s) -> JointTargetSample | None:
        if not self._points:
            return None
        t = float(source_time_s)
        if not math.isfinite(t):
            raise ValueError("INVALID_TRAJECTORY_SAMPLE_TIME")
        times = [point[0] for point in self._points]
        if t <= times[0]:
            q = self._points[0][1]
            return JointTargetSample(tuple(q), tuple(np.zeros(self.dof)), tuple(np.zeros(self.dof)), times[0], True)
        if t >= times[-1]:
            q = self._points[-1][1]
            # Once a streamed trajectory ends, hold its final configuration;
            # do not carry a finite-difference velocity forever.
            return JointTargetSample(tuple(q), tuple(np.zeros(self.dof)), tuple(np.zeros(self.dof)), times[-1], True)
        hi = bisect.bisect_right(times, t)
        lo = hi - 1
        t0, q0, dq0_given, ddq0_given = self._points[lo]
        t1, q1, dq1_given, ddq1_given = self._points[hi]
        duration = max(float(t1 - t0), 1e-9)
        u = float(np.clip((t - t0) / duration, 0.0, 1.0))
        dq0 = self._estimated_velocity(lo)
        dq1 = self._estimated_velocity(hi)
        # Cubic Hermite interpolation is C1 for position-only streams.  It
        # avoids the velocity jumps of zero-order target capture while staying
        # bounded by the source points.  Explicit accelerations remain
        # available for future Task27 publishers; with none supplied, ddq is
        # the analytical derivative of this cubic.
        h00 = 2*u**3 - 3*u**2 + 1
        h10 = u**3 - 2*u**2 + u
        h01 = -2*u**3 + 3*u**2
        h11 = u**3 - u**2
        q = h00*q0 + h10*duration*dq0 + h01*q1 + h11*duration*dq1
        dh00 = 6*u**2 - 6*u
        dh10 = 3*u**2 - 4*u + 1
        dh01 = -6*u**2 + 6*u
        dh11 = 3*u**2 - 2*u
        dq = (dh00*q0 + dh10*duration*dq0 + dh01*q1 + dh11*duration*dq1) / duration
        d2h00 = 12*u - 6
        d2h10 = 6*u - 4
        d2h01 = -12*u + 6
        d2h11 = 6*u - 2
        ddq = (d2h00*q0 + d2h10*duration*dq0 + d2h01*q1 + d2h11*duration*dq1) / (duration**2)
        if ddq0_given is not None or ddq1_given is not None:
            # The current Task27 stream has no acceleration field.  If a
            # richer source supplies one, use its endpoint values as a
            # diagnostic blend rather than pretending the cubic is exact.
            explicit = []
            if ddq0_given is not None:
                explicit.append(ddq0_given)
            if ddq1_given is not None:
                explicit.append(ddq1_given)
            ddq = np.mean(explicit, axis=0)
        return JointTargetSample(tuple(q), tuple(dq), tuple(ddq), t, False)
