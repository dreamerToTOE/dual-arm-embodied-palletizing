"""Task27 live cooperative torque-QP math.

The package contains the numerical core used by the Isaac bridge.  It accepts
the *synchronised* measured bilateral state and returns a joint acceleration
and torque pair.  It has no Isaac, ROS, USD, or hardware dependency; ownership,
model-source and collision gates remain the caller's responsibility.
"""

from .torque_qp import (
    FR3_POSITION_LOWER,
    FR3_POSITION_UPPER,
    CooperativeTorqueQPConfig,
    CooperativeTorqueQPResult,
    solve_cooperative_torque_qp,
)

__all__ = [
    "FR3_POSITION_LOWER",
    "FR3_POSITION_UPPER",
    "CooperativeTorqueQPConfig",
    "CooperativeTorqueQPResult",
    "solve_cooperative_torque_qp",
]
