import numpy as np
import pytest

from offline.task28_t0.torque_interface import (
    ARM_DOF_NAMES,
    TorqueInterface,
    TorqueReject,
)


def activate(interface):
    interface.activate(
        dof_names=tuple(ARM_DOF_NAMES) + ("fr3_finger_joint1", "fr3_finger_joint2"),
        position_sink_active=False,
        arm_effort_modes=("force",) * 7,
        arm_kp=np.zeros(7),
        arm_kd=np.zeros(7),
        finger_signature={"mode": ("force", "force"), "kp": (1.0, 2.0)},
        finger_signature_after={"mode": ("force", "force"), "kp": (1.0, 2.0)},
    )


def test_activation_requires_exclusive_arm_ownership():
    with pytest.raises(TorqueReject, match="POSITION_SINK_ACTIVE"):
        TorqueInterface().activate(
            dof_names=tuple(ARM_DOF_NAMES) + ("fr3_finger_joint1", "fr3_finger_joint2"),
            position_sink_active=True,
            arm_effort_modes=("force",) * 7,
            arm_kp=np.zeros(7), arm_kd=np.zeros(7),
        )


def test_activation_rejects_unknown_extra_dof_order():
    interface = TorqueInterface()
    with pytest.raises(TorqueReject, match="DOF_ORDER"):
        interface.activate(
            dof_names=tuple(ARM_DOF_NAMES) + ("unexpected_extra_dof", "fr3_finger_joint2"),
            position_sink_active=False,
            arm_effort_modes=("force",) * 7,
            arm_kp=np.zeros(7), arm_kd=np.zeros(7),
        )


def test_pair_command_is_bounded_and_cursor_checked():
    interface = TorqueInterface()
    activate(interface)
    first = interface.submit_pair(physics_step=0, time_s=0.0,
                                  left_Nm=np.zeros(7), right_Nm=np.zeros(7))
    assert interface.take_pending(physics_step=0, time_s=0.0) == first
    interface.record_write(command=first, applied_left=np.zeros(7),
                           applied_right=np.zeros(7), physics_step=0, time_s=0.0)
    interface.post_step(physics_step=1, time_s=1 / 60)
    second = interface.submit_pair(physics_step=1, time_s=1 / 60,
                                   left_Nm=np.full(7, 0.01), right_Nm=np.full(7, -0.01))
    assert second.physics_step == 1


def test_abs_rate_nan_units_and_missing_pair_fail_closed():
    interface = TorqueInterface()
    activate(interface)
    with pytest.raises(TorqueReject, match="TORQUE_ABS_LIMIT"):
        interface.submit_pair(physics_step=0, time_s=0.0,
                              left_Nm=np.array([36., 0, 0, 0, 0, 0, 0]), right_Nm=np.zeros(7))
    assert interface.latched_failure == "TORQUE_ABS_LIMIT"

    interface = TorqueInterface(); activate(interface)
    with pytest.raises(TorqueReject, match="UNITS"):
        interface.submit_pair(physics_step=0, time_s=0.0,
                              left_Nm=np.zeros(7), right_Nm=np.zeros(7), units="Nm")
    assert interface.latched_failure == "UNITS"

    interface = TorqueInterface(); activate(interface)
    with pytest.raises(TorqueReject, match="MISSING_PAIR"):
        interface.take_pending(physics_step=0, time_s=0.0)
    assert interface.latched_failure == "MISSING_PAIR"


def test_rate_rejects_and_does_not_continue_after_latch():
    interface = TorqueInterface(); activate(interface)
    first = interface.submit_pair(physics_step=0, time_s=0.0,
                                  left_Nm=np.zeros(7), right_Nm=np.zeros(7))
    interface.take_pending(physics_step=0, time_s=0.0)
    interface.record_write(command=first, applied_left=np.zeros(7),
                           applied_right=np.zeros(7), physics_step=0, time_s=0.0)
    interface.post_step(physics_step=1, time_s=1 / 60)
    with pytest.raises(TorqueReject, match="TORQUE_RATE_LIMIT"):
        interface.submit_pair(physics_step=1, time_s=1 / 60,
                              # J3: 5 Nm is below its 8 Nm absolute limit but
                              # exceeds its 240 Nm/s * 1/60 s = 4 Nm step limit.
                              left_Nm=np.array([0., 0, 5., 0, 0, 0, 0]), right_Nm=np.zeros(7))
    assert not interface.active
    with pytest.raises(TorqueReject, match="LATCHED"):
        interface.submit_pair(physics_step=2, time_s=2 / 60,
                              left_Nm=np.zeros(7), right_Nm=np.zeros(7))

