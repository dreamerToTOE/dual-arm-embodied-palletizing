"""Torque-compatible, non-activating interface for the existing D6 setup."""

from __future__ import annotations

from dataclasses import dataclass

from .common import load_json, BRIDGE_CONTRACT_PATH


@dataclass(frozen=True)
class SuctionStateInterface:
    mode: str = "TELEMETRY_ONLY"
    left_closed: bool = False
    right_closed: bool = False
    activation_authorized: bool = False

    def request_state(self, desired_left: bool, desired_right: bool):
        if desired_left or desired_right:
            raise RuntimeError("SUCTION_CLOSURE_NOT_AUTHORIZED_IN_R2")
        return SuctionStateInterface(self.mode, False, False, self.activation_authorized)

    def validate_contract(self):
        contract = load_json(BRIDGE_CONTRACT_PATH)["suction_contract"]
        expected = {
            "one_surface_gripper_per_arm": True,
            "grip_threshold_m": 0.003,
            "force_break_N": 1.0e6,
            "torque_break_Nm": 1.0e6,
            "bend_angle_rad": 0.0,
            "stiffness": 1.0e4,
            "damping": 1.0e3,
            "disable_gravity": False,
            "retry_close": True,
        }
        if contract != expected:
            raise RuntimeError("FROZEN_SUCTION_CONTRACT_DRIFT")
        return {"valid": True, "contract": contract, "closure_attempted": False}
