"""Frozen-model controls whose strict intermediate failures evade the old grid.

These are ordinary typed cubic-Hermite trajectories.  Their joints, grasp
contract, feedback flags and orientations stay fixed; no new trajectory type or
repair operation is introduced.  A separate closed-form scalar polynomial
identifies the witness independently of the interpolation/checker implementation.
"""
from dataclasses import dataclass, replace
import json
import math
from pathlib import Path

from task28_contact.modes import OperationMode, SupportSite
from task28_edge.experiments import waypoint
from task28_edge.types import (Interpolation, Motion, Trajectory,
                               TranslationInterpolation)


REPO = Path(__file__).resolve().parents[2]
SOURCE = REPO / "offline/task28_g1_2a/examples/r1_configurations.json"
DURATION_S = 0.02
WITNESS_U = 1.0 - 1.0 / math.sqrt(3.0)
SHAPE_MAXIMUM = 1.0 / (3.0 * math.sqrt(3.0))


@dataclass(frozen=True)
class AdversarialControl:
    trajectory: Trajectory
    source_configuration: str
    axis: int
    signed_peak_offset_m: float
    frozen_threshold_m: float
    expected_witness_reasons: tuple[str, ...]
    witness_u: float = WITNESS_U


def scalar_shape(u):
    """Independent f(u)=u(1-u)(1-u/2), rather than Hermite basis evaluation."""
    return float(u) * (1.0 - float(u)) * (1.0 - float(u) / 2.0)


def independent_scalar_offset(control, u):
    return control.signed_peak_offset_m * scalar_shape(u) / SHAPE_MAXIMUM


def independent_object_position(control, u):
    """Closed-form world position, with no edge interpolation helper involved."""
    position = list(control.trajectory.waypoints[0].object_position_m)
    position[control.axis] += independent_scalar_offset(control, u)
    return tuple(position)


def analytic_control_evidence(control, sampled_us=(0.0, 0.5, 1.0)):
    """Small JSON-serializable oracle facts for a separately evaluated trace."""
    trajectory = control.trajectory
    first, last = trajectory.waypoints
    sampled_us = tuple(float(u) for u in sampled_us)
    return {
        "oracle": "closed_form_cubic_derivative_root_not_grid_search",
        "shape": "u*(1-u)*(1-u/2)",
        "shape_derivative": "1-3*u+1.5*u*u",
        "shape_derivative_roots": [WITNESS_U, 1.0 + 1.0 / math.sqrt(3.0)],
        "source_configuration": control.source_configuration,
        "source_path": str(SOURCE.relative_to(REPO)),
        "axis": control.axis,
        "signed_peak_offset_m": control.signed_peak_offset_m,
        "frozen_threshold_m": control.frozen_threshold_m,
        "witness_u": control.witness_u,
        "witness_time_s": first.time_s + control.witness_u * (last.time_s - first.time_s),
        "witness_position_m": independent_object_position(control, control.witness_u),
        "ordinary_sample_us": sampled_us,
        "ordinary_sample_offsets_m": tuple(independent_scalar_offset(control, u) for u in sampled_us),
        "expected_witness_reasons": control.expected_witness_reasons,
        "evidence_boundary": "Synthetic computational witness; no motion or physical observation.",
    }


def _build(name, configuration_name, axis, signed_peak, threshold, reasons, supported=False):
    configuration = json.loads(SOURCE.read_text())["configurations"][configuration_name]
    mode = (OperationMode.BILATERAL_SUPPORTED if supported else
            OperationMode.BILATERAL_TRANSITIONING)
    site = SupportSite.SUPPLY if supported else None
    first = waypoint(configuration, 0.0, name + ":start", mode, site)
    last = waypoint(configuration, DURATION_S, name + ":end", mode, site)
    scale = signed_peak / SHAPE_MAXIMUM
    first_velocity = [0.0, 0.0, 0.0]
    last_velocity = [0.0, 0.0, 0.0]
    # f'(0)=1 and f'(1)=-1/2.  Tangents are in physical metres/second.
    first_velocity[axis] = scale / DURATION_S
    last_velocity[axis] = -scale / (2.0 * DURATION_S)
    first = replace(first, object_velocity_m_s=tuple(first_velocity))
    last = replace(last, object_velocity_m_s=tuple(last_velocity))
    trajectory = Trajectory(
        name, (first, last), Motion.GENERIC,
        Interpolation(translation=TranslationInterpolation.CUBIC_HERMITE),
        scope="Frozen-model off-grid failure control; mathematical synthetic input only, never executable motion.",
    )
    return AdversarialControl(trajectory, configuration_name, axis, signed_peak,
                              threshold, reasons)


def adversarial_controls():
    """Both controls pass all three samples of the unchanged default old checker.

    The maximum at 1-1/sqrt(3) falls strictly between 0 and 1/2.  The first
    control exceeds the unchanged 100 um grasp-translation tolerance by 100 nm.
    The second exceeds the unchanged 1 um support-depth tolerance by 10 nm,
    while preserving bilateral closure.  Strict threshold crossings can be
    missed despite a small displacement and an existing derivative-based grid.
    """
    return {
        "offgrid_closure_excursion": _build(
            "offgrid_closure_excursion", "supply_gap_4000um", 0,
            1.001e-4, 1e-4,
            ("LEFT_GRASP_TRANSLATION_MISMATCH", "RIGHT_GRASP_TRANSLATION_MISMATCH"),
        ),
        "offgrid_support_penetration": _build(
            "offgrid_support_penetration", "supply_gap_0000um", 2,
            -1.01e-6, 1e-6,
            ("SUPPORT_TABLE_PENETRATION", "CLEARANCE_TABLE_PENETRATION"),
            supported=True,
        ),
    }
