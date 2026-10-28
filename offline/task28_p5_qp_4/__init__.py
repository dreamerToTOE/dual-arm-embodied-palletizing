"""Offline-only unified dual-arm motion-reference contract for future torque control."""

from .reference import (
    ReferenceContract,
    ReferenceFrame,
    build_reference_sequence,
    load_qp3_positive,
    validate_reference,
    validate_sequence,
)

__all__ = [
    "ReferenceContract", "ReferenceFrame", "build_reference_sequence",
    "load_qp3_positive", "validate_reference", "validate_sequence",
]
