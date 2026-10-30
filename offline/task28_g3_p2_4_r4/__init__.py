"""Task28 G3-P2-4-R4 strict observer source adapter.

This package is a source/quality boundary only.  It does not talk to Isaac
Sim, ROS, libfranka, a robot, a contact sensor, or an actuator sink.
"""
from .source_adapter import (
    AdapterContract,
    AdapterRole,
    AdaptedSignal,
    SourceAdapter,
    SourceAdapterError,
    SourceDescriptor,
    SourceKind,
    SourceSample,
    SourceValidation,
    canonical_descriptor,
)

__all__ = [
    "AdapterContract", "AdapterRole", "AdaptedSignal", "SourceAdapter",
    "SourceAdapterError", "SourceDescriptor", "SourceKind", "SourceSample",
    "SourceValidation", "canonical_descriptor",
]
