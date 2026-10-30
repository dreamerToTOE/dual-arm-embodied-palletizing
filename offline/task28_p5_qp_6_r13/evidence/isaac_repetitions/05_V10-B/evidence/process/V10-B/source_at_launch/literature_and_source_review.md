# R10 source and method review

R10 re-read the frozen G1 FK/IK/FCL validator, R8 ownership and effort adapter, R9 virtual frame/QP/runtime, R9 V1/V2 raw traces, R9 CPU approach audit, Task24's staged side approach, and the installed Isaac 4.5 articulation/PhysX interfaces. The implementation uses the repository's world geometric Jacobian and the same FCL-0.7.0 state checker. No historical source is edited.

The corrected reference uses the quintic `s(u)=10u^3-15u^4+6u^5`. Its first and second derivatives are zero at both endpoints, unlike R9's cubic smoothstep, whose second derivative is discontinuous at a stationary-to-motion junction. R10 records physics/reference integration at 60 Hz and preflight geometry at the same 60 Hz; the R9 `1/600` metadata is preserved only as a negative-control inconsistency.

The pose-aware QP computes world-frame SE(3) errors from the measured TCP-implied virtual objects. A common error correction is formed only after both translation and rotation errors are expressed in the same world frame. A relative virtual-object error drives an affine equality on the difference between the two implied object twists, so measured relative drift can be corrected instead of merely frozen. Joint position lookahead is included in the QP box, rather than checked only after integration. The QP is still kinematic and does not certify inverse dynamics or contact wrench allocation.

TSID, MoveIt 2 dual-arm planning, MoveIt Pro multi-tip planning and the referenced cooperative HQP literature provide task-priority and multi-tip design patterns. They do not qualify the mounted L-tool inertia, suction contact, object wrench observability or this installed FR3 runtime; no external dependency or performance transfer is claimed.

The installed collision validator is repository code around a standalone FCL 0.7.0 state helper. Its public API evaluates a supplied joint state; it does not implement a continuous swept-volume certificate. R10 therefore tests between-sample collision negatives and develops conservative interval bounds, but reports `CONTINUOUS_SAFETY_NOT_ESTABLISHED` whenever the bound cannot certify an entire segment.
