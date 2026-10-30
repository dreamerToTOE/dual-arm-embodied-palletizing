"""Independent interval audit of the original Cube approach candidate.

The frozen verifier is run at bounded depths.  Its unresolved AABB/support rules
are preserved as evidence; no sampled result is promoted to a certificate.
"""
from __future__ import annotations
import json, sys
from dataclasses import replace, asdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
for name in ("task28_g1_1", "task28_g1_2a", "task28_g1_2b_0", "task28_g1_2b_1"):
    p = str(REPO / "offline" / name)
    if p not in sys.path: sys.path.insert(0, p)
from continuous_verifier import ContinuousVerifier
from task28_edge.types import Waypoint, Trajectory, Motion
from task28_contact.modes import OperationMode as M
from task28_contact.clearance import ConfirmationEvent as E
from task28_contact.modes import AttachmentIntent, ObservedAttachment, EvidenceSource
from task28_contact.clearance import SupportSite
from task28_edge.types import Provenance, Source
from task28_edge.checker import SampledEdgeValidator
from offline.task28_p5_qp_6_r12.virtual_frames import physical_cube_pose


def make_waypoint(q, t, ident, cube):
    return Waypoint(float(t), tuple(q["left"]), tuple(q["right"]), tuple(float(x) for x in cube[:3, 3]),
                    (0., 0., 0., 1.), M.BILATERAL_AIRBORNE, None, True, True,
                    AttachmentIntent(ident, -1., True, True),
                    ObservedAttachment(ident, float(t), ident, True, True, EvidenceSource.INDEPENDENT_SYNTHETIC),
                    Provenance(Source.EXPLICIT_SYNTHETIC, "PRE_CONTACT_ORIGINAL_CUBE:" + ident, ident,
                               "CPU-only approach audit; synthetic typed input, never observed or executed."),
                    E.OBSERVE, observation_stamp_s=float(t))


def main():
    approach = json.loads((HERE / "evidence/approach_audit.json").read_text())
    cube = physical_cube_pose().T_WCube
    names = ["FREE_HOME", "HIGH_OUTER", "OUTER_CONTACT", "PRE_CONTACT_READY"]
    trajectory = Trajectory("r12_original_cube_approach_interval_audit",
        tuple(make_waypoint(approach["solutions"][n], i, n, cube) for i, n in enumerate(names)),
        Motion.GENERIC, scope="CPU-only original Cube approach; no execution claim.")
    rows = []
    for depth in (0, 1, 2, 4):
        verifier = ContinuousVerifier(SampledEdgeValidator(), max_depth=depth)
        sampled = verifier.checker.validate(trajectory)
        result = verifier.verify(trajectory, sampled)
        rows.append({"max_depth": depth, "combined_status": result["combined_status"],
                     "joint_limits": result["joint_limits"]["status"],
                     "closed_chain": result["closed_chain"]["status"],
                     "collision": result["collision"]["status"],
                     "sampled_edge_status": getattr(sampled.status, "value", sampled.status),
                     "sampled_count": sampled.evaluated_sample_count,
                     "sampled_reasons": list(sampled.reasons),
                     "unresolved_interval_count": sum(1 for x in result.get("intervals", [])
                         if x.get("collision_status") == "CONTINUOUS_VERIFICATION_INCONCLUSIVE" or
                            x.get("closure_status") == "CONTINUOUS_VERIFICATION_INCONCLUSIVE"),
                     "witness_count": len(result.get("witnesses", [])),
                     "max_depth_observed": result.get("maximum_subdivision_depth", depth)})
    out = {
        "schema": "TASK28_R12_ORIGINAL_CUBE_CONTINUOUS_AUDIT_V1",
        "status": "CONTINUOUS_SAFETY_NOT_ESTABLISHED",
        "reason": "interval verifier leaves collision/support bounds unresolved; existing FCL is single-state rigid collision only and has no articulated FK CCD callback",
        "trajectory": trajectory.identity,
        "original_cube_identity": "PRE_CONTACT_ORIGINAL_CUBE",
        "finite_sample_source": "evidence/approach_audit.json",
        "depth_runs": rows,
        "adversarial_controls": "offline/task28_g1_2b_1/adversarial_controls.py demonstrates off-grid witnesses that endpoint/grid samples can miss",
        "execution": {"status": "BLOCKED", "suction": 0, "cube_commands": 0, "ros": 0},
        "not_a_certificate": True,
    }
    outpath = HERE / "evidence" / "continuous_safety_audit.json"
    outpath.write_text(json.dumps(out, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"status": out["status"], "depths": rows}))

if __name__ == "__main__": main()
