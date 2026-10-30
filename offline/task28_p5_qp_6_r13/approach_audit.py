"""Bind R13 to the exact archived R10 approach; never resolve IK again.

The archived finite FCL results remain descriptive only. Continuous safety is
assessed separately by continuous_safety_audit, with detached/supported modes.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
SOURCE = REPO / "offline/task28_p5_qp_6_r10/evidence/approach_audit.json"
SOURCE_SHA256 = "40529575d19946f9257d4044f309fd942c4c94b28fe1ceba477bc73d417781b3"
DIRECT_SOURCE = REPO / "offline/task28_p5_qp_6_r3/evidence/approach_audit.json"
DIRECT_SHA256 = "3c291eb1b2c2e0bdb5516e85f75908ebd90d07d3e94c3da2843e21dfbbb1c0d0"


def bound_bytes(path, expected):
    payload = Path(path).read_bytes()
    if hashlib.sha256(payload).hexdigest() != expected:
        raise ValueError("ARCHIVED_APPROACH_SOURCE_HASH_MISMATCH:" + str(path))
    return payload


def archived_bytes():
    return bound_bytes(SOURCE, SOURCE_SHA256)


def main():
    payload = archived_bytes()
    archived = json.loads(payload)
    direct = json.loads(bound_bytes(DIRECT_SOURCE, DIRECT_SHA256))
    witness = direct["straight_joint_interpolation"]["first_unexpected_collision"]
    if witness != archived["direct_home_to_nominal_negative"]["first_unexpected_collision"]:
        raise ValueError("ARCHIVED_DIRECT_NEGATIVE_BINDING_MISMATCH")
    evidence = HERE / "evidence"
    evidence.mkdir(exist_ok=True)
    target = evidence / "approach_audit.json"
    target.write_bytes(payload)
    binding = {
        "schema": "TASK28_R13_ARCHIVED_APPROACH_BINDING_V1",
        "source": str(SOURCE.relative_to(REPO)), "source_sha256": SOURCE_SHA256,
        "copy": str(target.relative_to(REPO)), "copy_byte_identical": target.read_bytes() == payload,
        "direct_negative_source": str(DIRECT_SOURCE.relative_to(REPO)),
        "direct_negative_source_sha256": DIRECT_SHA256,
        "direct_negative_witness": witness,
        "direct_negative_scope": "Separate R3 direct path, copied by R10; not a collision witness on the staged R10 path.",
        "r13_method": "Exact archive reuse; no IK, new path, physics, command or attachment generation.",
        "archived_finite_samples_are_continuous_proof": False,
        "continuous_safety": "CONTINUOUS_SAFETY_NOT_ESTABLISHED",
    }
    (evidence / "approach_archive_binding.json").write_text(json.dumps(binding, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"status": "ARCHIVED_APPROACH_REUSED_BYTE_IDENTICALLY",
                      "source_sha256": SOURCE_SHA256, "archived_status": archived["status"]}))


if __name__ == "__main__":
    main()
