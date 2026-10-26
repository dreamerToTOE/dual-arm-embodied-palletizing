#!/usr/bin/env python3
"""Task28 controller input, with no planning or robot-command implementation.

The future validator/controller consumes this immutable single-task specification.
This module uses only the standard library and can be inspected offline.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path


FIXTURE_ID = "task28_g1_0_new_single_cube_v0"
CUBE_ID = "Cube_01"
TASK_INDEX = 0
CUBE_INDEX = 0
CUBE_COUNT = 1
WORLD_FRAME = "world"
TARGET_WORLD_M = (0.82, -0.075, 0.26)


@dataclass(frozen=True)
class SingleCubeTask:
    fixture_id: str
    task_index: int
    cube_id: str
    cube_index: int
    pose_array_order: tuple[str, ...]
    frame_id: str
    initial_position_m: tuple[float, ...]
    target_position_m: tuple[float, ...]
    orientation_xyzw: tuple[float, ...]
    size_m: tuple[float, ...]
    mass_kg: float


def load_task(*, cube_ids=(CUBE_ID,), pose_count=CUBE_COUNT,
              task_index=TASK_INDEX, cube_index=CUBE_INDEX) -> SingleCubeTask:
    """Reject ambiguous mappings before a future controller can consume a task."""
    if tuple(cube_ids) != (CUBE_ID,) or pose_count != CUBE_COUNT:
        raise ValueError("Task28 requires exactly one PoseArray entry: Cube_01")
    if task_index != TASK_INDEX or cube_index != CUBE_INDEX:
        raise ValueError("Task28 requires task_index=0 and cube_index=0")
    root = Path(__file__).resolve().parents[1]
    manifest = json.loads((root / "fixture_manifest.json").read_text())
    contract = json.loads((root / "controller/task28_g1_0_controller_contract.json").read_text())
    cube = manifest["cube"]
    expected = [{"task_index": 0, "cube_id": CUBE_ID, "cube_index": 0,
                 "target_pose_world_m": list(TARGET_WORLD_M)}]
    if (manifest["fixture_id"] != FIXTURE_ID or contract["task_index_map"] != expected
            or cube["id"] != CUBE_ID or cube["index"] != CUBE_INDEX
            or cube["count"] != CUBE_COUNT or cube["target_pose_world_m"] != list(TARGET_WORLD_M)):
        raise ValueError("Task28 manifest/controller task mapping mismatch")
    return SingleCubeTask(
        FIXTURE_ID, TASK_INDEX, CUBE_ID, CUBE_INDEX, (CUBE_ID,), WORLD_FRAME,
        tuple(cube["initial_pose_world_m"]), TARGET_WORLD_M,
        tuple(cube["orientation_xyzw"]), tuple(cube["size_m"]), cube["mass_kg"],
    )


if __name__ == "__main__":
    print(json.dumps(asdict(load_task()), indent=2))
