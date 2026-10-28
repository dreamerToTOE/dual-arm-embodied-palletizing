from __future__ import annotations

import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
FIXTURE = REPO / "fixtures/task28_g1_0_new_single_cube_v0"
MANIFEST_PATH = FIXTURE / "fixture_manifest.json"
SCENE_CONTRACT_PATH = FIXTURE / "scene/task28_g1_0_scene_contract.json"
BRIDGE_CONTRACT_PATH = FIXTURE / "bridge/task28_g1_0_bridge_contract.json"
FR3_ASSET = Path(
    "/home/ubuntu2004/Documents/Codex/2026-10-25/"
    "files-pasted-by-the-user-task/work/g1_0b_asset_audit/fr3.usd"
)
FR3_SHA256 = "3feceb47747b49f908f9719299132deb9759f2d5edb390d14f0f3ed62c63255f"
FIXTURE_ID = "task28_g1_0_new_single_cube_v0"
TASK_ROOT = "/World/Task28G10SingleCube"
LEFT_ROOT = TASK_ROOT + "/left_fr3"
RIGHT_ROOT = TASK_ROOT + "/right_fr3"
CUBE_PATH = TASK_ROOT + "/Supply/Cube_01"
TCP_PATHS = {
    "left": LEFT_ROOT + "/fr3_link8/side_suction_tool/side_suction_tcp",
    "right": RIGHT_ROOT + "/fr3_link8/side_suction_tool/side_suction_tcp",
}
LINK8_PATHS = {"left": LEFT_ROOT + "/fr3_link8", "right": RIGHT_ROOT + "/fr3_link8"}
DOF_NAMES = [f"fr3_joint{i}" for i in range(1, 8)] + ["fr3_finger_joint1", "fr3_finger_joint2"]
ARM_DOF_NAMES = DOF_NAMES[:7]
FINGER_DOF_NAMES = DOF_NAMES[7:]
PHYSICS_DT = 1.0 / 60.0
TORQUE_SYNC_FRAME = "task28_g2_p5_qp_6_r2_torque_sync"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_json(path: Path):
    return json.loads(Path(path).read_text())


def dump_json(path: Path, value) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n")


def serialise_manifest_hashes() -> dict[str, str]:
    paths = [
        MANIFEST_PATH,
        SCENE_CONTRACT_PATH,
        BRIDGE_CONTRACT_PATH,
        FIXTURE / "scene/task28_single_cube_scene.py",
        FIXTURE / "bridge/task28_single_cube_bridge.py",
        REPO / "ros_ws/src/fr3_dual_side_suction_description/urdf/dual_side_suction.xacro",
        REPO / "docs/tasks/TASK28_G1_0_BASELINE_FREEZE.md",
        REPO / "docs/tasks/TASK28_G0A_INTERFACE_AUDIT.md",
        REPO / "docs/tasks/TASK28_G2_P5_QP_6_R1_CONSTRAINT_AWARE_DYNAMICS.md",
    ]
    return {str(path.relative_to(REPO)): sha256(path) for path in paths}
