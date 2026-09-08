# Task06-B：双 FR3 ROS / TF / Joint 通信拆分
# Isaac Sim 4.5 Script Editor 中运行。
#
# 目标：
#   - 左臂独立发布 /left/joint_states，订阅 /left/joint_command
#   - 右臂独立发布 /right/joint_states，订阅 /right/joint_command
#   - 左右 TF 暂时分别发布到 /left/tf、/right/tf
#   - 保留全局 /clock
#
# 注意：
#   Task06-B 只验证通信链不串线。
#   Task06-C 再统一处理双臂 MoveIt 所需的标准 /tf + 唯一 frame/joint 前缀。

import omni.graph.core as og
import omni.usd


stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("当前没有有效 USD Stage。")

LEFT_ROOT = "/World/left_fr3"
RIGHT_ROOT = "/World/right_fr3"
GRAPH_PATH = "/World/Task06ROSGraph"

OLD_GRAPH_PATHS = [
    "/World/ActionGraph",
    "/ActionGraph",
    GRAPH_PATH,
]


def require_prim(path):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise RuntimeError(
            f"缺少 {path}。请先在 Timeline Stop 状态运行 task06_dual_fr3_scene.py。"
        )
    return prim


def remove_if_exists(path):
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        stage.RemovePrim(path)


# ============================================================
# 1. 基线检查
# ============================================================

require_prim(LEFT_ROOT)
require_prim(RIGHT_ROOT)

# Task04/05 的旧单臂 ActionGraph 会继续监听 /joint_command，
# 双臂阶段不再保留，避免同一 Stage 中存在失效或重复 ROS 链。
for path in OLD_GRAPH_PATHS:
    remove_if_exists(path)

# ============================================================
# 2. 创建双臂 ROS2 ActionGraph
# ============================================================

og.Controller.edit(
    {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),

            ("LeftPublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("LeftSubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("LeftArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            ("LeftPublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),

            ("RightPublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("RightSubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("RightArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            ("RightPublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
        ],

        og.Controller.Keys.CONNECT: [
            # 全局 clock
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),

            # Left joint state / command
            ("OnPlaybackTick.outputs:tick", "LeftPublishJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "LeftSubscribeJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "LeftArticulationController.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "LeftPublishJointState.inputs:timeStamp"),
            ("LeftSubscribeJointState.outputs:jointNames", "LeftArticulationController.inputs:jointNames"),
            ("LeftSubscribeJointState.outputs:positionCommand", "LeftArticulationController.inputs:positionCommand"),
            ("LeftSubscribeJointState.outputs:velocityCommand", "LeftArticulationController.inputs:velocityCommand"),
            ("LeftSubscribeJointState.outputs:effortCommand", "LeftArticulationController.inputs:effortCommand"),

            # Right joint state / command
            ("OnPlaybackTick.outputs:tick", "RightPublishJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "RightSubscribeJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "RightArticulationController.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "RightPublishJointState.inputs:timeStamp"),
            ("RightSubscribeJointState.outputs:jointNames", "RightArticulationController.inputs:jointNames"),
            ("RightSubscribeJointState.outputs:positionCommand", "RightArticulationController.inputs:positionCommand"),
            ("RightSubscribeJointState.outputs:velocityCommand", "RightArticulationController.inputs:velocityCommand"),
            ("RightSubscribeJointState.outputs:effortCommand", "RightArticulationController.inputs:effortCommand"),

            # 左右 TF 暂时拆成独立 topic，避免同名 frame 互相覆盖。
            ("OnPlaybackTick.outputs:tick", "LeftPublishTF.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "LeftPublishTF.inputs:timeStamp"),
            ("OnPlaybackTick.outputs:tick", "RightPublishTF.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "RightPublishTF.inputs:timeStamp"),
        ],

        og.Controller.Keys.SET_VALUES: [
            ("ReadSimTime.inputs:resetOnStop", True),
            ("PublishClock.inputs:topicName", "/clock"),

            ("LeftPublishJointState.inputs:targetPrim", LEFT_ROOT),
            ("LeftPublishJointState.inputs:topicName", "/left/joint_states"),
            ("LeftSubscribeJointState.inputs:topicName", "/left/joint_command"),
            ("LeftArticulationController.inputs:robotPath", LEFT_ROOT),
            ("LeftPublishTF.inputs:targetPrims", [LEFT_ROOT]),
            ("LeftPublishTF.inputs:topicName", "/left/tf"),

            ("RightPublishJointState.inputs:targetPrim", RIGHT_ROOT),
            ("RightPublishJointState.inputs:topicName", "/right/joint_states"),
            ("RightSubscribeJointState.inputs:topicName", "/right/joint_command"),
            ("RightArticulationController.inputs:robotPath", RIGHT_ROOT),
            ("RightPublishTF.inputs:targetPrims", [RIGHT_ROOT]),
            ("RightPublishTF.inputs:topicName", "/right/tf"),
        ],
    },
)

print("")
print("====================================================")
print("Task06-B 双 FR3 ROS ActionGraph 创建完成")
print(f"Graph: {GRAPH_PATH}")
print("")
print("Left:")
print("  PUB /left/joint_states")
print("  SUB /left/joint_command")
print("  PUB /left/tf")
print("")
print("Right:")
print("  PUB /right/joint_states")
print("  SUB /right/joint_command")
print("  PUB /right/tf")
print("")
print("Global:")
print("  PUB /clock")
print("")
print("注意：/left/tf 与 /right/tf 只是 Task06-B 通信隔离测试。")
print("Task06-C 会改成适配 MoveIt 的唯一 frame 前缀 + 标准 /tf。")
print("")
print("下一步：点击 Play，然后在 ROS2 终端检查 topics 并分别发送左/右关节命令。")
print("====================================================")
