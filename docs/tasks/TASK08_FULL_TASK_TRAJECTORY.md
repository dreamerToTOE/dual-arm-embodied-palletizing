# Task08-A：完整 TaskTrajectoryCandidate 生成

## 状态

🟡 **已完成接口与离线规划实现；待 Isaac + MoveIt 联调验证。**

## 目标

Task08-A 将一台 FR3 的完整抓放过程表达为一个统一的候选，而不再只暴露 `LIFT -> PRE_PLACE` 携物段。

```text
当前起始状态 / HOME
-> PRE_PICK
-> CONTACT
-> ATTACH
-> LIFT
-> PRE_PLACE
-> PLACE
-> DETACH
-> RETREAT
```

生成候选时不下发 `/left/joint_command`、`/right/joint_command` 或吸盘命令；Isaac 场景中的机器人和 Box 均不运动。

## 新接口

```text
include/fr3_dual_palletize/task_event.hpp
include/fr3_dual_palletize/task_trajectory_candidate.hpp
```

`TaskTrajectoryCandidate` 包含：

```text
arm label / planning group / object id / eef link
完整 JointTrajectory
起始与最终关节状态
总时长
初始 Box pose 与计划释放 pose
按统一相对时间轴记录的 TaskEvent 列表
```

当前离散事件为：

```text
ATTACH
DETACH
```

`ATTACH` 发生在 CONTACT 后的名义 0.20 s 吸附命令窗口之后；`DETACH` 发生在 PLACE 后的名义 0.20 s 释放命令窗口与 0.30 s settle 窗口之后。这些停留段被写入完整 `JointTrajectory`，因此 Task08-B 可在统一时间轴正确切换 World / Attached Box 状态。

## Planning Scene 约束

候选规划会短暂：

```text
移除目标 Box
-> 规划 CONTACT
-> 作为 AttachedCollisionObject 规划 LIFT / PRE_PLACE / PLACE / RETREAT
-> 恢复目标 Box 到其规划前的 world pose
```

无论候选规划成功或失败，函数都会恢复目标 Box，避免一个候选污染另一臂候选或后续 Task07 执行。

## 当前输出节点

```text
ros2 run fr3_dual_palletize task08_candidate_demo
```

节点从 `/task07/box_poses` 读取两个 Box 的 Ground Truth，串行生成左右两个候选以保证临时 Planning Scene 状态隔离，并输出：

```text
完整轨迹点数
总时长
ATTACH 时间
DETACH 时间
计划释放 pose
```

## 后续：Task08-B

Task08-B 以两个 `TaskTrajectoryCandidate` 为输入，在同一双臂 RobotState 上按 10 ms 插值采样，并依据事件时间重建携带物状态，输出 `SAFE` 或包含首个冲突时间及碰撞对象的 `ConflictReport`。
