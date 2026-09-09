# Task09-B：冲突窗口局部等待

## 状态

🟡 **算法已实现并完成只读 MoveIt/FCL 验证；当前 Task08 交叉候选存在终态冲突，局部等待正确判定为无解。**

## 目标

相对 Task09-A 的整条任务启动延迟，Task09-B 只在让行臂完成 LIFT、进入公共工作区前插入等待：

```text
HOME -> PRE_PICK -> CONTACT -> ATTACH -> LIFT -> WAIT -> PRE_PLACE -> PLACE -> DETACH -> RETREAT
```

空间路径、吸附事件和 Box 几何均不改变。每一个等待候选都重新调用 Task08-B 的私有 MoveIt/FCL 时空检测器；本任务不下发 joint command 或 suction command。

## 实现

- `TaskTrajectoryCandidate` 新增 `TaskStageMarker`，在统一时间轴记录每个阶段边界；
- `SpatioTemporalConflictDetector` 不再在首个碰撞处停止，输出连续的 `ConflictWindow`；
- `LocalWaitCoordinator` 在 `LIFT_TO_PRE_PLACE` 的开始时刻插入 HOLD，并同步后移：
  - 所有后续 `JointTrajectoryPoint.time_from_start`；
  - 后续 `ATTACH` / `DETACH` 事件（本等待点在 ATTACH 后、DETACH 前）；
  - 后续阶段边界；
- 分别搜索 `LOCAL_WAIT_LEFT`、`LOCAL_WAIT_RIGHT` 的最小安全等待时间；先比较等待时间，再比较 makespan，同值时默认保持左臂优先；
- 如果原始检测报告在完整轨迹 horizon 的末端仍冲突，则直接报告无解：两臂最终都保持终止 RETREAT 姿态，延长中途 HOLD 不会改变终态几何关系。

## 运行命令

先启动已完成的 Task06 双臂 MoveIt2 与 Task07 dual suction bridge，再执行：

```bash
source /opt/ros/humble/setup.bash
source ~/lmy/dual-arm-embodied-palletizing/ros_ws/install/setup.bash
source ~/lmy/dual-arm-embodied-palletizing-task09-local-wait/ros_ws/install/setup.bash

ros2 run fr3_dual_palletize task08_candidate_demo \
  --ros-args \
  -p scenario:=task08_conflict \
  -p enable_local_wait_coordination:=true \
  -p wait_step_sec:=0.05 \
  -p max_wait_sec:=15.0
```

SAFE 对照组：

```bash
ros2 run fr3_dual_palletize task08_candidate_demo \
  --ros-args \
  -p scenario:=task07_safe \
  -p enable_local_wait_coordination:=true
```

## 实际验证（2026-09-09）

### 编译

```text
colcon build --packages-select fr3_dual_palletize --symlink-install
Finished <<< fr3_dual_palletize
```

仅出现既有 `geometry_msgs::msg::Pose` 初始化警告，未出现本次代码引入的编译错误。

### Task07 SAFE 对照组

```text
Task08-B SAFE: NO_CONFLICT
Task09-B LOCAL WAIT COORDINATION
strategy            = SIMULTANEOUS
minimum_safe_wait   = 0.000 s
Task09-B SAFE
```

### Task08 交叉场景

一次实际规划的原始检测结果：

```text
first conflict = 15.490 s
window 1 = [15.490, 16.010] s
window 2 = [17.710, 19.601] s
horizon  = 19.601 s
pair     = left_fr3_link6 <-> right_fr3_link5
type     = ARM_ARM
```

第二个窗口持续到 horizon，说明两臂在最终 RETREAT 姿态仍冲突。故 Task09-B 正确输出 `NO_SOLUTION`，而不是伪造一个“等待足够久即可安全”的结论。

## 边界与下一步

Task09-B 负责消除有限时间的公共区交叉冲突；它不改变空间路径，也不通过放宽 ACM 掩盖碰撞。针对当前终态冲突，下一步应在 Task09-C/后续子任务中生成安全退出轨迹或进行局部空间重规划，再统一交由 Task08-B 验证。
