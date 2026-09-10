# Task09-C：协调候选的 Isaac 完整执行

状态：✅ 已完成一次 Isaac + MoveIt 运行时验收。

## 目标

Task09-A/B 只解决“是否存在安全的联合时间轨迹”。Task09-C 将经 Task08-B/FCL 复检为 `SAFE` 的左右 `TaskTrajectoryCandidate` 真正下发到 Isaac，使两台 FR3 完成：

```text
HOME
  -> PRE_PICK
  -> CONTACT
  -> SUCTION ON / CLOSED
  -> MoveIt ATTACH
  -> LIFT
  -> （可选：一臂在 LIFT 后局部 HOLD）
  -> PRE_PLACE
  -> PLACE
  -> SUCTION OFF / OPEN
  -> Isaac settle pose 回写 MoveIt World
  -> MoveIt DETACH
  -> RETREAT
  -> SAFE_EGRESS -> HOME
```

不允许把未通过联合 FCL 时空检查的候选下发给 Isaac。

## 执行语义

`CoordinatedTaskExecutor` 使用一条共享单调时钟，按 10 ms 同时向：

```text
/left/joint_command
/right/joint_command
```

发布两个候选的插值关节目标。局部等待已经编码在候选时间轴中，不由执行端再次重新计算。

候选现在显式记录四种事件：

```text
SUCTION_ON  -> 物理吸盘命令；确认 CLOSED
ATTACH      -> MoveIt AttachedCollisionObject
SUCTION_OFF -> 物理释放命令；确认 OPEN
DETACH      -> 读取 Isaac Box Ground Truth，回写 MoveIt World
```

在每个事件点执行器让两臂**共同 HOLD**：两条轨迹均保持在该时刻已由 Task08-B/FCL 采样验证的双臂姿态。事件完成后，共享时钟整体暂停时长被扣除，再继续原协调轨迹；因此等待 Surface Gripper 状态不会让一台机械臂单独脱离协调调度。

`SUCTION_ON/OFF` 不改变 Task08-B 的碰撞状态；检测器仍只以 `ATTACH/DETACH` 在 World Box 与 Attached Box 间切换。

## 安全门禁

`task08_candidate_demo` 新增参数：

```text
execute_coordinated_candidate:=false  # 默认，不运动
```

只有下列条件都成立时，设为 `true` 才会发布命令：

1. 两条完整候选均成功生成；
2. Task08-B 生成有效报告；
3. 原始候选直接 `SAFE`，或 Task09-B local wait 后的 `verification_report` 为 `SAFE`；
4. 两个 Isaac joint command topic、两个 suction command topic 以及 suction state topic 全部就绪。

若候选仍碰撞，节点输出 `Task09-C REFUSED` 并退出，绝不发布 joint command。

## 构建记录

2026-09-09：

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing-task09-coordinated-execution/ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fr3_dual_palletize --symlink-install
```

结果：`fr3_dual_palletize` 编译、链接、安装成功。现有 `geometry_msgs::msg::Pose` 聚合初始化警告来自已有 Task07/Task08 源码，与 Task09-C 无关。

## 运行时验收命令

先启动 Task06 双臂 MoveIt2、Isaac 的 Task08 交叉场景，并在 Isaac 中运行 `task07_dual_suction_bridge.py`。然后单独终端运行：

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing-task09-coordinated-execution/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run fr3_dual_palletize task08_candidate_demo --ros-args \
  -p scenario:=task08_conflict \
  -p enable_safe_egress:=true \
  -p enable_local_wait_coordination:=true \
  -p wait_step_sec:=0.10 \
  -p max_wait_sec:=15.0 \
  -p execute_coordinated_candidate:=true
```

通过条件：

```text
Task09-B SAFE
Task09-C ... SUCTION CLOSED confirmed
Task09-C ... MoveIt ATTACH completed
Task09-C ... SUCTION OPEN confirmed
Task09-C ... Isaac settle pose -> MoveIt World
Task09-C EXECUTION PASS
```

## 运行时验收记录

2026-09-09，在 Task08 交叉场景、Task06 双臂 MoveIt2 与 `task07_dual_suction_bridge.py` 均运行时，执行了上面的命令。

本次随机 OMPL 候选的 Task09-B/FCL 结果：

```text
strategy              = LOCAL_WAIT_LEFT
yielding_arm          = LEFT
wait_start            = 9.628 s
minimum_safe_wait     = 3.200 s
coordinated_makespan  = 25.752 s
Task09-B SAFE
```

真实执行日志：

```text
LEFT  SUCTION ON  -> CLOSED -> MoveIt ATTACH completed
RIGHT SUCTION ON  -> CLOSED -> MoveIt ATTACH completed
RIGHT SUCTION OFF -> OPEN   -> DETACH -> Isaac settle pose -> MoveIt World
LEFT  SUCTION OFF -> OPEN   -> DETACH -> Isaac settle pose -> MoveIt World
Task09-C EXECUTION PASS: events=8, candidate=25.752 s,
  pause=3.738 s, wall=29.996 s
```

最终 `/task07/box_poses` Ground Truth：

| Box | 计划 release center (m) | Isaac settle center (m) | 水平误差 | 相对计划 release 的 3D 偏差 |
| --- | --- | --- | ---: | ---: |
| A / left | `(0.820, 0.120, 0.066)` | `(0.819281, 0.120150, 0.065000)` | `0.735 mm` | `1.241 mm` |
| B / right | `(0.820, -0.120, 0.066)` | `(0.818438, -0.119228, 0.065000)` | `1.742 mm` | `2.009 mm` |

`z` 比计划 release center 低约 `1 mm` 是 `PLACEMENT_RELEASE_GAP = 0.001 m` 释放后 Box 落到桌面静止面的预期结果，不是掉落失败。两臂最终 joint state 回到各自 HOME 附近（最大观测偏差约 `0.0005 rad`）。

本次验收依赖 Task08-B/FCL 的完整联合碰撞检查；当前 Bridge 未发布 PhysX contact telemetry，因此没有把“未观测到碰撞异常”误记为独立的物理接触检测指标。

## 修改文件

```text
ros_ws/src/fr3_dual_palletize/
  include/fr3_dual_palletize/task_event.hpp
  include/fr3_dual_palletize/palletize_primitive.hpp
  include/fr3_dual_palletize/coordinated_task_executor.hpp
  src/palletize_primitive.cpp
  src/coordinated_task_executor.cpp
  src/task08_candidate_demo.cpp
  CMakeLists.txt
```
