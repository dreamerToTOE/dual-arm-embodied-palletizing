# Task09-B：安全退出 + 冲突窗口局部等待

## 状态

✅ **已在当前 Task08 交叉场景生成并由 MoveIt/FCL 联合时空检测复检为 SAFE。**

> 本阶段只生成候选轨迹，不向 Isaac 或 ROS 关节/吸盘 topic 发布控制命令。实际双臂执行留在后续执行子任务验收。

## 问题与方案

原始 Task08 候选在 `RETREAT` 结束后保持该姿态。交叉场景中，两臂最终 RETREAT 姿态仍会碰撞，因此单纯的启动延迟或中途等待均无法解决。

本扩展组合两个不放宽碰撞模型的动作：

```text
每臂：
HOME -> ... -> PLACE -> DETACH -> RETREAT
                              -> MoveIt SAFE_EGRESS -> HOME

联合：
原始候选 / 含 SAFE_EGRESS 候选
        -> Task08-B
        -> 若仍有有限冲突窗口：在 LIFT_TO_PRE_PLACE 前插入最小局部 HOLD
        -> Task08-B 10 ms 联合复检
        -> SAFE
```

约束：

- `SAFE_EGRESS` 只在 `enable_safe_egress:=true` 时生成，默认关闭，不改变 Task08-A/B 原有候选；
- 真正 `DETACH` 后才将 Box 以计划释放位姿作为 MoveIt World CollisionObject 加回；
- `RETREAT -> HOME` 由 MoveIt 规划，仍考虑已放置 Box；
- 不修改 ACM，不关闭 robot/object 碰撞，不永久忽略当前 Box；
- 最终安全性只以 Task08-B 私有 PlanningScene 的 FCL 全时域检测为准。

## 实际验证（2026-09-09）

### 一次需要局部等待的候选

```text
left  SAFE_EGRESS = 3.502 s
right SAFE_EGRESS = 3.517 s

原始联合冲突窗口 = [14.640, 17.580] s, ARM_ARM
Task09-B strategy = LOCAL_WAIT_LEFT
wait_start         = 9.383 s  (LIFT_TO_PRE_PLACE)
minimum_safe_wait  = 3.600 s
coordinated_makespan = 26.372 s
Task08-B verification = SAFE
```

### 独立重规划回归

另一组 OMPL 单臂退出路径在加入 SAFE_EGRESS 后已直接满足：

```text
Task08-B SAFE: NO_CONFLICT
Task09-B strategy = SIMULTANEOUS
minimum_safe_wait = 0.000 s
```

这两类结果均为正确解：若安全退出路径已避开对方，则保持并行；若仍有短暂窗口，则仅在进入公共区前让一臂等待。

### 最终复测

```text
原始联合冲突窗口 = [10.490, 16.280] s, ARM_ARM
Task09-B strategy = LOCAL_WAIT_LEFT
wait_start         = 8.672 s
minimum_safe_wait  = 4.800 s  (wait_step_sec = 0.20)
coordinated_makespan = 24.837 s
Task08-B verification = SAFE
```

OMPL 的空间路径具有随机性，因此等待数值会随单臂候选路径变化；每次运行都重新执行完整 Task08-B 验证，而不是复用历史数值。

## 运行

```bash
source /opt/ros/humble/setup.bash
source ~/lmy/dual-arm-embodied-palletizing/ros_ws/install/setup.bash

colcon build --packages-select fr3_dual_palletize --symlink-install
source install/setup.bash

ros2 run fr3_dual_palletize task08_candidate_demo \
  --ros-args \
  -p scenario:=task08_conflict \
  -p enable_local_wait_coordination:=true \
  -p enable_safe_egress:=true \
  -p wait_step_sec:=0.05 \
  -p max_wait_sec:=15.0
```

预期为 `Task09-B SAFE`，并打印 `SIMULTANEOUS` 或 `LOCAL_WAIT_LEFT/RIGHT`。任何 `NO_SOLUTION` 都表示当次单臂 OMPL 空间路径无法通过时间协调消解，绝不下发运动命令。
