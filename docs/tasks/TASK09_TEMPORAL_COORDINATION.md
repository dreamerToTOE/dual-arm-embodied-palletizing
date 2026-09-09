# Task09-A：松协调延迟启动

## 状态

✅ **已实现并完成 MoveIt/FCL 只读联调。**

## 边界

Task09-A 只消费 Task08 的两条完整 `TaskTrajectoryCandidate` 与 `ConflictReport`：不改关节路径、不改 Task04 抓放时序、不下发控制命令。

```text
simultaneous candidate -> CONFLICT
-> 枚举 delay left / delay right
-> 每个策略仍以 10 ms 联合碰撞检查
-> 最小安全 delay 或 NO_SOLUTION
```

默认优先左臂：同一 delay 下先尝试 `DELAY_RIGHT`。搜索步长 `0.25 s`，最大延迟 `30 s`；所有候选都通过 Task08-B 的真实几何检查，而不是仅比较时间区间。

## 入口

Task08 交叉场景：

```bash
ros2 run fr3_dual_palletize task08_candidate_demo \
  --ros-args -p enable_temporal_coordination:=true
```

预期输出 `Task09-A SAFE`（含策略与两臂启动延迟），或明确 `Task09-A NO_SOLUTION`。两者均不会执行机器人或吸盘命令。

Task09-A 只验证时间协调；将计划真正下发到双臂执行、优先级切换与局部重规划留给后续子任务。

## 实测结果（2026-09-09）

Task08 交叉场景已实测得到 `Task09-A NO_SOLUTION`：在 `max_delay_sec` 范围内，整条轨迹延迟无法消除所有冲突。这一结果保留为 Full-task Start Delay baseline，不将其误标为可执行解。后续的局部等待实现与终态冲突分析见 [TASK09_LOCAL_WAIT.md](TASK09_LOCAL_WAIT.md)。
