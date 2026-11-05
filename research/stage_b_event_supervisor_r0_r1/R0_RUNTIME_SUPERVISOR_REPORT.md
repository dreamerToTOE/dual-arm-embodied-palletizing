# R0 运行时事件监督与门禁重构报告

## 结论

R0 状态：`R0_GATE_REFACTOR_PARTIAL`。

自由空间长轨迹已经从局部 QP 门禁中分离，并在 Isaac Sim 中完成了实际 FR3 运动。Cube 推进的单帧 lag 终止改成了新鲜观测、时间窗口和可恢复停顿。R0 仍为 partial，因为同一执行进程内自动把位置轨迹交给局部 QP 的完整 handoff 尚未实现；现阶段必须显式选择 `FREE_SPACE_TRAJECTORY_POSITION_DRIVE` 或 `LOCAL_CONTACT_QP`。

## 代码改动

- `isaac/scripts/task28_runtime_events.py` 新增 typed event interface。事件类别为 `NORMAL_OPERATION`、`RECOVERABLE_TASK_EVENT`、`CONTROL_FEASIBILITY_EVENT`、`OBSERVATION_UNAVAILABLE` 和 `FATAL_EXECUTION_ERROR`，并记录状态、下一状态、理由和时间戳。
- `task28_task27_one_pass.py` 在 free-space 模式只启动原始位置 feed bridge，在 live-QP 模式才启动 torque bridge；runner 写入 `runtime_events.jsonl`，异常不再统一伪装成普通任务失败。
- `task26_truck_box_push_in.cpp` 增加同名 C++ runtime event 分类和日志。Cube 推进检查 `CubeBuffer::Receipt` 的新鲜度，单帧 lag 只发 `SUSPECTED_STALL`；连续两个前进窗口且实际 Cube 前进小于 1 mm 才停止为 `SUSPECTED_STALL_CONFIRMED`。到达深墙 x 目标记录 `WALL_SEATED`。
- `kPushJamTolerance=0.020`、`kWallFlushTolerance=0.003`、`kPushTorqueLimit=80.0` 和局部 QP 20 mm envelope 均保持不变。

## 状态转换

```text
FREE_SPACE --正常位置轨迹--> FREE_SPACE
FREE_SPACE --显式控制交接--> TOOL_CONTACT / LOCAL_CONTACT_QP
INSERTION --预期推进--> INSERTION
INSERTION --到达深墙--> WALL_SEATED
INSERTION --连续 lag + 低实际进展--> HOLD / SUSPECTED_STALL_CONFIRMED
任一控制模式 --QP/力矩/跟踪失败--> HOLD 或 STOPPED
任一模式 --缺少或陈旧观测--> OBSERVATION_UNAVAILABLE / HOLD
任一模式 --非有限状态或无法保持 controlled stop--> STOPPED
```

`HOLD` 只表示停止当前不兼容命令，不等于允许盲目重试。恢复候选必须重新验证观测、所有权、碰撞和轨迹；本分支没有实现无限重试。

## 回归与故障注入

`research/stage_b_event_supervisor_r0_r1/tests` 的 15 个测试通过：

- 长自由空间模式的 runner 使用位置 bridge，且不启动 live QP。
- event journal 能把正常运行、可恢复 stall 和 fatal stop 写成不同事件。
- QP、陈旧观测、placement 失败的异常分类不同。
- 保留的 20 mm/3 mm/80 Nm 约束仍在源码中。
- 既有 R3/R4 接触证据和本次 R1 接触证据中存在工具/Cube、深墙/Cube 事件。
- 本次 R1 的 `runtime_events.jsonl` 为 `RUN_STARTED → FREE_SPACE_POSITION_DRIVE → TRIAL_COMPLETED`。
- A–I 故障注入覆盖长自由空间、tracking divergence、Cube stall、深墙到达、异常碰撞、陈旧观测、QP infeasibility、placement failure 和 attachment unavailable。

overlay 构建命令：

```bash
source /opt/ros/humble/setup.bash
colcon --log-base ros_ws/log build --merge-install --base-paths ros_ws/src \
  --build-base ros_ws/build --install-base ros_ws/install
```

结果：6 个包完成，仅有既有 CMake 弃用和格式 warning。

## 仍保留的门禁

FCL/IK、桌面/墙体/已放置 Cube 的碰撞物、关节位置/速度/加速度、torque 与 torque-rate、QP 求解状态、非有限状态、观测新鲜度、suction 状态读取、settle timeout 和最终 placement acceptance 均保留。`planning_only` 仍不发布运动命令，也不作为物理成功证据。

高 D6 force/torque 限值没有被解释为真实吸附反馈。R1 使用 PhysX contact report 和 articulation/Cube runtime state；没有启用未经验证的 force-feedback 控制。
