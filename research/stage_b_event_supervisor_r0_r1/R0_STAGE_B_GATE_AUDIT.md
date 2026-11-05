# R0 Stage B 活跃运行门禁审计

审计基线为 `548492a92a76fcfe497872c8a40cd380d2ac656a`，范围是 Task27 Isaac/ROS 实际执行链。审计只记录源码中已存在的门禁；没有把规划探针或历史日志中的假设当作运行门禁。

| 门禁 | 源码位置 | 触发条件和原目的 | 运行范围 | 新行为与仍需停止的条件 | 验证 |
|---|---|---|---|---|---|
| QP 局部扫掠 | `isaac/scripts/task28_task27_torque_bridge.py:1181-1194` | 相对或单臂 TCP 从固定锚点超过 20 mm，保护局部 QP 线性化域 | 仅 `--live-qp`；原先会把长自由空间轨迹误当成局部 QP 运动 | 保留 20 mm。自由空间由原始位置 bridge 独占；局部 QP 只在显式控制模式使用。超过局部域仍产生 `CONTROL_FEASIBILITY_EVENT` 并停住 | 历史 `g1a_fr3_f1_attempt02` 证明第 607 步触发，QP 状态仍为 `QP_OPTIMAL`；R3/R1 位置轨迹完整执行 |
| 驱动所有权接管 | `task28_task27_torque_bridge.py:375-404`；`task28_task27_one_pass.py:624-676` | Torque bridge 即使 `live_qp=false` 也把 7 个关节切到 force mode，旧 runner 还会捕获位置参考而不执行 | 实际 runner | 位置模式不启动 torque bridge，原始 `Task26BatchedFeedBridge` 是唯一 arm owner；QP 模式仍显式接管且检查手指策略 | R3 `FREE_SPACE_TRAJECTORY_POSITION_DRIVE`, `executor_exit=0`；R1 同样通过 |
| 轨迹/IK/FCL 规划失败 | `task26_truck_box_push_in.cpp:3340-3584`、`4100-4260`、`4923` 等 | 规划、IK、同步 FCL 或线偏离不通过，避免执行未知轨迹 | 物理执行与 planning-only 都生效 | 保留完整场景、Cube、桌、墙和已放置物体；失败保持当前状态或安全释放，不把 planning PASS 当物理 PASS | 完整 overlay 构建；R1 executor 日志含同步 FCL PASS 与物理执行 |
| Cube 单帧 lag | 原实现 `executePushWithSupervision()`，基线约 `2629-2703` | `abs(commanded_x-measured_x)>20 mm` 即返回 false | 推入段 | 保留 20 mm 数值；先检查新鲜 receipt，再要求连续两个前进命令窗口且 Cube 前进小于 1 mm 才发 `SUSPECTED_STALL_CONFIRMED`。单帧只记录可恢复事件；到达深墙 x 目标记为 `WALL_SEATED` | C++ 构建；故障注入测试检查双样本和 stale gate |
| Cube 观测新鲜度 | `task26_truck_box_push_in.cpp:630-729`、新 `CubeBuffer::latest()` 与推进检查 | 原路径可直接读取缓存 pose，没有 receipt age 判定 | 推入段 | receipt 缺失或超过 0.50 s 进入 `OBSERVATION_UNAVAILABLE`，不以缓存值继续推 | C++ 构建；测试检查 `PUSH_CUBE_OBSERVATION_STALE` |
| 力矩上限 | `task26_truck_box_push_in.cpp:358-364`、新推进检查 | 推入峰值超过 80 Nm 时停止，避免继续施加未知接触载荷 | 推入段 | 保留 80 Nm；分类为 `CONTROL_FEASIBILITY_EVENT`，需要 hold/诊断后才能另行规划 | R1 峰值约 30 Nm，未触发 |
| QP 速度、模型、数值和求解失败 | `task28_task27_torque_bridge.py:1195-1217`、`1592-1631`、`1877-1915` | 非有限状态、速度越限、QP 非最优或输出非法会 fail-closed | `--live-qp` | 保留硬停；runner 将异常分成控制可行性、观测不可用、可恢复任务事件和 fatal，不能把异常包装成正常接触 | Python 事件单测、torque bridge 源码审计 |
| 最终 placement acceptance | `task26_truck_box_push_in.cpp:2559-2668` | 位置误差、深墙 gap、侧墙 gap 不满足原始阈值即任务失败 | 所有物理 placement | 保留 3 mm wall-gap 与原有 cell/side gap 条件；失败为 `WALL_NOT_SEATED` 或待诊断事件，不能报告成功 | R1 final cell error 1.716 mm，深墙 gap 0.367 mm，侧墙 gap 1.676 mm |
| 关节 settle / timeout | `task26_truck_box_push_in.cpp:1263-1380`、`kJointSettleTimeoutSec` | 到位误差、稳定残差、等待超时 | 所有执行阶段 | 保留；没有可验证 hold 就停止，不盲目继续 | R1 executor 多次 settle 后完成 |
| 吸盘与 attachment 假设 | `task26_truck_box_bridge.py:336-447`、`task28_task27_torque_bridge.py` 的 suction 状态路径 | Surface Gripper 命令和状态属于仿真 bridge；高 D6 限值不是实测吸附模型 | 所有接触阶段 | suction command、closed state、PhysX tool/Cube contact 分开记录；不启用未经验证的 force feedback，不宣称吸附模型已验证 | R1 contact trace 来源为 PhysX pair report；无独立 suction-force 结论 |
| 全局异常到 stop | `task28_task27_one_pass.py:781-802`；torque bridge `1877-1915` | 防止未处理异常继续写命令 | 两层均存在 | runner 现在对异常写 typed event；torque bridge 内部仍 fail-closed。只有存在已验证 hold 的任务事件才可由上层恢复 | Python 单测、R1 `runtime_events.jsonl` |

保留的硬停止包括：非有限 robot state、未知/冲突的 actuator ownership、QP 非可行、torque/velocity/position 约束失败、观测不可用、未通过 FCL 的轨迹、无法维持 controlled hold、以及最终接受条件失败。此次没有提高或关闭 20 mm、3 mm、80 Nm 或碰撞约束。

当前限制是自动的“自由空间 → 局部 QP”同一进程中间交接仍未实现；本分支提供显式模式边界和事件接口，长自由空间段继续由位置 bridge 执行，局部 QP 仍需显式从已同步状态启动。
