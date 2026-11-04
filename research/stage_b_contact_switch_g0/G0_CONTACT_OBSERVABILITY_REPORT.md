# G0 接触可观测性报告

## 结论

本阶段在 Isaac Sim 4.5 中完成了 CPU/ GPU 物理运行（渲染 headless，未运行 FR3 控制器），但只报告接触观测，不报告力控、插入成功或恢复成功。总体结果为 **`G0_PARTIAL`**：

- 隔离刚性工具/Cube 接触链在三次重复中通过步对齐、接触身份、方向、冲量/力换算、撤回和动量核对，可标记 `VERIFIED`（限定于该 proxy 场景）。
- 最小 Cube/Table 在醒着时段三次补充重复均通过；进入睡眠后连续接触报告停止，不能把零报告解释成零支撑，因此 Cube/Table 总体为 `PARTIAL`。
- Cube/DeepWall 只有一次加载成功记录，未完成可重复卸载；两次有界失败保留，故为 `PARTIAL`。
- fuller Task27 场景预检仍显示支撑力/动量失配，禁止把最小场景结果转移为控制器可用的 fuller-scene 力通道。
- 吸盘状态未采样，为 `UNOBSERVABLE`。

因此没有启动 G1–G4、RL、真实硬件或物理恢复实验。

## 运行环境与隔离

- 基线分支：`side-suction-palletizing`，提交 `91bcf85cd32df91298a6ec428ada1d7563174260`。
- 研究分支：`research/stage-b-contact-switch-g0`，独立 worktree。
- Isaac Sim：4.5.0，`isaac/scripts/task28_launch_bundled_humble.sh`，headless、单实例。
- 机器预检：16 CPU threads、约 46 GiB RAM、RTX 3070 8 GiB；本阶段没有并行启动多个 Isaac Sim 实例。
- 场景约束：Cube 运行时质量约 0.8 kg，重力 `[0,0,-9.81] m/s²`，摩擦静态 0.90/动态 0.75、恢复系数 0，TGS、stabilization、CCD 保持开启。未修改生产场景或关闭相关碰撞。

接受门槛记录在 [`protocol_manifest.json`](protocol_manifest.json)。必须诚实记录一个流程偏差：该协议文件在首轮物理运行之后才补齐；阈值来自既有 D1/C2 实现、历史 7.848 N 参考和固定物理意义，未根据最终结果回调，因此该偏差计入总体局限，而不是伪装成事前注册。

## 观测实现

新增/复用的证据脚本：

- [`scripts/g0_c1_table.py`](scripts/g0_c1_table.py)：最小 Cube/Table，记录 pre/post pose、四元数、线/角速度、睡眠、contact event、矩阵力、原始冲量、法向/摩擦分量和动量残差。
- [`scripts/g0_c2_wall.py`](scripts/g0_c2_wall.py)：最小 Cube/Table/DeepWall；只通过 dynamic-control 施加有界世界坐标 +X 物理力， measured contact 段不写 Cube pose/velocity；记录间隙、墙矩阵力/冲量、事件时间和卸载状态。
- [`scripts/g0_c3_tool.py`](scripts/g0_c3_tool.py)：复用现有孤立刚性 tool proxy，记录接近、加载和撤回。
- [`scripts/g0_c4_full_scene.py`](scripts/g0_c4_full_scene.py)：原始 Task27 fuller scene + 复制工具 proxy，只进入预检，不进入 FR3 执行。
- [`scripts/observer_math.py`](scripts/observer_math.py)：有限值、步对齐、法向投影和 `force = impulse / actual physics dt` 的纯函数；对应测试在 [`tests/test_observer_math.py`](tests/test_observer_math.py)。

所有 JSONL 记录都保留实际物理 `dt`。`dt=1` 的矩阵只用于保留原始冲量；没有使用 wall-clock dt。contact-report 与矩阵都来自同一 PhysX 求解器，不宣称为独立传感器。

## 实验清单与结果

共保留 **13 个 Isaac Sim 进程尝试**：C1 六次、C2 三次、C3 三次、C4 一次。每个 run 有唯一目录、`summary.json`、原始 `timeseries.jsonl`、`contact_events.jsonl`（如有）和 `.process.log`；真实状态和非零退出记录在 [`run_manifest.json`](run_manifest.json)。

### C1：最小 Cube/Table

C1-01–03 是原始 D1 观察器的完整重复；C1-04–06 是加入 pre/post pose 与角速度字段后的完整重复。六次均有 240 个物理步、24 个 contact events，质量 `0.8000000119 kg`，重力 `-9.81000042 m/s²`。

三次关键 patched 重复（04–06）完全一致：

- 醒着样本 24（步骤 1–24，约 0.0167–0.4000 s），睡眠样本 216，第一睡眠步 25。
- 醒着 Cube/Table 法向力均值 **7.847575 N**；历史期望 `m|g| = 7.848000 N`，相对误差约 **0.0054%**。
- 醒着逐步未解释冲量最大绝对分量 `[2.78e-10, 3.43e-10, 1.71e-8] N·s`；矩阵力乘真实 dt 与原始矩阵冲量最大差 `5.10e-9 N·s`。
- pre-step 速度严格对齐上一行 post-step 速度（最大差 0）；接触法向向上，力/冲量方向和单位一致。
- 睡眠期间矩阵连续力为零且 contact event 不再连续出现；这表示 PhysX 不再对静止睡眠体提供活动步报告，不能套用醒着状态的 `mΔv = (F_contact + mg)dt`。

这验证了最小醒着支撑参考和睡眠语义，但没有解决历史 fuller-scene 的支撑力衰减，所以 Cube/Table 总体只标记 `PARTIAL`。

### C2：Cube/DeepWall

C2-01 保留了实现 bug：`balance` 收到标量重力，导致 `TypeError`，0 个样本、1 个事件，进程非零退出；随后已修复并完成回归测试。C2-02 使用 `+8 N` 激励、加载期 `+1 N`、撤回期 `-5 N`，300 个样本、552 个事件；C2-03 将撤回改为 `-12 N`，在 92 个样本后触发明确的 `MOTION_BOUND_EXCEEDED`，保留为失败证据。

C2-02 的可解释片段：

- 首次加载步 **49**（约 0.817 s），墙间隙降到约 `1.8e-7 m`；Cube/Wall 矩阵法向力峰值的大小 **38.9648 N**，事件冲量和矩阵冲量在同一物理步匹配。
- Cube/Table 仍提供约 9.39 N 的 z 支撑；墙力方向为 Cube 受到的 -X 方向，图中使用有符号值，报告峰值用绝对法向大小。
- 加载期间几何接触、事件、矩阵力和运动一致；但 `-5 N` 撤回后墙仍保持加载，没有满足“力降至 0.02 N 以下且持续分离”的卸载门槛。
- 因此一次成功加载不能升级为可用于自适应力控的 VERIFIED 通道；C2 结论是 `PARTIAL`。

### C3：隔离刚性 Tool/Cube proxy

C3-01–03 三次完整运行均通过现有 `analyze_force_probe.py` 的全部检查：210 样本、143 pair events，接触步 97–126（1.6167–2.1000 s），无其它 Cube 接触事件，撤回后力低于 `1e-5 N`。

代表性数值（三次一致）：

- 峰值工具对 Cube 的 +Y 力 `0.0563907 N`，加载段平均 `0.0279652 N`；横向分量低于 1%。
- tool 位移 `-0.004981 m`，Cube 位移 `+0.009770 m`。
- 截止最后接触步的 Cube 动量变化 `0.0146310076 N·s`，pair 原始冲量 `0.0146310028 N·s`，差约 `4.8e-9 N·s`；报告力乘真实 dt 与事件冲量的最大差 `4.36e-11 N·s`。

该结果验证了“隔离刚性碰撞 proxy 的接触观测链”，不验证吸盘密封、吸附力或真实工具闭链。

### C4：fuller Task27 场景转移

C4-01 在进入 FR3 执行前的预检即失败，保留 12 个样本、60 个事件，进程非零退出；没有进行接触控制。运行时质量和重力仍正确，但：

- 期望支撑 `7.848000 N`，实测均值 `3.299012 N`，相对误差 **57.96%**；
- 动量残差 `[-2.51e-5, 7.97e-5, 0.909792] N·s`；
- 因此 fuller-scene 观测链被标为 `UNRELIABLE/BLOCKED`，不能把它解释为已解决的最小场景结果。

## 同步、单位和物理一致性审计

完整审计表见 [`observability_audit.json`](observability_audit.json)。关键结论如下：

1. 每个采样行有连续 step ID、simulation time 和实际 physics dt；C1 patched 记录 pre/post pose/quaternion 与线/角速度，C2/C3 也记录关键刚体运动。
2. 矩阵接口以 `dt` 返回力，原始 contact report 以 N·s 返回冲量；所有比较先使用 `F·dt`，没有把 wall-clock 时间代入。
3. 接触身份使用 collider path 集合；几何间隙、法向和接触点与事件同一行保存。接触 offset 事件不自动升级为 loaded contact。
4. 醒着刚体的动量核对包括 Cube 质量、重力冲量、矩阵/事件接触冲量和摩擦；睡眠行不做活动体平衡。
5. 角速度和四元数字段已记录，但本轮不据此宣称姿态控制或角动量闭合；后续若用于控制，需增加刚体惯量和角动量项。

## 失败模式与剩余不确定性

- C2-01 的标量重力 bug 已修复；原始失败目录和非零状态保留。
- C2-03 的较强撤回触发运动边界，说明当前实验动作没有形成可重复的卸载窗口；没有继续无限调参。
- C4 fuller-scene 支撑/动量预检仍失败，且与历史“Cube awake 时支撑下降”问题一致；原因没有在 G0 中猜测或掩盖。
- 睡眠报告停止是观测语义问题，不是物理支撑消失的证据。
- C3 的 proxy 是碰撞几何替代物，不能支持吸盘或双臂闭链结论。
- 早期协议文件生成顺序偏差降低了 G0 的审计强度；后续实验必须在首个物理运行前固定协议。

## G0 判定与最小下一步

**G0 = `G0_PARTIAL`。** 当前允许的下一步是一个最小、固定初态的墙 proxy 诊断实验：重复 C0 固定位置与 C1 几何触发停止/顺应保持，先只使用几何间隙、Cube 运动和接触身份；在墙力通道通过三次加载+卸载、fuller-scene 支撑预检通过之前，不加入力反馈分支。所需最小新增 primitive 是“有界、可撤回的墙接触动作/卸载动作”；若该 primitive 仍无法提供可重复卸载，则 G1 对墙接触的结论应为 `INCONCLUSIVE`，而不是制造失败来支持层级控制。

没有任何本报告结果证明物理 jam recovery、力控成功、吸盘完整性或真实任务成功。
