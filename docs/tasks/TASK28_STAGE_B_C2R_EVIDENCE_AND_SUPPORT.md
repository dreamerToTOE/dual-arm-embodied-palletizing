# TASK28 Stage B-C2R：证据完整性与桌面支撑力

## 结果：B_C2R_BLOCKED；B0_PARTIAL 不变

唯一一次新物理运行在静止支撑预检中停止：**运行时质量及重力已确认，但记录的桌面反力与 Cube 的垂直动量收支不一致**。工具始终静止，未进入 APPROACH、CONTACT、FORCE_RAMP 或 EXIT；没有力跟踪或 FR3 侧推结果。此前 `B_C2_PARTIAL` 和 `B0_PARTIAL` 保持原判。

实验从原 Task27 场景加载原 0.12 m Cube、原桌面和复制的七形状四杯工具；工具为独立单轴刚体代理，不是 FR3。Cube 在物理启动前放置到原桌面，接触期间没有 Cube 位姿写入。原质量、桌面摩擦、重力、碰撞几何与检查没有为通过预检而更改。

## 证据管线

运行前的合成回归使用 **240 条逐步记录及 240 条接触事件**，覆盖多状态、NumPy 标量、NaN/Inf、丢失事件及注入写盘错误。运行器逐条写入并 flush；写盘/编码异常使控制停止，summary 标记失败，证据不完整不能被验收为成功。子进程包装器独立检查实际退出状态、事件/样本计数、状态顺序与证据存在性。连同状态机、预检、逐轴动量和既有力观测回归共 **45 passed**；七形状工具的 C++ FCL 检查通过 201 个预接触/退避采样点，穿入端点被拒绝。

新运行保留了 **13 条步后记录和 65 条接触事件**，与运行 summary 计数一致，无写盘错误；目标序列未完成，因此 `evidence_complete=false` 表示**任务证据不完整**，不是说这 13 条已保存记录被截断。[时间序列](../../offline/task28_stage_b_contact/evidence/task28_c2r_support_v1/timeseries.jsonl)、[原始事件](../../offline/task28_stage_b_contact/evidence/task28_c2r_support_v1/contact_events.jsonl)、[运行总结](../../offline/task28_stage_b_contact/evidence/task28_c2r_support_v1/summary.json)、[退出码](../../offline/task28_stage_b_contact/evidence/task28_c2r_support_v1/process_exit.json)及[六面板图](../../offline/task28_stage_b_contact/evidence/task28_c2r_support_v1/c2_timeseries.png)均保留。

实际 Isaac 启动脚本子进程退出码是 **0**，但其总结明确为 `FAILED`；外层证据验证器因此返回 **1**，没有把子进程的 0 误判为成功。代码审查发现 `app.close()` 在原先的最终退出码传递之前执行；本轮后将最终退出改为落盘并 flush 后直接退出，避免该路径吞掉失败码。此修正有离线进程包装测试，**尚未再次运行 Isaac 验证**；实际记录仍以上述 0/1 为准。

## 支撑力和逐轴动量核算

运行时 PhysX 读取 Cube 质量 **0.800000012 kg**、重力 `(0, 0, -9.81000042) m/s²`，预期重量 **7.84800045 N**。Cube 与桌面碰撞形状检查通过，两个成对力视图有效且矩阵形状为 `(1,1,3)`。每步 `dt=0.0166666675 s`，观测采集跨度约 0.302–0.383 ms，接触报告的回调步号与对应步后快照一致。Cube 在记录区间始终显示 `sleeping=false`。源场景记录 TGS 求解器、stabilization 开启，线性阻尼 0、角阻尼 0.05；具体稳定化内部冲量未独立输出。

预检使用至少 12 步、20% 重量相对误差和 20% 重力冲量残差上限，且至少 30 步通过才可进入接近；另保留力矩阵与接触报告差不超过 0.25 N 的检查。这些门限用于发现约 60% 的明显不一致，不是测量标定。运行器于第 12 个记录步首次判定失败，同一次 app update 还记录了第 13 步失败；两步命令均为零。13 步支撑力均值 **3.1517 N**，从 7.8463 N 降至 1.3844 N；矩阵与按同一步报告冲量换算的桌面法向力最大差约 `4.4e-7 N`。两条接口共享 PhysX，不能因此视为独立校准。

接触对包含每步 Cube/Table 一对，以及四个 Cup/Cube 的检测候选；四杯累计冲量均为零。候选事件由接触 offset 产生，不能当作实际工具载荷。没有记录到其他有载 Cube 接触；该结论限于订阅范围和记录窗口。工具/Cube 力始终 0 N，代理位置不变，Cube 靠近静止；无法据此证明最终接触后安全分离。

对第 1→13 步逐轴计算 `m Δv − J_tool − J_table − J_other − m g Δt`，使用**实际运行时质量和重力**；接触报告主体的法向冲量先按 Cube 为 collider0/collider1 确定符号。总残差 `(X,Y,Z)` 为 **`(-2.59e-5, +8.02e-5, +1.017519) N·s`**。独立读取的桌面摩擦数据积分约 `(-2.63e-5, +7.83e-5, 0) N·s`；进一步计入后残差约 **`(+3.76e-7, +1.88e-6, +1.017519) N·s`**。摩擦分量与水平动量变化的方向及量级一致，但无法解释 Z 向差异；其他外力/求解器内部冲量没有直接测量。逐步明细见[本次动量账](../../offline/task28_stage_b_contact/evidence/task28_c2r_support_balance_v2/per_step.jsonl)及[汇总](../../offline/task28_stage_b_contact/evidence/task28_c2r_support_balance_v2/metrics.json)。

对旧 83 步的[逐步重算](../../offline/task28_stage_b_contact/evidence/task28_c2r_previous_balance_v2/per_step.jsonl)仍得到 Z 残差 **8.789772 N·s**；旧运行时质量/重力及原始事件文件缺失，0.8 kg 与 9.81 m/s² 只能作为假设，旧摩擦冲量也未记录。新运行实际质量/重力与该假设一致，降低了这些假设错误导致新异常的可能，但不能追认旧运行的运行时值。

## 根因与下一步

- **已确认：**支持力不匹配在两次受支撑场景中出现；本次原始接触事件和力矩阵在同一步的 Z 向读数一致，Cube 未休眠，时间步有序。旧单杯离体诊断曾观察到另一 Cube 睡眠与速度归零同步，但这不能证明旧受支撑运行的原因。
- **尚不确定：**接触数据可能不包含 PhysX 稳定化或其他内部作用；也可能存在未记录的外力、读取/时间语义或其他求解器行为。表面法向与反力符号、接触事件记录匹配以及摩擦方向有实测支持，但不足以排除所有帧或时间错误。保留 **UNKNOWN**，不把两个同源接口的一致性视为独立力标定。
- **建议下一步：**进行一个独立、最小的静止原 Cube/原桌面支撑诊断，逐步同时读取运行时质量、重力、法向/摩擦和全部可观测外力，核对 PhysX 稳定化及 callback/子步语义；在量纲和动量闭合前维持现有预检，暂不运行力渐变或集成 B0。

本次代码还新增逐事件写盘、证据验证、静止支撑门和逐轴分析；未进行增益扫描、额外物理重试、RL 或 FR3 侧推。

离线复核命令：

```bash
PYTHONPATH=. python3 -m pytest -q offline/task28_stage_b_contact/test_c2_evidence.py offline/task28_stage_b_contact/test_c2_preflight.py offline/task28_stage_b_contact/test_c2_control.py offline/task28_stage_b_contact/test_c2r_balance.py offline/task28_stage_b_contact/test_force_probe.py
g++ -std=c++17 -O2 offline/task28_stage_b_contact/c2_geometry.cpp $(pkg-config --cflags --libs fcl) -o /tmp/task28_c2r_geometry
/tmp/task28_c2r_geometry
python3 offline/task28_stage_b_contact/c2r_balance.py offline/task28_stage_b_contact/evidence/task28_c2r_support_v1 NEW_OUTPUT_DIRECTORY
```
