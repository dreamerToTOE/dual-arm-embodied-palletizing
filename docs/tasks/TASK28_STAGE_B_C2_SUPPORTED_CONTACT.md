# TASK28 Stage B-C2：有支撑接触、力渐变和退出

## 结果：B_C2_PARTIAL；B0_PARTIAL 不变

唯一一次 Isaac Sim 4.5 物理实验产生了 **83 个完整步后样本**，均处于 `APPROACH`。运行结束后写出记录时，NumPy 布尔值无法被 JSON 编码，证据从第 84 条或更晚处截断，原始事件文件及运行总结均未写出。**不能判断之后是否建立接触、完成力渐变或安全退出**。按工单要求未重试。进程返回码为 0，但日志含未捕获异常；不能把该返回码当作实验成功。

| 项目 | 判断 |
| --- | --- |
| A. 先前隔离单杯成对力可观测性 | 已验证，见[先前报告](TASK28_STAGE_B_CONTACT_FORCE_OBSERVABILITY.md)；不是标定 |
| B. 桌面支撑下工具/Cube 接触 | 未验证；保留数据只有接近段 |
| C. 力渐变反馈 | 未验证；没有保留的渐变样本或力跟踪结果 |
| D. 完整 FR3 侧推 | 未执行；不适用代理结果 |

## 配置和前置检查

实验从原 Task27 场景开始：在仿真启动前将原 Cube_01 放到原桌面 `(0.5, 0, 0.26) m` 并开启重力，保留原 `0.12 m` 立方体与写入场景的 `0.8 kg` 质量属性。接触期间未对 Cube 写位置或速度。源场景桌面尺寸 `1.5 × 0.8 × 0.2 m`；源材质静摩擦 0.90、动摩擦 0.75、恢复系数 0。实验没有移除 Cube 碰撞体或关闭碰撞对。

侧推体是从原左工具直接复制的七形状碰撞几何：竖杆、横杆、歧管和四个半径/长度各 10 mm 的吸杯，吸杯相对偏移为 ±24 mm。它作为 **1 kg、仅允许 Y 平移的独立刚体代理**，以世界坐标力驱动，不代表 FR3。初始四杯接触面与 Cube −Y 面的名义间隙为 3 mm；对原工具七形状、Cube 和原尺寸桌面的 C++ FCL 检查覆盖 201 个接近/退回采样点，拒绝了穿入端点。此离线检查不包含双臂全场景，运行前的物理接触事件还用于拦截实际场景的其他碰撞。

实验意图使用按 Cube/工具和 Cube/桌面分开的 PhysX 成对力，以及接触报告中的路径、冲量与法向；步后记录带有物理步、仿真时间、状态、位置、速度、驱动力和传感器数据。控制器预设 APPROACH → CONTACT → FORCE_RAMP → EXIT；原计划参考力从 0.03 N 渐变至 0.15 N，而不是套用 4 N。它检查观测有效性、步号和时间、成对力与事件一致性、方向、额外碰撞、穿透、位移、速度、力和状态超时。超过限制或观测异常时命令归零并停止本次尝试。上述是实现与离线测试，**不是物理完成的证据**。

运行前针对状态转移、无接触、观测缺失/乱序/过期、错误坐标和方向、接触丢失、碰撞、过力、过穿入和运动限位的 Python 测试与 C++ FCL 检查通过。修复记录问题和加入下文的支撑力一致性保护后，离线回归为 **31 passed**，FCL 检查通过；这些后续修改没有再次执行物理实验。

## 已保留的物理结果

[逐步数据](../../offline/task28_stage_b_contact/evidence/task28_c2_supported_v1/timeseries.jsonl)、[异常日志](../../offline/task28_stage_b_contact/evidence/task28_c2_supported_v1/serialization_error.log)、[离线分析](../../offline/task28_stage_b_contact/evidence/task28_c2_supported_v1/analysis.json) 和[六面板图](../../offline/task28_stage_b_contact/evidence/task28_c2_supported_v1/c2_timeseries.png)均已保存。`summary.json` 是**事后依据截断记录和异常日志重建**，不是原始运行总结。接触事件文件缺失；图中接触对信息仅来自幸存的每步聚合字段，不能覆盖后续状态。

- 保存区间为 0.0167–1.3833 s；83/83 个样本均是 APPROACH。此区间工具/Cube 成对力为 **0–0 N**，工具沿 +Y 最多前进 **2.959 mm**，Cube Y 最大位移约 **0.000214 mm**。没有保留下来的力参考跟踪样本，无法计算力渐变误差。
- 此区间 Cube/桌面成对法向读数从 **7.846 N** 降至约 **1.177 N**，Cube 仍接近静止。矩阵值和同一步接触报告聚合值相近，但两者共享 PhysX，不能独立标定。该 83 步的 `unexpected_pairs` 字段均为空，不能证明缺失时段没有意外接触。
- 按原场景写入的 0.8 kg、假设世界重力为 `(0,0,-9.81) m/s²`，以幸存每步 Cube 总接触冲量加重力核算动量，Z 残差约 **+8.790 N·s**；运行时实际质量、重力和完整原始接触事件没有被保存。这个差异不能被解释为仅有工具/Cube 力，也不能忽略：现有数据不足以区分实际质量/重力、其他未记录作用、求解器或采样语义。下一次物理验证前必须确认这些量和完整作用力收支。

## 旧速度归零现象

旧单杯 v4 在第 142→143 步由 `0.01828876 m/s` 变为 0；Cube 线性阻尼在该探针中设为 0、重力关闭，代码没有在接触后写 Cube 位姿或速度，旧日志没有记录休眠/唤醒状态。没有其他 Cube 碰撞对的记录；求解器稳定化、角阻尼及外部状态更新的运行时状态没有足够证据排除。

本次同一场景内另设了不与实验体接触的 Cube 几何副本：关闭重力与线性阻尼，在开始时一次性给定相同的 `0.01828876 m/s`，逐步查询 `PhysXSimulationInterface.is_sleeping`。第 24→25 步，速度恰在 `sleeping: false → true` 时归零，时间为 **0.4167 s**。这提供了 PhysX 休眠可以产生该现象的直接证据，支持旧现象由休眠导致的假设；由于旧运行未记录激活状态，**旧运行的确切原因仍为 UNKNOWN**，不能宣称已经证明。

## 修正、限制和下一步

错误发生在实验结束后的批量 JSON 写盘，导致后续数据和事件丢失。现已将证据改为每步/每事件即时写入并 flush，显式处理 NumPy 标量，写盘错误导致控制器停止；针对 `numpy.bool_` 和非有限数据添加了回归。针对本次发现的支撑力不一致，后续代码还要求运行时确认质量和重力，并在无工具接触的初始支撑期检查支持力与重量的大致一致性；该新增保护尚未经过物理验证，不能静默放宽。

下一步先修复并验证证据链，定位 Cube/桌面冲量与完整动量收支的差异，包括运行时质量、重力、接触对、摩擦、睡眠/稳定化及同步时序。确认这些前置条件后，再进行一个新的有界完整工具代理 CONTACT → FORCE_RAMP → EXIT 实验。保持 B0 保护门和原始失败证据；不应据本轮结果接入 FR3 或宣称力跟踪通过。

## 变更和复现

- `isaac/scripts/task28_c2_supported_contact.py`：原场景代理、休眠诊断和一次有界执行；未来运行写入新的 run ID。
- `offline/task28_stage_b_contact/c2_control.py`：本探针的状态机、传感器保护与 JSON 编码。
- `offline/task28_stage_b_contact/c2_geometry.cpp`、`test_c2_control.py`：离线碰撞及负例测试。
- `offline/task28_stage_b_contact/analyze_c2.py` 与 `evidence/task28_c2_supported_v1/`：截断数据的分析、图和故障证据。

```bash
PYTHONPATH=. python3 -m pytest -q offline/task28_stage_b_contact/test_c2_control.py offline/task28_stage_b_contact/test_force_probe.py
g++ -std=c++17 -O2 offline/task28_stage_b_contact/c2_geometry.cpp $(pkg-config --cflags --libs fcl) -o /tmp/task28_c2_geometry
/tmp/task28_c2_geometry
python3 offline/task28_stage_b_contact/analyze_c2.py offline/task28_stage_b_contact/evidence/task28_c2_supported_v1
```
