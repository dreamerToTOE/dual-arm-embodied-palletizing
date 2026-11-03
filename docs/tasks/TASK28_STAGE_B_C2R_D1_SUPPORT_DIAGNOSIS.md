# TASK28 B-C2R-D1：最小 Cube/桌面支撑力诊断

## 结论：D1_PARTIAL

独立最小场景中，Cube 清醒时的桌面法向力与重量和逐步动量**一致**；Cube 进入 PhysX 休眠后，成对力和接触事件均为零，而 Cube 保持静止。休眠状态下把每个回调都当作受重力积分的活动刚体，会构造出不适用的残差。

这解释了**本次最小诊断的休眠段**，但没有解释先前 C2R 场景中 Cube 明确处于清醒状态时支撑力从 7.846 N 降至 1.384 N 的现象。该原始异常根因仍为 **UNKNOWN**；不应把此次睡眠发现追认为旧运行的根因。**B_C2R_BLOCKED 与 B0_PARTIAL 不变。**

## 配置与证据

唯一一次运行在 Isaac Sim 4.5 中创建独立临时场景，仅有一个动态 Cube、一个静态桌面、原摩擦材质和物理场景；没有 FR3、吸盘工具、控制器或轨迹。Cube 为原 `0.12 × 0.12 × 0.12 m`、运行时质量 `0.800000012 kg`；桌面为原 `1.5 × 0.8 × 0.2 m`，顶面 Z=0.2 m，Cube 初始中心 Z=0.26 m。重力 `(0,0,-9.81000042) m/s²`，静摩擦 0.90、动摩擦 0.75、恢复系数 0；TGS、CCD 和 stabilization 均按原 Task27 场景启用。线性阻尼 0、角阻尼 0.05，位置/旋转锁定均为 0。没有重力、材质或质量扫描，也没有更改项目原场景文件。

场景先静置 90 次应用更新，在开始记录前唤醒 Cube 一次，之后没有写 Cube 位姿、速度或外力。随后的 **240 个物理步、约 4.0 s** 全部记录，运行返回码 0。观测窗口内只有 Cube 和桌面碰撞对；报告共 24 条，均为 `/World/Cube_01 <-> /World/Table`。完整 prim 清单、阈值与配置在[运行总结](../../offline/task28_stage_b_contact/evidence/task28_d1_support_v1/summary.json)。

- [逐步数据与动量账](../../offline/task28_stage_b_contact/evidence/task28_d1_support_v1/timeseries.jsonl)：每步包含运行时质量/重力、时间与 dt、步前/步后速度、位置、休眠状态、矩阵力和原始冲量、摩擦、法向与接触点、逐步和累计残差。
- [原始接触事件](../../offline/task28_stage_b_contact/evidence/task28_d1_support_v1/contact_events.jsonl)、[汇总](../../offline/task28_stage_b_contact/evidence/task28_d1_support_v1/analysis.json)、[支撑力、速度、位移和残差曲线](../../offline/task28_stage_b_contact/evidence/task28_d1_support_v1/support_curves.png)、[退出码](../../offline/task28_stage_b_contact/evidence/task28_d1_support_v1/process_exit.json)。灰色曲线区间表示步后读取到的休眠状态。

## 测量结果与物理含义

| 项目 | 测量 |
| --- | ---: |
| 物理回调 dt | 每步 `0.0166666675 s`；配置 60 Hz |
| 清醒 / 休眠步数 | 24 / 216；第 25 步进入休眠 |
| 清醒时 Cube/桌面 Z 法向力 | 均值 **7.847575 N**，范围 **7.838227–7.848058 N** |
| 运行时 `m g` | **7.848000 N** |
| 清醒步逐轴残差累计 | `(2.45e-10, −9.30e-11, 1.16e-7) N·s` |
| 休眠步矩阵法向力 | 216/216 为 0 N；无持续接触报告 |
| 整个窗口 Cube 最大速度 / 相对首样本最大位移 | `0.0002474 m/s` / `7.25e-8 m` |
| 接触矩阵原始冲量与事件冲量最大分量差 | `1.31e-8 N·s` |
| 矩阵 `force × dt` 与原始冲量最大分量差 | `5.10e-9 N·s` |

每个清醒步按世界坐标核算 `m(v_after − v_before) − J_event − J_friction − m g dt`；事件按碰撞体顺序转换为作用于 Cube 的法向冲量，摩擦从 PhysX friction data 另取。清醒段的运动反推总接触冲量与报告的法向及摩擦冲量闭合到上述数值。两个接触接口共享 PhysX 后端；运动和质量/重力核算提供不同的物理一致性检查，但不构成外部传感器标定。

第 25 步步后变为休眠，速度归零，接触矩阵和持续报告随之归零。若仍在所有休眠回调中机械代入 `m g dt`，会得到约 **28.253 N·s** 的累计“残差”；这一数字不能解释为实际缺失的支撑冲量或地面消失。第 25 步是状态转换，转换时的内部处理没有单独测量。PhysX 官方接口明确指出：接触双方休眠时不发送持续接触报告；休眠刚体的线速度和角速度为零。[PhysX 接触报告标志](https://nvidia-omniverse.github.io/PhysX/physx/5.1.3/_build/physx/latest/struct_px_pair_flag.html)、[PhysX 刚体休眠状态](https://nvidia-omniverse.github.io/PhysX/physx/5.1.3/docs/RigidBodyDynamics.html)。这里的数值属于仿真地面状态，并非现实力传感器输出。

步前回调 dt 与步后 dt 一致；本次 240 个应用更新各收到一个物理步回调，且每个步前速度与前一已记录步后的速度一致。本次没有发现可见的状态覆盖。该检查不证明 PhysX 内部求解器没有迭代或更细的内部处理，也不能证明接触报告暴露了所有内部求解器贡献。官方文档将 stabilization 描述为改善低速对象收敛的额外阻尼，并提示它可能影响动量；本实验未切换该设置，无法将旧异常归因于它。[Omni Physics 稳定化说明](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/dev_guide/simulation_control/simulation_control.html)。

## 对工单问题的回答

1. **没有工具代理时是否出现同样异常？** 清醒的 24 步没有出现旧运行那样的支撑力持续衰减；进入休眠后读取为零是另一个可识别的状态。最小场景不能排除原场景中多视图、其他场景对象或执行时序的影响。
2. **Cube 静止而报告不足是否重现？** 休眠期间是，但没有持续接触报告且不应按清醒刚体的重力步方程核算；清醒期间报告接近重量、逐步动量闭合。
3. **缺失求解器贡献、报告不完整、单位或状态覆盖是否得到证明？** 当前测得清醒段没有显著残差，`dt` 缩放和同源报告一致；休眠段没有持续报告有官方语义支持。对旧 C2R 清醒段，仍不能从本次证明或排除内部求解器贡献、其他外力/状态写入、多个视图相互影响或不同时间语义。先前观测的 13 步全为 `sleeping=false`，不应简单用休眠解释。
4. **接口是否可作定量力反馈？** 本次清醒、静态、单视图条件下法向数据通过重量和运动一致性检查；它在休眠时输出零，且旧完整场景清醒异常未解决，因此尚不能作为原场景可靠的连续定量反馈。不能把本结果称为传感器标定或 FR3 力控验证。

下一步建议在原 C2R 场景中做受控的**单视图与双视图接触读取、回调顺序及激活状态对照**，逐步记录原始 `dt=1` 冲量和 actual-dt 力，并保持现有支撑预检。对休眠应显式区分“无新报告”与“无接触”，不要伪造非零力；如需要连续反馈，可评估明确记录唤醒状态的读取策略，或以 `m(a−g)` 减去已知外力估计净接触力。后者将多个接触及未知外力混在一起，且加速度微分易受噪声影响，不能直接替代成对力传感。

## 验证与复现

物理运行前离线回归 **21 passed**，包括旧 C2R 残差重放；加入本次真实数据回归后 **22 passed**。实验失败时不自动重试；本次未运行 B0、FR3 侧推、力渐变或学习任务。

```bash
PYTHONPATH=. python3 -m pytest -q offline/task28_stage_b_contact/test_d1_math.py offline/task28_stage_b_contact/test_c2r_balance.py offline/task28_stage_b_contact/test_c2_evidence.py offline/task28_stage_b_contact/test_c2_preflight.py
DISPLAY=:1 OMNI_KIT_ACCEPT_EULA=YES ROS_DOMAIN_ID=157 /home/ubuntu2004/isaacsim-4.5.0/python.sh isaac/scripts/task28_d1_support_probe.py --run-id NEW_UNIQUE_ID
python3 offline/task28_stage_b_contact/analyze_d1.py offline/task28_stage_b_contact/evidence/NEW_UNIQUE_ID
```
