# TASK28 B-C2R-D2：接触视图和回调隔离

## 结论：D2_PARTIAL

在两组初始状态匹配的原 Task27 场景诊断中，仅使用 Cube/Table 视图的 A 和同时使用 Cube/Table、Cube/Tool 视图的 B，**均重现清醒时支撑力衰减，且两条曲线逐步相同**。因此增加第二个接触视图并非在本设置中复现该异常的必要条件。原 C2R 清醒期异常的具体根因仍为 **UNKNOWN**；D1 的清醒静态闭合与 D2 的异常也尚不能归因于某一个场景差异。

**B_C2R_BLOCKED 和 B0_PARTIAL 不变。** 本次没有侧推、力渐变、完整 B0 或 FR3 运动。

## D1、C2R 与本次实现对照

| 项目 | D1 最小场景 | 先前 C2R | 本次 D2 A/B |
| --- | --- | --- | --- |
| 物理场景 | 独立 Cube 和原尺寸桌面 | 原 Task27、复制完整工具、休眠诊断 Cube 与 FR3 | 原 Task27、相同复制工具与诊断 Cube；为避免 FR3 运动，在临时场景停用两臂 |
| Cube/桌面 | 0.8 kg、0.12 m Cube、原尺寸桌面和原摩擦 | 相同名义质量、几何、摩擦 | 相同；运行时 `0.800000012 kg`、重力 Z `−9.81000042 m/s²` |
| 工具 | 无 | 七形状代理，初始静止 | 原七形状代理，两组均不施力、无位移 |
| PhysX 求解器 | TGS，stabilization、CCD 开启 | TGS，stabilization 开启 | 两组 TGS、stabilization 开启，原场景其余设置保持原样 |
| 接触视图 | 1 个：Cube/Table | 2 个：Cube/Tool、Cube/Table | A：Table；B：Tool 后 Table；两组手动准备相同的工具接触报告 API |
| 创建/初始化 | 视图在播放前创建；暖机后初始化 | 两视图播放前创建；暖机后按 Tool、Table 初始化 | 与 C2R 对齐：播放前创建；暖机后初始化，B 依次 Tool、Table |
| 接触报告订阅 | 暖机后注册 | 暖机前注册 | 两组均暖机前注册，观测窗口内逐事件写盘 |
| 物理回调 | 暖机后注册步前与步后 | 暖机后注册一个步后 | 暖机后注册步前、早步后及晚步后；B 先 Tool 后 Table 读取，晚回调反序读取 |
| 刚体状态 | PhysX dynamic-control 刚体姿态/速度、休眠状态 | 相同来源，并读取工具与诊断 Cube | 两组同样读取 Cube、工具位置速度及 Cube 休眠状态 |
| 暖机、唤醒 | 90 次应用更新，记录前唤醒一次 | 90 次更新，记录前唤醒 Cube/工具；诊断 Cube 单次赋速 | 两组均 90 次更新、**53 个物理步**；同样仅在窗口前唤醒/赋速，没有测量中重复唤醒 |
| 观测时间 | 240 步，逐步 dt | 预检 13 步，逐步 dt | 两组各 60 步、步前/步后 `dt=0.0166666675 s` |

`RigidContactView` 安装源码显示，构造时默认可准备接触报告，`initialize()` 未传 view 时分别创建 tensor simulation view；本次明确传 `disable_stablization=False`，没有通过视图关闭稳定化。D1 与 D2 的许多场景因素同时不同，因此 D1 曲线只是参照，不是单变量对照。

## A/B 物理证据

一个诊断任务先后启动两个独立 Isaac 进程、分别建立全新场景；没有失败重试。每组记录 60 步，各进程退出码 0。两组初始 Cube 位置、速度、姿态和工具位置的记录差异为 **0**；预热物理步数均为 53。第一观测帧也匹配，但这并不能将先前 D1 或 C2R 的初始条件视作完全匹配。

| 指标 | A：Table 单视图 | B：Table + Tool 双视图 |
| --- | ---: | ---: |
| 清醒 / 休眠步 | 24 / 36 | 24 / 36 |
| 清醒期首步 → 第 24 步法向力 | 7.838451 → **1.185916 N** | 7.838451 → **1.185916 N** |
| 清醒期平均法向力 | **2.271468 N** | **2.271468 N** |
| 清醒期 Z 动量残差累计 | **2.230588 N·s** | **2.230588 N·s** |
| 支撑矩阵与同一步接触事件冲量最大向量差 | `9.32e-9 N·s` | `9.32e-9 N·s` |
| 早/晚步后回调支撑力最大差 | 0 N | 0 N |
| 工具最大位移 | 0 m | 0 m |

两组所有 60 个对应步的支撑力数值相同；本次早/晚回调分别以优先级 0 和 900 注册，B 的视图读取顺序在两个回调之间反转，仍未测出差异。这只覆盖本次检查的注册优先级和读取顺序，不能排除其他异步执行或内部 PhysX 时间语义。所有记录的前回调、接触报告、早回调和晚回调时间顺序符合预期。

每组清醒期记录 24 条 Cube/Table 接触报告，以及四枚 Cup/Cube 各 24 条候选报告；杯面几何间隙约 **2.998–3.000 mm**，这些候选的报告法向冲量均为 0，B 的 Tool 视图力亦为 0。它们不证明不存在其他未报告的求解器贡献，且本次没有单独测量 Tool/Cube 切向摩擦。Cube 最大实测速度约 `0.000242 m/s`；仅按已记录接触冲量、桌面摩擦、重力与步前/步后速度计算的清醒期 Z 动量残差没有闭合。

对照旧 C2R 的前 13 个数据点，本次 A 曲线与旧支撑力逐点最大差约 **0.00783 N**；旧运行不属于本次严格匹配实验，因此这里只表明数值形态相近。D1 单视图清醒期均值 **7.847575 N**，与重量约 7.848 N 一致；其场景没有工具、诊断 Cube、原 ROS 图和其他 Task27 场景对象，故不能据此单独证明哪一项导致 D2 衰减。

## 可复查证据与限制

- [A/B 比较数据](../../offline/task28_stage_b_contact/evidence/task28_d2_view_compare_v1/comparison.json)、[对照曲线](../../offline/task28_stage_b_contact/evidence/task28_d2_view_compare_v1/comparison.png)、[进程与重置记录](../../offline/task28_stage_b_contact/evidence/task28_d2_view_compare_v1/manifest.json)。
- [A 逐步数据](../../offline/task28_stage_b_contact/evidence/task28_d2_view_compare_v1/A/timeseries.jsonl)、[A 接触事件](../../offline/task28_stage_b_contact/evidence/task28_d2_view_compare_v1/A/contact_events.jsonl)、[B 逐步数据](../../offline/task28_stage_b_contact/evidence/task28_d2_view_compare_v1/B/timeseries.jsonl)、[B 接触事件](../../offline/task28_stage_b_contact/evidence/task28_d2_view_compare_v1/B/contact_events.jsonl)。每步记录真实 dt、前后速度、休眠、原始冲量/力、事件对、摩擦、回调时间与动量残差。
- 原 Task27 ROS 图仍尝试访问已停用的两臂，导致 A/B 日志分别出现 **822/798 条错误**，主要是找不到 FR3 articulation；这些错误和原始日志均已保留。两组 Cube/工具的刚体及所需接触视图句柄有效、实验完成且状态匹配，但这些日志意味着本次不能视作与完整 C2R 环境完全等价，也不能排除其间接影响。没有为了消除错误而追加物理重试。
- 旧 C2R 的失败证据与现有预检保持不变。物理报告和矩阵是同一 PhysX 后端的两条读取路径，并非独立力校准；睡眠期零读数不能当作接触力实际为零。

## 诊断判断与下一步

本次对照排除了“**必须创建两个视图才会发生该衰减**”这一特定假设，也没有观察到本次两个步后回调或其读取顺序造成的数值差异。零法向冲量的杯面候选、诊断 Cube、原场景/ROS 图、停用 FR3 产生的图错误及 PhysX 内部稳定化仍是并存差异；没有单独改变其中任何一项，不能把它们直接定为根因，也不能声称 PhysX 本身错误。

建议下一步先在可隔离的复制场景中做**单一因素**对照：保留原 Cube/桌面和求解器参数，比较完整静止工具存在与否，同时避免无效 FR3 ROS 图；记录活跃/休眠状态、所有成对法向及摩擦，再决定下一项。此为提案，不是本轮结果。

在当前接口未得到连续定量验证前，可用 `m(a−g)` 减去已知外力估计 Cube **总**接触合力，并对加速度噪声和休眠单独处理；它不能区分桌面与工具的各自反力，也无法排除未知外力或稳定化贡献，因此不能直接替代成对接触力闭环。不得因此放宽现有安全门。

物理任务前离线回归 **25 passed**；加入实测 A/B 回归后 **26 passed**。重新检查命令：

```bash
PYTHONPATH=. python3 -m pytest -q offline/task28_stage_b_contact/test_d2_analysis.py offline/task28_stage_b_contact/test_d1_math.py offline/task28_stage_b_contact/test_c2r_balance.py offline/task28_stage_b_contact/test_c2_evidence.py offline/task28_stage_b_contact/test_c2_preflight.py
python3 offline/task28_stage_b_contact/plot_d2.py offline/task28_stage_b_contact/evidence/task28_d2_view_compare_v1
```
