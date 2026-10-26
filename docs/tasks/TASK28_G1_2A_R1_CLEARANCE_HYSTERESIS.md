# Task28 G1-2A-R1｜几何间隙滞回与时序阶段确认

日期：2026-10-26（Asia/Seoul）

仓库：`dreamerToTOE/dual-arm-embodied-palletizing`；分支：`side-suction-palletizing`。
审查起点：`7c7116a09315e2bebdad7bc2d1a12b2de0ca30af`。

## 1. 判定与实现边界

**`G1_2A_R1_PASS`：仅适用于离线单帧几何、离散观测序列和操作阶段确认。**

原 G1-2A 的 188 项测试全部保留并通过；新增 R1 测试 108 项通过，总计 **296 项**。
代表性 26 帧序列、12 项机器记录的故意错误、15 组阈值/噪声敏感性序列和 14 项
Cube-only 支撑观测噪声控制均完成。输入、源文件 hash、数值残差与拒绝原因均有机器证据。

冻结 fixture `task28_g1_0_new_single_cube_v0`、G1-1 实现、原有 G1-2A 测试、SRDF/ACM、
吸附阈值、保守碰撞几何、Task24/26/27 均未修改。R1 仅在 Task28 offline 模块中增量
增加过渡模式、时序确认器、几何及测试；原 G1-2A evidence 保留原样。没有启动 Isaac、
ROS 或机器人/吸附命令，没有实现 NR、QP、RL、规划器、轨迹边或扫掠验证；没有开始 G1-2B。

本 fixture 仍为 `NEW_BASELINE_NOT_YET_PHYSICALLY_VALIDATED`。桌面没有力传感器，
所有 `physical_support_observed` 输出均为 **false**。本文的“严格支撑候选”只表示
几何与局部接触合同通过，不能证明接触力、负载转移、摩擦、D6 attachment 或真实稳定性。

## 2. 几何接触与操作确认分层

R1 使用两层状态，避免把毫米阈值当成物理接触条件。

第一层是单帧 `OperationMode` 与 `GeometricContact`：

| 单帧 mode | 接触合同 |
|---|---|
| 原 `DETACHED_SUPPORTED` | supply 上严格几何支撑候选，左右未附着；不要求双臂闭链 |
| 原 `BILATERAL_SUPPORTED` | supply/target 上严格几何支撑候选；双臂闭链、关节限位、完整碰撞检查 |
| 新 `BILATERAL_TRANSITIONING` | 双侧附着观测一致、完整 SE(3) 闭链、Cube 几何间隙 `>1e-6 m`；无 Cube/table 支撑接触例外 |
| 原 `BILATERAL_AIRBORNE` | 保留原 G1-2A/G1-1 airborne 单帧合同；它本身不声明三帧时序确认 |
| 原 `RELEASED_SUPPORTED` | target 严格支撑候选、左右释放观测一致；不要求双臂闭链 |

正间隙超出原数值接触带时，`GeometricContact=SEPARATED`。因此 0.5、1、2 mm
都明确是**几何分离**，即使操作阶段仍为 `LIFTING`，也不会变成“物理支撑”。原先
`1e-6 m` 数值带仍只产生 `SUPPORT_CANDIDATE`，不冒充实测接触或力支撑。

第二层 `ClearancePhase` 记录控制器的时序确认阶段，不能改变第一层的几何结果。
例如 `NEAR_TABLE_CONFIRMED + SEPARATED` 是合法组合，但释放授权为 false。
`BILATERAL_AIRBORNE` 单帧输入也不能直接设置 `AIRBORNE_CONFIRMED`。

## 3. 实际 Cube 几何间隙与有限桌面

只使用输入 `InputState.cube` 的观测位姿 `T_W_O=[R,t]`，不使用命令 TCP、期望末端
位置或 lift 指令值推导间隙。合成测试中的 Cube 位姿被明确标注为合成输入；生产 API
的 external 来源也只是一项 provenance 声明，不是已经接入/验证传感器的证据。

冻结 Cube 半边长 `h=[.06,.06,.06] m`。完整有向包围盒的八个顶点为：

```text
v_s = t + R diag(h) s,  s ∈ {−1,+1}³
z_table = .200 m
c_full = min_s(v_s.z) − z_table
       = t_z − Σ_i h_i |R_zi| − z_table
```

该公式处理 roll、pitch、yaw 对最低顶点的影响，而不是只用中心高度减固定半高。
冻结桌面 xy bounds 是 `x∈[−.05,1.15] m`、`y∈[−.4,.4] m`。有限桌面的诊断间隙为：

```text
B_xy = 冻结桌面矩形
c = min{p_z − z_table | p ∈ Cube_OBB, p_xy ∈ B_xy}
```

完整投影在桌内时，`c=c_full`。部分悬边时，使用 OBB 六个半空间与桌面 xy 四个
半空间的交集；枚举 120 个平面三元组求交集顶点，再求最小 z。部分悬边只返回诊断
间隙，**拒绝时序确认**；完全不重叠时返回 `None` 并拒绝，避免把无限桌面当成真实桌面。
没有虚构 table 厚度或替代模型。`1e-12 m` 只处理浮点可行性/阈值相等边界，独立于
微米接触容差、毫米研究参数及观测噪声。

独立数学控制包括解析支持函数、预定义多轴旋转、边界/无交叠和不同实现的
`scipy.linprog` oracle。部分悬边例子：pitch=−π/4、center
`[1.15+.06√2−.01,0,.4]`，完整最低间隙约 `.115147 m`，有限 footprint 内的诊断
间隙 `.190000 m`；两值不同，且此状态被拒绝。倾斜导致最低角点穿桌面的负例也拒绝。

严格支撑仍调用原 `SupportPolicy`：底部四点、底面法向、供料/目标 footprint、桌面
bounds、contact 位置/法向/深度及 `1e-6 m` 门限全部保留。0.5 mm 和 2 mm 都不满足
该严格支撑合同。转移和空中状态继续执行原 FCL、414 个 shape pair 查询及原 40 个
ACM shape skip；没有新增 ACM pair，也没有扩大局部 suction-contact 例外。

## 4. 明确事件图与三帧守卫

`ConfirmationEvent` 只表示离线输入中的声明，不输出任何硬件/吸附命令。

| 当前阶段 | 事件/合格观测 | 后继阶段 |
|---|---|---|
| `INITIALIZING` | 三帧 supply 严格支撑、detached | `DETACHED_SUPPORTED_CONFIRMED` |
| 已确认 detached supply | 显式 `ATTACH`，新 request 与独立双侧反馈，旧 G1-2A 因果/静止守卫通过 | `ATTACHING` |
| `ATTACHING` | 三帧严格 supply 支撑与双侧反馈 | `BILATERAL_SUPPORTED_CONFIRMED` |
| 已确认 bilateral supply | 显式 `BEGIN_LIFT` | `LIFTING` |
| `LIFTING` | 连续三帧 `c>=d_high`，每帧闭链/限位/FCL 有效 | `AIRBORNE_CONFIRMED` |
| 已确认 airborne | 显式 `BEGIN_PLACE` | `PLACING` |
| `PLACING` | 连续三帧 `c<=d_low` | `NEAR_TABLE_CONFIRMED`，仍不授予支撑或释放 |
| `PLACING` / near-table | 连续三帧 target 严格支撑 | `BILATERAL_SUPPORTED_CONFIRMED`，target 释放前置条件成立 |
| 已确认 bilateral target | 显式 `RELEASE`，新 request、因果/静止守卫与独立释放反馈 | `RELEASING` |
| `RELEASING` | 三帧 target 严格支撑且左右释放反馈一致 | `RELEASED_SUPPORTED_CONFIRMED` |

`ATTACH`/`RELEASE` 的新 request、意图时间、独立反馈及物体/关节静止检查复用原
G1-2A 转换门禁，并使用 source **实际验证时钟**。三帧中 request 必须保持；不能
通过普通 OBSERVE 改变 attachment 意图。其余 mode/event 跳转均拒绝。

释放要求**处理本帧之前**已经确认 target bilateral 支撑。释放帧不能充当目标支撑
的第三帧，也不能在 `NEAR_TABLE_CONFIRMED` 或任何几何分离状态下释放。R1 实现的
`can_release` 只是本次已验证观测时刻的离线授权结果，不是物理安全控制器。

## 5. 滞回、计数与观测有效性

默认研究参数为 `d_low=.001 m`、`d_high=.003 m`，可在新建会话时配置，未做物理
标定；会话内不能换参数。确认帧数固定为 3，不能降低。毫米参数不进入任何支撑、
穿透、吸盘、ACM、关节或闭链容差。

- 未确认抬升：任意 `c<d_high` 帧清零 high 计数；一/两帧达标不能确认 airborne。
- 已确认 airborne：dead band 内保持已确认阶段；低于/等于 low 的一/两帧不会切换。
  连续三帧低间隙但无 `BEGIN_PLACE` 被拒绝，不能隐式放置。
- 已开始放置：三帧 `c<=low` 确认 near-table；随后在 dead band 内保留该阶段。
  反向三帧 `c>=high` 才退回未确认支撑的 `PLACING`。任何 low 确认都不会生成支撑例外。
- target 严格支撑使用单独三帧计数。它可以同时满足 low 条件，但授权只来自严格支撑。

每帧必须满足原 age≤.100 s、skew≤.020 s、future≤.001 s；attachment 反馈也纳入
同步。左右 joints、左右 TCP、Cube、feedback 六个 stamp 必须分别严格递增，caller
`now_s` 也严格递增；仅刷新 feedback、重复 Cube 或某一臂 stamp 不能贡献计数。
Cube evidence ID 与 attachment evidence ID 各自在本会话内全程不能复用；无效尝试中
格式合法的 ID 也被消耗。Cube 来源必须 typed，且不能在一个会话混用 external/synthetic。

**无效观测选择 reset，不 suspend**：过期、异步、重复、乱序、非法几何、失去附着、
碰撞、闭链失败、非法事件等都清零所有连续计数，并撤销阶段确认/释放授权。这避免
把错误前的两帧与错误后的单帧拼成三帧。历史阶段标签可以保留用于诊断，但结果
`phase_confirmed=false`；恢复后需要三帧重新确认。错误发生在已确认 target support
之后也不能沿用旧释放授权。

合法 metadata 的时间 watermark 即使后续几何/事件失败仍前进，要求下一次观测更新；
非法/未来/乱序 stamp 不用于更新 watermark。默认相邻有效 metadata 的任一观测间隔
超过 .100 s，也清零计数并把当前帧作为新 run 的第一帧。这样三张分别很新、却相隔
很久的快照不会拼成“连续三帧”。这些都是合成时钟合同，没有测量 ROS 时序性能。

## 6. 正负控制与量化结果

[26 个冻结 FK/IK 控制](../../offline/task28_g1_2a/examples/r1_configurations.json) 覆盖
supply/target 各 12 个间隙（0、.5、.9、1、1.1、2、2.5、2.9、3、3.1、3.5、4 mm）和
roll=.01/.1 rad 的真实最低间隙 3 mm。全部满足冻结限位、完整双臂 closure、独立
USD-vs-URDF FK 和 FCL。最大 closure 为 `2.9766e-8 m` / `2.4596e-7 rad`；最大独立
FK 差为 `1.9502e-8 m` / `4.8968e-12 rad`。26 个配置均明确是合成输入，不是传感器采样。

代表序列的关键结果：

| 观测 | 几何/时序结果 |
|---|---|
| initial `[.65,0,.26]` 三帧 | strict support candidate；detached support confirmed |
| 显式 attach 三帧 | attachment confirmed；支持模式闭链通过 |
| 抬升 .5/1/2 mm | 每帧 SEPARATED、LIFTING、0 个支撑例外、无释放授权 |
| 3 mm 第 1/2/3 帧 | high count 1/2/3；仅第 3 帧确认 airborne |
| target transport 4 mm | 已确认 airborne 保持，无路径安全含义 |
| 下降 1/.5/.5 mm | low count 1/2/3；确认 near-table，仍为 SEPARATED、不可释放 |
| target `[.82,−.075,.26]` 三帧 | 第 3 帧确认严格目标支撑；才允许显式 release |
| release 三帧 | 第 3 帧确认 released-supported；不声称真实吸附释放成功 |

负例覆盖：穿桌、错误 supply/target footprint、悬边/无桌面投影、倾斜最低角穿透、
过期、异步、重复 ID、乱序或冻结 stamp、非法 quaternion/frame、失去单侧 attachment、
关节越限、完整闭链错误、Cube/tool、Cube/arm、arm-arm、self、robot/table 碰撞、
只有一/两帧达标、无新 request、非法 phase 跳转、未支撑或尚未确认时释放、参数
中途改变及合成/外部来源混用。所有无效序列都不贡献确认计数。

## 7. 阈值与噪声敏感性

敏感性输入是有界的确定性合成几何序列，**不是实测噪声分布、误报概率或实时延迟**。
为保证每帧与关节/FK/闭链相容，阈值网格使用上述各间隙的完整合成配置。

抬升输入 `[(3+a),(3−a)]×3, [3+a]×3, [4]×3 mm`；表中为 bootstrap 后首次
确认的帧序号。尚未满足三帧时一律没有确认。

| d_high (mm) | a=0 mm | a=.1 mm | a=.5 mm |
|---:|---:|---:|---:|
| 2.5 | 3 | 3 | 3 |
| 3.0 | 3 | 9 | 9 |
| 3.5 | 12 | 12 | 9 |

下降接近输入 `[(1−a),(1+a)]×3, [1−a]×3, [.5]×3 mm`；仅确认 near-table，
所有这些帧仍几何分离且不能释放。

| d_low (mm) | a=0 mm | a=.1 mm |
|---:|---:|---:|
| .75 | 12 | 12 |
| 1.00 | 3 | 9 |
| 1.25 | 3 | 3 |

阈值附近交替噪声会延迟确认，连续三个同向偏差仍可确认，所以三帧规则不能替代
传感器误差标定。已确认 airborne 在 2.9/3.1 mm 或 dead band 小波动中保持；已确认
near-table 在 .9/1.1 mm 和 dead band 小波动中保持，不会逐帧来回切换。

额外在固定支持关节配置上只扰动 Cube z 观测：−2、−.5、0、+.5、+2、+100、+500 μm。
supply/target 各 7 项中只有 |noise|≤1 μm 的数值带控制通过 supported-state；其余
全部拒绝且没有支撑接触例外。这说明毫米滞回没有扩大严格几何支撑条件，也说明未来
观测噪声可能导致严格支撑确认假阴性，必须在仿真/物理研究中量化，不能任意放宽门限。

## 8. 复现、证据与回归

在仓库根目录使用当前 Python 3.10.12 / NumPy 1.24.4 / SciPy 1.8.0 / FCL 0.7.0：

```bash
PYTHONPATH=offline/task28_g1_1:offline/task28_g1_2a \
  python3 -u offline/task28_g1_2a/run_r1_validation.py
# G1_2A_R1_PASS: 296 tests (188 baseline + 108 R1); 26 frames; 15 sensitivity sequences
```

原 188 项与 R1 测试分别计数；R1 共 geometry 36、temporal 70、sensitivity 2。
也可一次运行全部：

```bash
PYTHONPATH=offline/task28_g1_1:offline/task28_g1_2a \
  python3 -m pytest offline/task28_g1_1/tests offline/task28_g1_2a/tests -q -p no:cacheprovider
```

机器证据：[r1_validation_results.json](../../offline/task28_g1_2a/evidence/r1_validation_results.json)。
包含 source/fixture hashes、环境、分组测试计数、26 帧完整位姿/ID/时间戳/来源/几何顶点/
计数/阶段/FK/closure/FCL 结果、12 项负例实际拒绝原因、15 组噪声序列和 14 项 Cube-only
支撑噪声结果。配置重生成脚本为
[build_r1_configurations.py](../../offline/task28_g1_2a/examples/build_r1_configurations.py)，
仅运行带边界预算的单状态 IK/FK/FCL，不是规划器；普通复现直接读取已版本化 q。

## 9. 仍需仿真/物理验证的事项与停止点

没有验证真实 Cube pose 的精度/延迟、真实 ROS 同步、独立 attachment feedback、D6
建立/断开、真实桌面/吸盘/载荷变形、吸附力、摩擦和支撑稳定性。几何间隙也不是力或
负载转移观测。默认 1/3 mm、三帧、100 ms 间隔门禁需要后续用真实噪声与采样率评估；
持续偏差、遮挡和错误 provenance 不能由当前有限合成测试排除。

离散阶段确认没有检查两个帧之间的运动、速度、碰撞边、连续时间或扫掠体积。G1-2B
仍需另行审查授权；本任务到 G1-2A-R1 结束，不自动开始后续阶段。
