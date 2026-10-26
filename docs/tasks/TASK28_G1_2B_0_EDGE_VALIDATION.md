# Task28 G1-2B-0｜离线双臂闭链轨迹边验证

日期：2026-10-26（Asia/Seoul）

仓库：`dreamerToTOE/dual-arm-embodied-palletizing`；分支：`side-suction-palletizing`。
起始提交：`bd519c47d226c1fb66bd6e36ed273194dd4ebb8b`。

## 1. 判定与核心结论

**`G1_2B_0_PASS`：表示本报告定义的离线有限采样接口和正负控制通过。**

两个有效端点之间，确实可以存在闭链错误、载荷碰撞、限位超越或非法接触模式变化。
本任务构造并实际检查了四类这样的候选轨迹，先确认各自两个端点通过完整门禁，再
记录中间失败。因此 endpoint-valid 不能推出 sampled-edge-valid，更不能推出连续安全。

原有 **296** 项测试全部保留通过，新增 **40** 项通过，共 **336** 项。七个原 A-stage
分段中，六段为 `SAMPLED_EDGE_VALID`；纯垂直放置段采样几何有效，但目标支撑时序
确认未完成，故为 `EDGE_INCONCLUSIVE`。接上原序列已有的显式静止支撑观测后，组合
placement+hold 边通过。没有修改接受标准、增加假传感器帧或让插值点贡献三帧确认。

所有轨迹的 `continuous_safety` 固定为 `NOT_ESTABLISHED_FINITE_SAMPLING_ONLY`。
没有实现 G1-2B-1、CCD/swept verifier、P4-NR/规划器、QP、力控制或 RL；没有启动
Isaac、ROS、机器人/吸附命令或物理实验，也没有声称仿真或物理执行成功。

## 2. 冻结边界与实现位置

使用 `fixtures/task28_g1_0_new_single_cube_v0`，唯一 `Cube_01`、index 0。fixture
manifest SHA256 仍为
`f809a4a57ec8b2a3719f6bcd9c53a467b8ed2f5353928f62d8adfb02d91f3286`。
fixture、模型、保守几何、SRDF/ACM、吸附阈值、G1-1/G1-2A-R1 实现与测试、
Task24/26/27 均未修改。新实现独立位于
[offline/task28_g1_2b_0](../../offline/task28_g1_2b_0/README.md)。

复用原 `ContactModeValidator`、`SupportPolicy`、`ClearanceGeometry`、真实 FCL 和
`TemporalClearanceConfirmer`。独立 FK 使用 G1-1 已审查的
`tests/reference_fk.py`：按冻结 USD joint local0/local1 与 X-axis 公式计算，生产
模型则按 URDF joint origin/Z-axis FK。新 `oracle_binding.json` 校验该源文件 hash；
只缓存已核验 fixture JSON，未修改其数学公式或旧实现。

每个结构有效的 sample 检查双臂 link0–8 与两个 TCP，共 20 个独立 FK 比较；误差
门限保留 `1e-6 m/rad`。TCP 输入由该独立数学 oracle 重建并注明来源，不声称是独立
测得的传感器值。闭链门限仍为 `1e-4 m / 1e-3 rad`。载荷、双臂、工具和桌面全部
保留在 FCL 中，每个完整查询检查 414 个 shape pair、按原 ACM 跳过 40 个 shape pair。

## 3. Typed 轨迹与重建合同

`types.py` 定义 frozen dataclass：

| 类型 | 内容 |
|---|---|
| `Waypoint` | synthetic time、左右 7 关节、共享 Cube xyz/xyzw、mode/site、左右 attached flags、独立 intent/feedback、provenance、事件，以及可选 joint/object 切向量 |
| `Provenance` | `EXPLICIT_SYNTHETIC` 或 `MODEL_INTERPOLATION`，Cube/attachment evidence ID 与来源说明 |
| `Trajectory` | 有序 waypoint、motion kind、插值规格、scope、显式 R1 prelude 与要求的终态阶段 |
| `SamplingPolicy` | 时间/关节/Cube 平移/旋转分辨率与总 sample 预算 |
| `EdgeResult` | 端点结果、全部采样数据、错误位置、残差极值、实际采样数、时序结果与计算时间 |

变换为列向量主动 `T_A_B`，世界 `world`、米、弧度、合成秒；quaternion 为 xyzw。
共享 Cube 位姿是轨迹中独立给定的 SE(3)，不是从命令 TCP 推断，也不是从某一个手臂
的 implied object pose 强行代替。真实传感器/ROS adapter 未实现；本接口当前输入
明确限定为离线合成 waypoint，不能把 provenance 声明当成实测证据。

### 3.1 默认线性插值

每个 span 的归一化参数 `u=(t−t0)/(t1−t0)∈[0,1]`：

```text
q_L(u) = (1−u) q_L0 + u q_L1
q_R(u) = (1−u) q_R0 + u q_R1
p_O(u) = (1−u) p_O0 + u p_O1
R_O(u) = R_O0 Exp(u Log(R_O0ᵀ R_O1))
```

旋转沿最短 SO(3) geodesic，不做 quaternion 元素线性插值；相反符号 quaternion
表示相同旋转，π 旋转控制也有独立测试。**左右关节独立插值不保证闭链**，因此每个
sample 均重新检查完整平移/旋转 closure，包括两侧 implied object 与共享 Cube。

### 3.2 显式非线性控制

可选择 cubic Hermite，端点切向量必须显式给出，不能静默补零或运行 IK 修复：

```text
x(u) = h00 x0 + h10 Δt v0 + h01 x1 + h11 Δt v1
h00=2u³−3u²+1, h10=u³−2u²+u,
h01=−2u³+3u², h11=u³−u²
```

joint tangent 单位 rad/s、translation tangent 单位 m/s。每个 sample 原样检查；
checker 不解 IK、不投影到闭链流形、不移动抓点、不裁剪关节、也不缩小 Cube。
这是位置轨迹验证；没有认证速度、加速度、动力学或实际时间执行可行性。

## 4. 采样和结果分类

默认每个 span 根据下列要求确定均匀子区间数：

- `max_dt_s=.05`；
- 单关节增量不超过 `.02 rad`；
- Cube 平移增量不超过 `.001 m`；
- object geodesic 旋转增量不超过 `.01 rad`；
- 全轨迹最多 `10001` 个 sample。

线性/Hermite 多项式对归一化 u 的导数界由端点及二次导数多项式的内部极值计算；
对 joint 取逐关节最大值，对平移取分量界的欧氏范数。旋转角使用 SO(3) geodesic
长度。按各界与分辨率之比向上取整，再取不小于 2 的偶数，保证每个 span 包含精确
中点及所有原 waypoint。浮点向上取整可能产生更密的网格；每段实际 subdivisions
与实际最大 sample 增量均记录。该分辨率是可配置研究输入，不是已证明充分的碰撞
安全精度。达到预算而未完成覆盖时不能返回有效边。

| 状态 | 含义 |
|---|---|
| `ENDPOINT_VALID` | 仅请求端点检查且两端全部门禁通过；没有中间覆盖声明 |
| `SAMPLED_EDGE_VALID` | 所有规定 sample、离散 mode/event 检查及请求的时序终态均通过 |
| `SAMPLED_EDGE_REJECTED` | 在已评估位置发现确定的约束违反，记录全部原因和位置 |
| `EDGE_INCONCLUSIVE` | 预算/合同/后端证据不足，或请求的时序终态尚未确认；不能当作通过 |

FCL 后端故障本身只说明证据不足，不被伪称为真实碰撞；若同时存在确定的闭链/限位/
碰撞错误，则仍可以拒绝该候选。非法格式、超大时间、缺失 Hermite 切向量与非法
prelude 事件均有 fail-closed 控制。对有限但超限的数学关节配置，保留 FK/closure/FCL
诊断，同时明确拒绝限位，不产生执行许可。

## 5. 接触模式与 R1 时序集成

运输 sample 不移除 Cube，也不授予 table 豁免。抬升/放置中，只有 Cube **实际几何**
通过冻结 site 的原微米级 `SupportPolicy` 时，sample 才使用 `BILATERAL_SUPPORTED`；
几何分离时使用 `BILATERAL_TRANSITIONING`，继续闭链、限位和完整 FCL，且无支撑接触
例外。支撑/吸盘局部规则均复用原实现，不写入 ACM。

mode/attached flags/intent 变化需明确事件。`ATTACH`/`RELEASE` 保留旧 G1-2A 新 request、
因果顺序与物体/关节静止守卫；BEGIN_LIFT/BEGIN_PLACE 必须处于正确 motion 的边界。
中间无事件地 release/reattach、错误 motion 中的事件或事件重复均拒绝。

`enforce_temporal_confirmation=True` 时必须显式提供 `required_terminal_phase`。
每段回放保存的 R1 prelude 和本段**显式 waypoint 观测**，不能直接相信调用者声称的
已确认阶段。模型插值点的 ID/时间只用于离线重建合同，其 provenance 明确为
`MODEL_INTERPOLATION`；**永远不送入 R1，不贡献三帧确认**。改变采样密度不会改变
显式 observation 数量或确认结果。

末端必须达到请求阶段且 `phase_confirmed=true`；否则即使几何 sample 全部有效，
也为 `EDGE_INCONCLUSIVE / TERMINAL_PHASE_NOT_CONFIRMED`。缺少 source 确认的 attach
会被 R1 实际拒绝。纯数值负例可不请求完整 A-stage 时序，但结果明确记录该范围。
所有 support 输出仍是几何候选，没有力传感器或实测 D6 attachment 证据。

## 6. 四类端点通过、中间失败控制

四个控制都先经过 `validate(..., endpoint_only=True)`，两端返回 `ENDPOINT_VALID`。
随后才评估规定的中间状态。所有控制使用真实冻结模型与 FCL，无需把抽象几何后端
样例伪装成可达机器人轨迹。

| 控制 | 规定候选与实际结果 |
|---|---|
| A：闭链 | supply/target 各 4 mm 间隙的有效 q 与 Cube pose，独立线性插值。中间最大 closure 约 `0.027891 m / 0.130452 rad`；拒绝。还可能同时产生工具/载荷碰撞，全部保留，不声称是只有闭链一项错误 |
| B：载荷碰撞 | 首末同为有效 supply Cube z=.264 m；显式中间 Cube z=.240 m，通过 bounded IK。该中间点闭链 `4.534e-10 m / 2.666e-9 rad`、最小关节余量 `.347964 rad`，FCL 仅检出 Cube/table 穿透 `.020 m`；拒绝 |
| C：关节限位 | 首末 q 完全相同且有效。对一个关节设 `v0=4(upper−q0+.01)/Δt`、`v1=−v0`，Hermite 中点 `q=upper+.01`，最小限位余量 `−.01 rad`；拒绝。该点也可能破坏闭链，诊断保留 |
| D：模式 | q/Cube 保持 target 支撑姿态不动，显式中间点由 bilateral 切到 released 再切回，缺少 attach/release 事件。所有单点几何门禁通过，mode schedule 拒绝 |

B 是真实冻结运动学的有效闭链**点**却在碰撞中的控制；两侧 span 的普通线性插值
还可能破坏闭链，不能据此宣称存在可执行下沉路径。C 是显式非法的数学曲线，从未
声称超限姿态物理可执行。有效区间端点之间的普通线性关节插值不可能越过 box joint
limits；因此 C 使用明确的非线性 overshoot，而没有伪造“线性中点超限”。

## 7. 冻结 A-stage 分段调查

[完整 waypoint 输入](../../offline/task28_g1_2b_0/examples/a_stage_waypoints.json) 保存
248 个预先规定单状态：3 grasp、3 attach、71 lift、94 transport、71 placement、
3 support-confirm、3 release。物体轨迹明确为 frozen supply `[.65,0,.26]`、垂直抬升
到 `.4 m`、运输至 `[.82,−.075,.4]`、垂直下降到 target `[.82,−.075,.26]`，没有规划搜索。

独立 builder 在每个 object pose 求 bounded 单状态 IK，使用前一个已保存 q 作显式
seed，每臂最多 1 attempt、300 nfev，规定物体点距不超过 2 mm。checker 完全不调用
该 builder 或 IK。原始构造耗时 `763.8765 s`；保存 q 的核验方式约 `10.7 s`。
这些是本机离线计算测量，不是 ROS 时序或机器人执行时间。

248 个 waypoint 全部通过旧单状态门禁、独立 USD/URDF FK 和 R1 序列；最大 waypoint
closure `1.10665e-7 m / 1.09027e-6 rad`，最小限位余量 `.252759806 rad`。没有发现
冻结模型下的单状态构造阻塞，但这不保证 waypoint 之间的边。下表是进一步实际边检查：

| 分段 | sample 数 | 最大 closure 平移 (m) / 旋转 (rad) | 最小关节余量 (rad) | 结果 |
|---|---:|---|---:|---|
| grasp boundary | 5 | detached，不要求双侧 closure | .368345 | SAMPLED_EDGE_VALID |
| attachment confirmation | 7 | `4.15e-11 / 4.43e-10` | .368345 | SAMPLED_EDGE_VALID |
| vertical lift | 277 | `4.13472e-6 / 5.59284e-6` | .368345 | SAMPLED_EDGE_VALID |
| bilateral transport | 189 | `4.45999e-6 / 8.02022e-6` | .498401 | SAMPLED_EDGE_VALID |
| vertical placement | 277 | `1.49919e-5 / 2.12778e-5` | .252760 | EDGE_INCONCLUSIVE：末端严格支撑仅 1 帧 |
| support confirmation | 7 | `1.55359e-10 / 1.58386e-9` | .252760 | SAMPLED_EDGE_VALID |
| released boundary | 7 | `1.32998e-10 / 1.25725e-9` | .252760 | SAMPLED_EDGE_VALID |
| placement + 原显式 support hold | 283 | `1.49919e-5 / 2.12778e-5` | .252760 | SAMPLED_EDGE_VALID，target 支撑已确认 |

每段包含前一段末状态以检查跨段边界，并独立回放其 source prelude。放置段几何/
FK/闭链/FCL samples 全部有效；证据不足仅在于 R1 三帧目标支撑确认尚未完成。后续
原本就存在的静止观测补足该确认，然后 release 才获准。没有删除该中间状态、改阈值
或把密集插值点当作观察帧。完整规定序列可以逐段得到上述离散结果，但仍无连续执行证明。

## 8. 复现、证据与测试

在仓库根目录、Python 3.10.12 / NumPy 1.24.4 / SciPy 1.8.0 / FCL 0.7.0：

```bash
PYTHONPATH=offline/task28_g1_1:offline/task28_g1_2a:offline/task28_g1_2b_0 \
  python3 -u offline/task28_g1_2b_0/run_validation.py
# G1_2B_0_PASS: 336 tests (296 accepted baseline + 40 edge tests)
```

普通复现使用版本化 joint vectors；若确需重新构造，执行独立 builder（仅离线单状态
IK，可能耗时约 13 分钟）：

```bash
python3 offline/task28_g1_2b_0/examples/build_a_stage.py
# 或保留原始 q/solve receipts，只重跑门禁：
python3 offline/task28_g1_2b_0/examples/build_a_stage.py --revalidate-waypoints
python3 offline/task28_g1_2b_0/examples/build_collision_control.py
```

总览为 [evidence/summary.json](../../offline/task28_g1_2b_0/evidence/summary.json)，记录测试
计数、环境、fixture/source hashes、每段实际网格增量、残差/限位极值、失败位置、碰撞
原因和计算时间。每条 `evidence/controls/*.jsonl`、`evidence/a_stage/*.jsonl` 文件首行
为完整 typed trajectory（含 waypoint、切向量、mode、来源和 prelude），后续逐行保存
全部 sample 和显式时序观测，末行为 EdgeResult。每个 sample 包括完整双臂 q、Cube
SE(3)、TCP 重建、独立 FK 残差、闭链、限位、完整 FCL contact pairs 与 support 结果。
无早停隐藏其它失败；预算不足时明确记录没有完成的 planned/evaluated 数量。

全部随机性来源明确：没有随机路径搜索；builder 使用保存的前一个 q 作为固定 seed，
独立数学控制使用规定多项式/位姿。计算时间受主机负载、缓存和文件系统影响，没有
测量真实 ROS 性能、仿真帧率或设备响应。

## 9. 局限与后续扩展接口

有限网格可能漏掉两个 sample 之间短暂的 self/arm-arm/tool/payload/environment
碰撞或闭链误差。给出的导数界只用于配置采样增量，不是碰撞间距下界，也没有认证
所有连续状态。原始模型、位姿观测、吸附、桌面、摩擦与载荷仍有此前记录的仿真/
物理不确定性，fixture 标签保持 `NEW_BASELINE_NOT_YET_PHYSICALLY_VALIDATED`。

`StrongerVerifier` 仅预留接受 typed trajectory 与 sampled evidence 的协议，未来
G1-2B-1 可在此增加真正的 swept/CCD 或保守有界证明。当前没有此实现或 certificate，
也不会把 sampled-edge-valid 升格为 continuously-safe。

停止于 G1-2B-0，等待进一步审查。没有自动进入 G1-2B-1 或 P4-NR。
