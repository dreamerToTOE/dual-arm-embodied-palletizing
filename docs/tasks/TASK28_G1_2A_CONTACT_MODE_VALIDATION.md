# Task28 G1-2A｜离线接触模式与支撑转换验证

日期：2026-10-26（Asia/Seoul）
仓库：`dreamerToTOE/dual-arm-embodied-palletizing`
分支：`side-suction-palletizing`
审查起始提交：`4b5806e4250c468865b1f217f5afbcba15e61b2a`

## 1. 判定与边界

**`G1_2A_PASS`：仅表示离线接触模式、单状态碰撞/支撑门禁和代表性端点转换通过。**

本任务使用已批准的 `fixtures/task28_g1_0_new_single_cube_v0`，并把 G1-1 的双臂
SE(3) 闭链和 FCL 查询作为未修改的参考实现。没有修改 fixture、关节限位、保守
碰撞几何、SRDF/ACM、吸附阈值或 Task24/26/27 执行路径。没有启动 Isaac Sim、ROS、
MoveIt 或机器人/吸附命令，没有实现 NR、QP、力控制、RL、G1-2B 或任何路径规划。

本次证明的是离线合同和静态端点门禁，不证明 D6 attachment、吸附力、摩擦、承载
稳定性、传感器质量、ROS 实时性或物理执行成功。历史 Task24-H 的结果也没有转移到
这个新 fixture。

实现位于 [offline/task28_g1_2a](../../offline/task28_g1_2a/README.md)，原始机器证据
见 [validation_results.json](../../offline/task28_g1_2a/evidence/validation_results.json)，
独立 FCL 支撑诊断见
[independent_fcl_support_audit.json](../../offline/task28_g1_2a/evidence/independent_fcl_support_audit.json)。

## 2. 冻结输入和数据约定

唯一对象是 `Cube_01`、index `0`，尺寸为 `0.120 m` 立方体、质量 `0.8 kg`；桌面
实体为 `task28_table`。初始中心为 `[0.650, 0.000, 0.260] m`，目标中心为
`[0.820, -0.075, 0.260] m`，桌面顶面为 `z=0.200 m`。输入仍遵守 G1-1 的
world、米制、Z-up、四元数 xyzw、列向量变换和 `Cube_01` frame/ID 合同。

G1-1 的状态门禁仍检查左右完整关节观测、左右 TCP、Cube 观测、限位、FK 和时间：

| 条件 | 冻结离线合同 |
|---|---|
| observation 最大年龄 | `0.100 s` |
| 五类输入最大总 skew | `0.020 s` |
| future lead | `0.001 s` |
| 四元数 | 有限 xyzw，长度误差不超过 `1e-6` |
| 关节限位 | 读取冻结模型，越界拒绝 |

G1-2A 另外使用 typed `AttachmentIntent` 表示计划/命令意图，以及 typed
`ObservedAttachment` 表示独立反馈。`source=INDEPENDENT_SYNTHETIC` 只说明测试输入
是确定性的合成观测；`COMMAND_ECHO` 永远不能当作反馈。反馈必须与 request ID、意图
时间和当前 mode 一致，且不能过期、未来或与左右观测异步。

## 3. 四个接触模式

`OperationMode` 是严格枚举，状态中的 attachment flags、`AttachmentMode`、意图和
独立观测必须与枚举一致。

| 模式 | support site | attachment | 必要条件 |
|---|---|---|---|
| `DETACHED_SUPPORTED` | `SUPPLY` | 左右均 detached | Cube 八顶点不穿桌面，底部四点在 supply footprint/桌面，支撑记录若存在须通过局部门禁；不要求双臂闭链 |
| `BILATERAL_SUPPORTED` | `SUPPLY` 或 `TARGET` | 左右均 bilateral | 上述支撑几何、独立双侧反馈、完整 SE(3) closure、FK/限位和全部非支撑碰撞通过 |
| `BILATERAL_AIRBORNE` | 无 | 左右均 bilateral | 完整 closure；Cube 所有顶点到桌面顶面的最小间隙 `>1e-6 m`；不接受 support site 或 Cube/table 支撑重分类 |
| `RELEASED_SUPPORTED` | `TARGET` | 左右均 detached | target 支撑几何、独立释放反馈、FK/限位和全部非支撑碰撞通过；不要求闭链 |

支撑模式不是“看到 mode 就全部放行”。状态仍先运行原始 G1-1 FCL；只允许一条
owner 为 `Cube_01` 与 `task28_table` 的记录进入本地支撑分类。其它返回记录原样
保留并继续拒绝。原 ACM 没有改变，支撑分类也不会写回 ACM。

## 4. 支撑接触的数学合同

物体位姿采用 `T_world_object`。半边长 `h=[0.06,0.06,0.06] m`，由八个局部
顶点 `v_i∈{−h_x,+h_x}×{−h_y,+h_y}×{−h_z,+h_z}` 计算
`p_i = R_world_object v_i + t_world_object`。

对 `z_table=0.200 m`，支撑几何必须满足：

1. 所有八个 `p_i.z >= z_table - 1e-6 m`，否则返回 `SUPPORT_TABLE_PENETRATION`；
2. 四个底部点的 `|p_i.z-z_table| <= 1e-6 m`；
3. 物体底面法向与世界 `-Z` 的夹角不超过 `1e-6 rad`；
4. 底部点全部位于指定 supply `[0.650,0.000]` 或 target `[0.820,-0.075]` 的
   `0.120 m × 0.120 m` footprint，且位于冻结桌面 xy bounds；
5. geometry、法向、footprint、深度和 contact 数均为有限值，接触报告超过或等于
   `128` 条时按可能截断拒绝。

实际 FCL 记录的法向无论 owner 顺序如何都归一为 **Cube → table**，必须接近
`[0,0,-1]`，单位长度误差和角误差都不超过 `1e-6`。每条 contact 的位置必须在
桌面平面、Cube 底面和指定 footprint 内，深度绝对值不超过 `1e-6 m`。这条规则只
重分类当前 `Cube_01/box` 与 `task28_table/box` 记录；没有凭空制造 contact。

FCL 0.7.0 的独立诊断记录了边界行为：精确 initial/target 支撑各返回 4 条深度
`0`、法向 `-Z` 的 contact；下沉 `5e-7 m` 返回约 `5.0e-7 m` 深度并仍在数值带内，
下沉 `2e-6 m` 被拒绝；抬高 `5e-7 m` 无 FCL contact。抬高后几何 near-zero 分类
不等于物理支撑证据。把相同 Cube 平移到错误 patch `[0.9,0.2,0.26]` 仍会得到
桌面法向 contact，但 footprint 检查拒绝；绕 X 轴 `0.001 rad` 造成约 `6.0e-5 m`
穿透并拒绝。实现没有为了让边界通过而扩大容差或缩小几何。

## 5. 分阶段碰撞语义

`PhaseCollisionValidator` 在所有 mode 中先调用未修改的 G1-1 `CollisionValidator`，
因此保留双臂、两个 side-suction 工具、Cube、桌子、self/arm-arm/robot-table 及
相关 shape pair 的检查回执。

- 支撑 mode：仅对上述精确 Cube/table 记录运行第 4 节的局部 `SupportPolicy`。通过
  后从该条记录的原始拒绝原因中移除对应单条 reason，并记录 accepted contact；
  任意其它记录、后端失败、shape 数量错误或不明 owner 均继续拒绝。
- airborne mode：不调用支撑重分类，不接受 support site；对 Cube 八顶点计算
  `min(z)-z_table`，必须严格大于 `1e-6 m`，并保留 G1-1 的全部原始 collision reasons。
- detached/released：不要求闭链，但仍做完整 FK、限位、状态和 collision gate；只在
  其声明 support site 上接受本地支撑接触。

这是一条 phase-local policy，不是永久 ACM 例外。侧吸工具/Cube 的预期接触仍由
G1-1 的局部 suction-contact 合同处理；本任务没有增加或放宽其阈值。

## 6. 转换图和守卫

允许的代表性 A-stage 图为：

```text
DETACHED_SUPPORTED --attach--> BILATERAL_SUPPORTED
       --lift--> BILATERAL_AIRBORNE
       --transport_snapshot--> BILATERAL_AIRBORNE
       --place--> BILATERAL_SUPPORTED
       --release--> RELEASED_SUPPORTED
```

实现拒绝其它 mode/event 组合。每个 endpoint 先完整验证；再检查：

- source/target 时钟、所有五类观测 stamp 和 attachment evidence ID 严格前进；
- attach/release 必须在相应 supply/target site，有新的 request ID，满足
  `old_feedback_stamp < intent.issued_at <= new_feedback_stamp`，并且 Cube 位姿和
  左右关节在事件端点不动；
- lift/place 保持 intent、xy 和旋转，分别要求 z 增大/减小，且 site 分别是 supply/target；
- transport snapshot 仍为 airborne，intent 不变；
- 双侧 mode 的 endpoint 必须通过完整 SE(3) closure；支持和释放 endpoint 必须通过
  第 4、5 节的局部支撑和碰撞条件。

`TransitionResult.scope` 固定为
`MODE_AND_ENDPOINT_GUARDS_ONLY_NO_EDGE_SAFETY`。实现不插值、不检查速度、时间连续
性、边级 FCL 或 swept volume；这些是后续 G1-2B 的范围。

## 7. 正向控制和量化结果

使用冻结 initial/target 位置、冻结模型 FK/IK 状态和确定性合成 observation，序列
包含 6 个 endpoint、5 个合法事件：

| endpoint | mode/site | closure 平移误差 (m) | closure 旋转误差 (rad) | airborne clearance (m) | accepted support records |
|---|---|---:|---:|---:|---:|
| A supply | detached/supply | — | — | — | 1 |
| B supply | bilateral/supply | `8.4958e-09` | `7.8592e-08` | — | 1 |
| C lifted supply | bilateral/none | `1.8669e-09` | `2.0572e-08` | `0.140000` | 0 |
| C target transport | bilateral/none | `8.2109e-09` | `2.4379e-08` | `0.140000` | 0 |
| B target | bilateral/target | `2.9766e-08` | `2.4596e-07` | — | 1 |
| D target | released/target | — | — | — | 1 |

五个转换全部通过 endpoint 和事件守卫。initial `[.650,0,.260]` 与 target
`[.820,-.075,.260]` 都实际进入支撑模式验证；它们不是只用抬高的替代姿态。
这说明当前问题通过显式支撑语义处理，未把 Cube/table 作为全局碰撞豁免。

## 8. 负向控制

`tests/test_contact_support.py`、`test_contact_pipeline.py`、`test_contact_metadata.py`
和 `test_contact_transitions.py` 覆盖下列拒绝原因：

| 控制 | 预期拒绝 |
|---|---|
| Cube 下沉超过 `1e-6 m` | `SUPPORT_TABLE_PENETRATION` / contact depth exceeded |
| 支撑点移到指定 footprint 外 | `SUPPORT_OUTSIDE_DESIGNATED_FOOTPRINT` |
| airborne 与桌面相交或带 support site | `AIRBORNE_CLEARANCE_NOT_ESTABLISHED` / `AIRBORNE_SUPPORT_SITE_PRESENT` |
| mode、flags、intent、feedback 不一致 | `MODE_ATTACHMENT_MISMATCH`、`FEEDBACK_MODE_MISMATCH` 等 |
| command echo 冒充反馈 | `ATTACHMENT_COMMAND_ECHO_NOT_FEEDBACK` |
| stale 或左右/对象时间异步 | `STALE_ATTACHMENT_FEEDBACK` / `ASYNCHRONOUS_ATTACHMENT_FEEDBACK` |
| bilateral 闭链残差超限 | G1-1 closure reason，例如 `LEFT_GRASP_TRANSLATION_MISMATCH` |
| Cube/工具、Cube/臂、臂/臂、自碰、robot/table 碰撞 | 原始 G1-1 collision reason 保留并拒绝 |
| release unsupported、错误 site、无新 request | support/causal/transition reason |
| 非法跳转、重复 evidence、非单调时间或事件运动不符 | `ILLEGAL_MODE_TRANSITION` 及对应 guard |
| FCL/backend 失败或 support policy 被篡改 | `COLLISION_BACKEND_FAILURE` / `SUPPORT_POLICY_CONFIGURATION_CHANGED` |

负例不会通过改变全局 ACM、放宽几何、跳过 closure 或把命令当反馈来“修复”。

## 9. 测试和 provenance

在 Python 3.10.12、NumPy 1.24.4、SciPy 1.8.0、FCL 0.7.0 环境执行：

```bash
PYTHONPATH=offline/task28_g1_1:offline/task28_g1_2a \
  python3 -m pytest offline/task28_g1_2a/tests -q -p no:cacheprovider
# 117 passed

PYTHONPATH=offline/task28_g1_1:offline/task28_g1_2a \
  python3 -m pytest offline/task28_g1_1/tests offline/task28_g1_2a/tests -q -p no:cacheprovider
# 188 passed（G1-1 回归 71，G1-2A 117）

PYTHONPATH=offline/task28_g1_1:offline/task28_g1_2a \
  python3 -u offline/task28_g1_2a/run_validation.py
# G1_2A_PASS: 188 tests; 6 snapshots; 5 transitions
```

`validation_results.json` 保存 fixture manifest SHA256、环境版本、支持容差、每个
endpoint 的 closure/clearance/collision 量化值、转换结果、负例 reason catalog 和
实现源文件 hash。`independent_fcl_support_audit.json` 保存 initial/target、深度边界、
错误 footprint 和倾斜负例的原始 contact 记录。测试产生的 `__pycache__` 已由目录
`.gitignore` 排除，未把构建或运行时状态写入 fixture。

## 10. 未完成事项和 G1-2B 前置条件

本任务仍不能回答：真实 D6 是否建立双侧 attachment、吸盘是否承受 Cube 重力、接触
摩擦/稳定性是否满足、Isaac PhysX 的 support manifold 是否与 FCL 一致、真实 ROS
时间戳是否满足当前阈值、以及从一个 endpoint 到下一个 endpoint 的整条轨迹是否无碰。

进入 G1-2B 前必须在保持本 fixture 和 ACM 不变的前提下，审查并批准：真实/仿真
attachment feedback adapter、support/contact 观测 provenance、连续时间与时钟合同、
轨迹边/扫掠 FCL 预算、载荷状态更新和失败分母。G1-2A 不自动开始这些工作。
