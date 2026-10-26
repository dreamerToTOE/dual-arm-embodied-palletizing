# Task28 G1-1｜离线双臂闭链运动学验证器

日期：2026-10-26（Asia/Seoul）
仓库：`dreamerToTOE/dual-arm-embodied-palletizing`
分支：`side-suction-palletizing`
起始提交：`ce407473d6fd380916328c6a5d882a596480ec3d`

## 1. 判定与范围

**`G1_1_PASS`：仅表示本文件定义的离线、单状态验证范围通过。**

G1-0B 的条件就绪状态已获研究者接受。本次建立独立模块，完成完整 SE(3)
抓取闭合、冻结模型 FK/IK/限位、带合成时间戳的状态门禁、包含载荷的真实 FCL
碰撞接口及正负控制。测试共 **71 项通过**，三个真实双臂模型状态均通过全部门禁。
所有实现、测试输入、量化结果和依赖身份均已保存。

没有启动 Isaac、Kit 或 ROS 节点，没有发关节/吸附命令，没有实现 NR/P4-NR、
QP、力控制、P2、RL 或 G1-2 边/扫掠验证。构建的唯一可执行文件是独立 FCL
单状态查询 helper，输出位于 `/tmp`，不构建机器人工作区。
冻结 fixture 仍为 `NEW_BASELINE_NOT_YET_PHYSICALLY_VALIDATED`。
历史 Task24-H `0.184/0.115 mm` 未作为本次结果，也未用于定义通过标准。

实现目录为 [offline/task28_g1_1](../../offline/task28_g1_1/README.md)。
量化原始结果见 [validation_results.json](../../offline/task28_g1_1/evidence/validation_results.json)。
本次隔离检出为
`/home/ubuntu2004/Documents/Codex/2026-10-25/files-pasted-by-the-user-task/work/g1-1-validator`；
用户原工作目录未切换、清理或覆盖。

## 2. 冻结模型与接口版本

输入固定为 `fixtures/task28_g1_0_new_single_cube_v0`。
`task28_validator/fixture_binding.json` 记录其起始 commit 和 manifest SHA256。
构造模型/碰撞接口时先核对 manifest 及其全部文件 hash，随后读取冻结 URDF、
关节限位交集、保守网格、工具碰撞原语及原 SRDF。本任务未修改任何 fixture 文件，
因此没有 fixture revision；也未修改 Task24/26/27 或其构建/执行路径。

G1-1 新定义的门禁与派生抓取合同在本目录实现和测试中版本控制：

| 项目 | 合同 |
|---|---|
| 对象 | 唯一 `Cube_01`，边长 .120 m；world/metre/Z-up，identity planning-world shift |
| TCP | `left_fr3_side_suction_tcp` / `right_fr3_side_suction_tcp`，读取冻结 URDF 的完整 link8→TCP 变换 |
| nominal grasp | object Y 两面 ±60 mm，再加每侧 1 mm 名义 capture 间隙；Z 偏置 0；旋转见下文 |
| 间隙来源 | 已冻结来源的 Task24 controller `kSideContactCommandGap=.001`；小于原 .003 m capture 门限。这里只作明确的离线派生定义，不声称实测 attachment 标定 |
| closure / FK 观测容差 | translation ≤`1e-4 m`，rotation ≤`1e-3 rad`；可收紧，不可放宽 |
| IK 接受容差 | 重新 FK 后 translation ≤`1e-6 m`，rotation ≤`1e-5 rad`，且满足全部有效硬限位 |
| 时间合同 | 合成同一 clock domain；age ≤.100 s，五类观测总 skew ≤.020 s，future lead ≤.001 s |
| 四元数 | xyzw，有限、长度 4、norm 误差 ≤`1e-6`；非法值不自动修复 |

端到端默认合同使用 1 mm 间隙。零间隙 `gap=0` 是单独的数学/碰撞接触控制；
它不被静默替代为默认端到端抓取合同。未来实际 attachment 标定变化必须明确
记录合同/fixture revision，不能在验证中动态移动抓点来消除残差。

## 3. 完整 SE(3) 约定与闭合残差

采用列向量主动变换：`T_A_B` 将 B 坐标表达映射至 A。
`T_A_C = T_A_B @ T_B_C`，translation 单位 m，旋转单位 rad。
URDF RPY 顺序为 `Rz(yaw) Ry(pitch) Rx(roll)`；输入 quaternion 为 xyzw。
矩阵必须有限、底行 `[0,0,0,1]`、旋转正交且 det=+1；拒绝反射、复数和广播形状。

实现显式区分五类变换：

- `T_W_O`：世界至共享 Cube 的位姿。
- `T_W_GL`、`T_W_GR`：世界至左右 TCP 的 FK/观测位姿。
- `T_O_GL`、`T_O_GR`：物体至两个固定 grasp/TCP 的完整变换。

默认 `T_O_GL.translation=[0,-.061,0]`，
`T_O_GR.translation=[0,+.061,0]`。
两旋转矩阵为：

```text
R_O_GL = [ 0  1  0 ]      R_O_GR = [ 0 -1  0 ]
         [ 1  0  0 ]               [-1  0  0 ]
         [ 0  0 -1 ]               [ 0  0 -1 ]
```

这与冻结的世界侧吸姿态在 object identity 时一致；object 旋转后，两个完整抓取
变换随物体旋转，而不是继续使用世界 Y 对称假设。

```text
T_W_O_from_left  = T_W_GL @ inverse(T_O_GL)
T_W_O_from_right = T_W_GR @ inverse(T_O_GR)
Delta_LR = inverse(T_W_O_from_right) @ T_W_O_from_left
translation_error = ||translation(Delta_LR)||_2
rotation_error = ||Log_SO3(rotation(Delta_LR))||_2
```

还分别将两个 implied object pose 与给定 `T_W_O` 比较，使用同样的平移及
SO(3) geodesic angle。因此两侧共同偏移 10 mm，即使互相闭合为零，也不能通过。
完整 π 旋转错误会被拒绝；没有用 TCP 间距或 Y 对称代替 SE(3) 约束。

## 4. FK、IK 与独立数值核验

`model.RobotModel` 从冻结 URDF 构建完整 joint tree，以
`T_parent @ T_joint_origin @ R_axis(q)` 计算 FK；输出包括双臂所有 link、工具与
TCP。限位读取并核对冻结的 USD/YAML 交集，不放宽原始上下界。
低层 FK 可用于 out-of-limit 代数参考；**FK 计算成功不表示关节状态获准**，
最终管线必须先通过有效限位门禁。

IK 使用 SciPy `least_squares(method="trf")`，带冻结 bounds、固定 seed 28 的
候选序列、明确 attempt/nfev 上界。它是离线单臂逆运动学求解器，未实现闭链
投影、NR 扩展或采样规划。返回解必须重新计算完整 FK 并通过位姿与限位残差。
`IK_NO_SOLUTION_WITHIN_BUDGET` 只表示局部求解未在预算内成功，不能证明不可达。
只有目标越过“各段固定 translation 长度之和”的必要 reach-sphere 上界，才返回
`IK_OUTSIDE_REACH_BOUND`；100 m 目标的负例使用这一独立几何证据，不运行求解器。
未求得关节解时 FK 残差为 null，reach bound 超出量另列，不伪造零姿态误差。

独立检查采用不同数据表达和计算路径：

1. **URDF 对 USD oracle**：oracle 使用 G1-0B 冻结审计中的 authored X 轴和
   local0/local1 关节框架，计算 `T_local0 R_X(q) inverse(T_local1)`，不调用
   生产 FK/SE(3) helper。生产实现使用 URDF Z 轴及 origin。分别核对 link0–8
   和 TCP；HOME、固定 seed 的 20 个状态及三个 IK 控制均有覆盖。
2. **手推零角参考**：TCP 位置 left `[.738,-.445,.846]`、right
   `[.738,.445,.846]`，旋转为上文侧吸矩阵。q=0 违反第 4/6 关节限位，
   所以仅作 FK 代数对照，绝不作有效执行状态。
3. **Jacobian**：几何叉积 Jacobian 对独立中心差分；最大元素差约
   `1.04e-10`。这里只验证局部运动学导数，不进行轨迹边碰撞验证。
4. **IK 回验**：目标也由独立 USD oracle 或明确给定的 object pose 定义；
   解回到独立 oracle 再核验，未只与自身 FK 比较。

量化 evidence 中的 48 次独立 TCP 比较，最大 translation 差
`2.42425e-8 m`，rotation 差 `4.89763e-12 rad`，均低于 `1e-6 m/rad`
的跨格式核验门限。

## 5. typed 输入状态有效性

[状态实现](../../offline/task28_g1_1/task28_validator/state.py) 使用 frozen dataclass：

| 类型 | 字段 |
|---|---|
| `JointObservation` | side、frame_id、stamp_s、names、positions_rad |
| `PoseObservation` | frame_id、stamp_s、position_m、quaternion_xyzw、object_id |
| `InputState` | left/right joints、left/right TCP、Cube、attachment mode、左右 attached flags |
| `StatePolicy` | max_age、max_skew、max_future、quaternion norm tolerance；只允许收紧 |
| `StateValidationResult` | valid、稳定的 machine-readable reasons |

只接受按 fixture 顺序给出的 7 个完整关节名；所有离线 observation frame 必须为
`world`，Cube 必须为 `Cube_01`。这不是声称真实 ROS JointState header 原本就有
该 frame；未来 adapter 必须明确转换到此合同。

拒绝缺失/重复/错序关节、非有限或复数值、错误 shape、非法/非单位 quaternion、
错 frame/ID、过期/未来 observation，以及五类 observation 的整体异步。
时间以 caller 显式 `now_s` 为准；默认正例 now=10.0，stamp=9.99。
Cube stamp=9.5 拒绝为 stale；右关节 stamp=9.95 与其他 9.99 的 skew=.04 s，
即使均未过期也拒绝。时间阈值是新的离线门禁，**没有测量真实 ROS timing**。

attachment mode 只支持 typed `DETACHED` 和 `BILATERAL`。
bilateral 要求左右 attached flags 都为 true；detached 要求都为 false；
单臂模式、字符串冒充枚举或不一致 flags 均拒绝。
最终 `ClosedChainValidator` 只接受 bilateral 闭链状态。伪造/NaN policy、
复数被隐式转实数、标量广播等 API 绕过也有回归控制。

## 6. 持件碰撞与预期吸附接触

[碰撞实现](../../offline/task28_g1_1/task28_validator/collision.py) 使用真实 FCL 0.7.0
独立 C++ helper；模型中包含：

- 双臂 link0–7 的 16 个冻结保守实体 hull。
- 两个侧吸工具的 14 个子几何体：每侧 3 box + 4 cylinder。
- 实体 `Cube_01` 和实体 table，共 32 个子几何体。

保守 STL 以 `fcl::Convexd` 实体加载，而非只检查三角形表皮，所以完全被 table
包住的 Cube 仍被检测为碰撞。覆盖非豁免 self-collision、arm-arm、robot/tool/
Cube/table；同一刚体内部的重叠工具原语不互相检查。
原 SRDF 48 个 link pair 按原宏精确展开，对实际几何产生 40 个 skip，
每状态进行 **414 个 shape pair 查询**。没有新增 ACM pair，没有整块跳过
工具/Cube，也没有删除当前 payload。

查询返回 shape_count/checked_pair_count 完成回执；数量不匹配、缺失、重复、
非有限/畸形 contact、后端失败均拒绝，无近似 fallback。默认 helper 根据源码、
FCL 版本和 flags 在 `/tmp` 编译；显式传入的 backend_path 是受信任代码注入点，
回执不是对任意恶意 binary 的认证。

### 6.1 接触不是全局 ACM 豁免

原始 FCL 对接触仍执行完整检查。只有同时满足下列条件，某一条 collision record
可被分类为本状态允许的 attachment contact：

1. mode 为 bilateral，并且此状态的全部八个杯端盖均位于指定 object-local patch：
   X/Z 各 ±24 mm、Y=±60 mm，法向朝向正确的 Cube 面。
2. 只有相应 cylinder 的前端盖记录可接受；manifold/杆件不可接受。
3. cap axis 角误差 ≤`1e-6 rad`，cap centre/contact plane 误差 ≤`1e-6 m`，
   contact 必须位于 cap radius 内，绝对 penetration ≤`1e-6 m`。
4. contact normal 角误差 ≤`.01 rad`；零深度 GJK normal 条件较差，所以该项
   比独立 cap-axis 门禁宽，但不放宽平面、位置或 penetration。截满 128 条
   contact 的情况不授予接触例外。

mode 本身不能授权接触；对侧缺失、错侧、patch 平移、深穿透和 manifold 碰撞
都拒绝。ACM 数据从未被修改。真实 suction 开闭/约束力未检查，也未声称证明。
端到端默认的 1 mm 间隙正例不需要任何接触例外；零间隙控制是独立碰撞诊断，
不是另一套默认抓取变换。

### 6.2 故意碰撞控制

| 控制 | 结果 |
|---|---|
| nominal 1 mm gap | FCL 无碰撞，不使用接触例外 |
| 正确八杯端面精确接触 | 4 个实际返回 contact 均通过局部门禁，最大深度约 `8.38e-15 m`；未返回接触的精确零间隙受浮点/GJK影响，不冒称八个杯均被物理检出 |
| 相同接触但 detached | 拒绝，不授予任何接触例外 |
| 单侧工具缺失 / 错侧 / patch 平移 | 拒绝 |
| cup 向内穿入 2 mm | 拒绝，最大检出深度约 `.002 m` |
| manifold/Cube 交叠 | 拒绝，不能被 cup 接触规则遮蔽 |
| 冻结 initial/target 的 Cube/table 精确支撑接触 | 两臂 IK 均成功，但 carried-state 拒绝零深度接触；不自动授予 table 豁免 |
| Cube/table 穿透 20 mm | 拒绝，检出深度 `.020 m` |
| Cube 完全包在 table 内 | 拒绝，检出深度 `.160 m` |
| Cube/arm、arm-arm、非豁免 self、robot/table | 均有独立故意重叠负例并被拒绝 |

冻结 fixture 的 initial `[.65,0,.26]` 与 target `[.82,-.075,.26]` 也已单独核验：
两臂 IK 均在有效限位内成功，FCL 唯一拒绝项是 Cube/table 零深度支撑接触。
因此不能把本次三个 lifted 正例解释为完整抓取/放置流程已经通过；支撑与释放
模式必须在后续显式定义，不能直接放宽全局 Cube/table ACM。

部分碰撞控制刻意把 link 放在独立指定的非运动学位姿，目的是单独验证后端，
不能当成机器人可达姿态。三个端到端正例则使用真实 FK 和有效 joint limits。

## 7. 保守几何评估

原始 Isaac arm collision 点在主 USD 内，以 authored `convexHull` 表示；
原始 MoveIt 使用 `franka_description` 2.8.1 STL。两者 joint frame 对齐，
但点集不完全相同，link0–7 的 AABB 最大差约 `.335–5.528 mm`。
新 fixture 用双方点集的 convex union，再外扩约 `1 µm` 防止 float32 舍入。
已有逐面包含证明与 hash；本次原样使用，**没有缩小网格让测试通过**。

可能的 false positive 包括：

- convex hull 填平原 STL 的凹陷/空隙，使邻近工具、Cube 或另一臂被提前判碰；
- USD/STL 两源局部外包络不同，union 在腕部/基座或狭窄通道多占毫米级空间；
- 数值 padding 在极小余量、桌面/邻箱擦边状态产生额外拒绝；
- 精确接触的 GJK normal/深度可能不稳定，严格门禁可拒绝本来预期的接触。

接受 false positive 造成的保守拒绝，不通过扩大 ACM、忽略 payload 或缩几何来
提高通过率。G1-1 的 FCL 实体并不证明 PhysX cooked hull/contact 与其完全一致。
隐藏 hand/finger 惯性及真实吸附动力学沿用 G1-0B 未验证边界，均不在本次证明内。

## 8. 正负控制与复现

完整命令（不会启动 ROS/Isaac）：

```bash
python3 offline/task28_g1_1/run_validation.py --output /tmp/task28_g1_1_results.json
```

依赖与 compile 命令见 [README](../../offline/task28_g1_1/README.md) 及
[backend README](../../offline/task28_g1_1/task28_validator/backend/README.md)。
当前 Python 3.10.12、NumPy 1.24.4、SciPy 1.8.0、pytest 6.2.5、FCL 0.7.0；
helper 源码和已用 libfcl hash 均有记录。测试/求解使用固定种子及固定输入；
不下载依赖、不写机器人状态。单状态 collision 不等于 trajectory safety。

三个全管线正例，object pose 均为明确指定的离线测试状态：

| 状态 | object position / yaw | 左右 implied object 平移残差 m | 旋转残差 rad | FCL |
|---|---|---:|---:|---|
| 对称 bilateral | `[.65,0,.40]` / 0 | `1.86694e-9` | `2.05718e-8` | clear，414 pairs |
| 非零共享平移 | `[.70,-.025,.45]` / 0 | `1.05540e-7` | `1.42532e-6` | clear，414 pairs |
| 共享物体旋转 | `[.65,0,.45]` / .15 rad | `4.04325e-8` | `1.32181e-6` | clear，414 pairs |

另有不依赖 FK 的手写 90° object rotation 正例，验证两侧位置/旋转同时随物体
变换。三个实际模型正例的六次 IK 最大 translation 残差 `1.77068e-7 m`、
最大 rotation 残差 `1.42552e-6 rad`，均在 IK 接受门限内；
逐臂精确数值和关节向量见 evidence（不将该残差解释成物理放置误差）。

负例包括：grasp transform 错 10 mm（闭合平移 `.010 m`）；保持 TCP 位置但
错误 π orientation（闭合旋转 π；implied object 平移也改变）；100 m 不可达；
限位越界 10 mrad；stale/asynchronous/missing/NaN/quaternion/frame/Cube ID/
attachment 错误；故意 payload/environment 碰撞。每项记录拒绝 reason。
同一状态若先出现闭合错误，后续 collision 仍被检查；没有用其他错误掩盖 payload。

## 9. G1-2 的剩余要求与停止点

G1-1 只给出单状态 predicate；开始 G1-2 前仍需单独审阅与授权，并定义：

1. 完整 trajectory-edge 验证：两臂/共享物体的联合插值、沿边闭链和限位检查、
   payload 随物体状态更新，以及连续/扫掠碰撞或有明确误差界的自适应离散策略。
2. 能拒绝“两个端点通过但中间 self/arm-arm/payload/table 相交”的负路径控制；
   处理角度 wrap、接近奇异状态及有限步长/误差界，不能只复用端点 PASS。
3. 明确支撑/抓取/搬运/释放以及 detached→bilateral→release 的 mode transition 规则和状态局部接触例外
   的启用/撤销；未完成的接触模式不能变成永久 ACM 例外。
4. 将 grasp 标定、路径段 object pose、两臂 joint/TCP 同步合同绑定，记录未来
   adapter 的 clock domain 和实时 freshness 行为；当前合成时间不代表 ROS 实测。
5. 保留保守网格、原 SRDF、硬 joint limits 和 suction 门限；未来任何修订都需
   明确版本记录。NR 规划、Isaac/物理执行仍需另外工单授权。

**最终状态：`G1_1_PASS`，仅离线定义范围。已停在 G1-1；不自动开始 G1-2，等待审阅。**
