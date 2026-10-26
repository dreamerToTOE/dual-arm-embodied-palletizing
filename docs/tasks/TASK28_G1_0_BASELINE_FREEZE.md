# Task28 G1-0 / G1-0B｜单 Cube 侧吸基线审计与 NEW fixture 冻结

审计日期：2026-10-26（Asia/Seoul）
仓库：`dreamerToTOE/dual-arm-embodied-palletizing`
分支：`side-suction-palletizing`
G1-0 起始提交：`b2bf8abc42fa88b8f8e590af00c4d6fedf236942`
G1-0B 起始提交：`2ef2edaa0eca86684a0d045f90cdfe35317565f4`

## 当前判定（G1-0B）

**`G1_0B_CONDITIONALLY_READY`**
**fixture：`task28_g1_0_new_single_cube_v0`**
**标签：`NEW_BASELINE_NOT_YET_PHYSICALLY_VALIDATED`**

研究者已明确批准第 5 节 NEW baseline；本工单不再尝试重建 Task24-H 历史执行。
现已在独立 checkout 中实现并冻结 Task28 的场景、桥接、controller 单任务输入、
MoveIt 描述包、生成 URDF、保守碰撞网格、manifest 和静态测试。静态正例通过，
34 个负例均按预期拒绝，25 个外部模型文件 hash 匹配，离线 xacro 再生成与冻结
URDF 一致。`OmniPBR.mdl` 视觉材质 hash 仍为 `UNKNOWN`，所以保留条件就绪状态。
此状态仅表示静态基线就绪，不代表构建、运行时接口、吸附或物理执行成功。

历史审计结论仍为 **`HISTORICAL_FIXTURE_NOT_RECOVERED`**；原 G1-0 的
`G1_0_BLOCKED` 是批准前的历史状态。Task24-H `0.184 / 0.115 mm` 数值保留为
历史报告证据，未转移到 NEW fixture，也未作为新基线实验结果。下面第 1–4 节
保留原 G1-0 的证据和边界；第 5–9 节记录本次已批准的实现及停止点。

## 1. 工作区、起始点与只读边界

| 项目 | 结果 |
|---|---|
| 独立检出 | `/home/ubuntu2004/Documents/Codex/2026-10-25/files-pasted-by-the-user-task/work/g1-0-freeze` |
| branch / HEAD | `side-suction-palletizing` / `b2bf8abc42fa88b8f8e590af00c4d6fedf236942` |
| 远端起点 | `git ls-remote origin refs/heads/side-suction-palletizing` 与起始 SHA 一致 |
| 工作树 | 起始时干净；原工作区 `/home/ubuntu2004/lmy/dual-arm-embodied-palletizing` 未切换、未清理、未覆盖 |
| 已读 | `docs/tasks/TASK28_G0A_INTERFACE_AUDIT.md`、`docs/tasks/TASK28_PAPER_TO_MAIN_RESEARCH_ROADMAP.md`、Task24 报告、相关 Git 历史与本机保留记录 |
| 只读检查 | Git 图/树/blob、路径和参数检索、文件 hash、源码静态一致性比较、文档与日志来源核对 |
| 明确未做 | 未启动 Isaac Sim；未执行 ROS/MoveIt/机器人命令；未构建、训练或运行 NR/QP/P2/RL/B1；未修改 Task24/26/27 路径、ACM、joint limit、吸附参数或成功标准 |

### 1.1 当前 HEAD 的关键静态事实

当前 HEAD 的相关 blob：

| 文件 | HEAD blob |
|---|---|
| `isaac/scripts/task24_side_suction_tight_scene.py` | `18f84afd9ea3e4fe9c7c42abe7e752040f428f7c` |
| `isaac/scripts/task24_side_suction_tight_bridge.py` | `4694b0669d380c6b1abed1d24603f01d705c7276` |
| `ros_ws/src/fr3_dual_palletize/src/task24_side_suction_tight.cpp` | `1bc1ce203a3e3e2ef39d12765e9a077f43974222` |
| `ros_ws/src/fr3_dual_side_suction_description/urdf/dual_fr3_side_suction.urdf.xacro` | `8ed583b1f001fca8adbcc4709e7221d091b5d08e` |
| `ros_ws/src/fr3_dual_side_suction_description/urdf/dual_side_suction.xacro` | `035a05f1a5b9d63cb22bc260c8fde3256e188605` |
| `ros_ws/src/fr3_dual_side_suction_description/srdf/dual_fr3_side_suction.srdf.xacro` | `677ab3bb5ab215d89fb6a9babd81d8f01afc6cd6` |
| `ros_ws/src/fr3_dual_side_suction_description/launch/moveit_dual_side_suction.launch.py` | `55c90a5182e3b68f574360331516dd27cd0a8cc1` |
| `ros_ws/src/fr3_dual_side_suction_description/config/kinematics.yaml` | `e65c7feb4899e61788302bf14673bc6c591d4c71` |

静态读取得到：当前 scene `ACTIVE_CUBE_COUNT=8`，基座为 `(0.65,-0.60,0)` / `(0.65,+0.60,0)`，桌面顶面 `z=0.200 m`；当前执行器首任务引用 `cube_index=6`（`Cube_07`），目标为 `(0.820,-0.120,0.260)`。这就是 G0-A 所记录的单件参数/索引不一致。

## 2. Git 历史恢复结果

### 2.1 早期单件实现确实存在

提交 `1c3a31b348a585b8ad52a975b28e35f6dfd14d8b`（2026-09-17，`fix(task24): isolate single-cube tight-coordination baseline`）修改了四个关键文件，且实现意图明确是单件：

| 文件 | `1c3a31b` blob |
|---|---|
| Task24 报告 | `a63ef8d2e97503c4126589081301ee6ea72c89f0` |
| scene | `23f752535533317f2060fd28b57b059b29567076` |
| bridge | `a2ee8cd1a55ca04e5c3a7816c21fcc78e5cb0931` |
| controller | `550d3e6ee8d31b27faa9dcd5350a69376318015b` |
| side-suction URDF | `6e1bd5a6b727db9ac4c6fe3bee5fb898e01ebfc6` |
| tool xacro | `658e8acb743e525caaa0b1b0db110f80973d3860` |

该提交及其后续 `20b708e`/`00739dd` 的单件候选参数如下：

| 参数 | 早期单件候选证据 |
|---|---|
| Isaac 真实物体 | `/World/Task24/Supply/Cube_01`；`ACTIVE_CUBE_COUNT=1`；PoseArray 只发布 Cube_01 |
| 任务索引 | controller `kTasks[0]` 使用 `cube_index=0` |
| source | `(0.650, 0.000, 0.110) m`（桌面顶面 `0.050 m` + 半边长 `0.060 m`） |
| target | `(0.820,-0.075,0.110) m`，早期报告中的目标高度 |
| 基座 | `(0.65,-0.50,0)` 与 `(0.65,+0.50,0)` |
| Cube | `0.120 m`、`0.8 kg` |
| 侧向间隙 | `20b708e` 后 controller `1 mm`；几何对齐门限 `2 mm` |
| 共同运输 | `kLiftHeight=0.280 m`；`kReleaseGapZ=0.001 m`；FCL sample `0.010 s` |
| 目标误差门限 | `kPlacementTolerance=0.010 m`；姿态门限 `3°` |
| TCP/tool | link8 固定安装；TCP 侧向偏置 `±0.155 m`、局部 `z=0.080 m`；早期 90×90 mm manifold |
| D6 bridge | capture `0.003 m`、force/torque break `1e6`、stiffness `1e4`、damping `1e3`；早期 bridge 使用 `bendAngle=15°` |

`20b708e` 明确将接触间隙从 `10 mm` 改为 `1 mm`，将共同抬升从 `0.160 m` 改为 `0.280 m`，并收紧几何门限；它没有把早期单件 scene 改成最终报告所述的 `0.200 m` 桌面。

### 2.2 成功报告与源码在 Git 图中分离

关键提交拓扑为：

| 提交 | 内容 | 是否含完整 Task24-H 可执行源码 |
|---|---|---|
| `1c3a31b` | 单件隔离实现 | 含早期单件源码，但参数是 `TABLE_TOP_Z=0.050`、目标 `z=0.110`，没有最终成功报告 |
| `20b708e` | 单件几何/吸附门限修正 | 含早期单件源码；仍为 `z=0.110`，bridge 仍与最终报告存在 bend/桌高差异 |
| `00739dd` | 本地控制文档快照 | 含上述早期单件源码和配置 |
| `80550fe` | Isaac 本地控制文档提交 | 其树中不再包含 Task24 scene/bridge/controller |
| `73345de` | Task25 文档提交，写入 Task24-H 成功结果 | **只含文档**；`git ls-tree` 找不到 Task24 scene/bridge/controller |
| `d3efec6` | 重新引入 Task24 八件 source/scene/bridge | 从文档分支重新引入；`ACTIVE_CUBE_COUNT=8`、`Cube_07` 首任务、目标 Y=`±0.120`，不是报告所述单件 fixture |
| `b2bf8ab` | G0-A 文档终点 | 保留上述八件 current path |

`73345de` 的文档记录了最终报告中的 `0.200 m` 桌面、目标 `(0.820,-0.075,0.260)`、两次 `0.184/0.115 mm` 结果，并写明 `bendAngle=0`、`1e4/1e3`、`1e6`。但是该提交树没有产生这些结果所用的可执行 scene/bridge/controller，无法把报告参数和源码 blob 绑定到同一次运行。

随后 `d3efec6` 的新 source/scene/bridge 明确改为八件，scene 使用 `(0.65,±0.60,0)`、桌顶 `0.200 m`、Cube_07/08 首批供料；controller 的 `kTasks[0]` 使用 `cube_index=6` 和 `y=-0.120`。这组源码不能作为 `(0.820,-0.075,0.260)` 单件成功运行的精确来源。

### 2.3 原始运行证据不能完成绑定

G0-A 已检查仓库 tracked 文件、原工作区 artifacts、Task27/Task26 本机日志和 WorkBuddy 记录。Task24-H 报告中的两次单件精确原始完整日志、启动时源码/config/asset hash、Isaac USD 保存版本和 MoveIt overlay 均未找到。现有报告是 `VERIFIED(D)`，不是可独立重放的 `VERIFIED(R)`。

因此，下列事实可以恢复：

- 历史目标位置、最终误差、官方 HOME 文字记录、0.200 m 工作高度的报告参数；
- 早期单件代码的 ID/count 设计和一组可核验的旧 blob；
- 当前八件版本的全部 tracked source blob。

下列事实不能恢复：

- 产生 Task24-H 两次 PASS 的完整 source/config/asset 组合；
- 该组合中桌高 `.050→.200`、基座 `±.50→±.60`、目标 `z=.110→.260`、工具 manifold、D6 bend 和 `+1 mm` Cup 中心偏置各自何时、以哪个文件版本生效；
- 物理运行时的 Isaac USD、FR3 drive/gain/limit、MoveIt overlay、ROS domain 和命令环境 hash；
- 两次 PASS 的原始日志、seed、完整失败分母和可复现墙钟轨迹。

这不是通过再运行实验可以在本任务内合法填补的缺口；任务明确禁止启动 Isaac 或物理复验。

## 3. 按要求核对差异

| 核对项 | 历史报告/早期候选 | 当前 HEAD | 结论 |
|---|---|---|---|
| Cube ID/count | 早期单件 `Cube_01`、index 0、count 1 | 8 个真实 Cube；首任务 index 6=`Cube_07` | 不可直接改参数复用；`active_count=1` 会与 current index 越界 |
| target pose | 报告 `(0.820,-0.075,0.260)` | `(0.820,-0.120,0.260)` | 关键 Y 差 45 mm |
| source pose | 早期候选 `(0.650,0,0.110)`；最终报告未绑定 source | current `Cube_07=(0.690,-0.070,0.260)` | source 和高度都无同一提交闭合 |
| table / Cube z | 早期候选桌顶 `.050`、Cube `.110` | current 桌顶 `.200`、Cube `.260` | 代码演化已发生，不能假定历史运行采用哪版 |
| bases | 早期候选 `y=±.50`；最终报告正文记 `±.60` | current `y=±.60` | 报告参数与早期 source 不同 |
| TCP | 早期/current 均以 link8、侧向 `±.155`、z=.080 为主 | current URDF 与 scene 为 `±.155`、z=.080 | 位置主体可恢复，但 exact tool collision/manifold 版本不同 |
| tool collision | 早期 manifold 90×90；current xacro manifold 70×70 | current route 使用 70×70 | 历史运行的碰撞几何 hash 不可确定 |
| grasp orientation | sidePose left `(√½,√½,0,0)`、right `(√½,-√½,0,0)` | current 同类构造 | 静态朝向可恢复；不等于历史 TCP 实测标定 |
| HOME | 报告写官方 `[0,-π/4,0,-3π/4,0,π/2,π/4]` | current scene 也写该目标 | 文字和 current 配置一致，但历史 USD 收敛/drive 参数无 hash |
| suction | 报告 `3 mm/1e6/1e6/bend=0/stiff=1e4/damp=1e3` | current bridge `3 mm/1e6/1e6/bend=0/stiff=1e4/damp=1e3` | current 配置与报告一致；早期 `20b708e` bridge 为 bend=15°，不能作为历史运行来源 |
| ACM/FCL | 报告声明未扩大 ACM；完整持件 FCL 仍需验证 | current loaded FCL 删除当前 payload，SRDF 仅固定安装相邻允许 | exact historical planning scene/ACM state 未绑定；不能冻结为 NR fixture |
| Isaac/ROS/MoveIt | 报告写 Isaac 4.5/ROS2 Humble/MoveIt2/OMPL，要求 `ROS_LOCALHOST_ONLY=0` | current package/launch 可读，运行 overlay/asset hash 缺失 | 环境家族可恢复，精确运行环境不可恢复 |

当前主机只读环境可记录为：Isaac Sim `/home/ubuntu2004/isaacsim-4.5.0/VERSION`=`4.5.0-rc.36+release.19112.f59b3005.gl`；ROS 2 Humble；MoveIt 2 Debian 包 `2.5.9`；OMPL `1.7.0`；FCL `0.7.0`；原工作区外部 `franka_description` package `2.8.1`、`franka_fr3_moveit_config` package `2.7.1`。这些是**当前检查环境**，不是 Task24-H 两次运行的 provenance；历史 overlay、Isaac FR3 USD、启动时加载的 xacro/install 前缀和 DDS 状态没有可验证 hash。

## 4. 非变更静态一致性检查

本次未运行 build 或 simulator，只做静态检查：

1. 起始 branch、HEAD、remote SHA 一致，工作树无用户改动。
2. `git ls-tree` 核对 `1c3a31b`、`20b708e`、`00739dd`、`73345de`、`d3efec6`、`b2bf8ab` 的路径和 blob；确认成功报告提交与执行源码提交不在同一树。
3. 读取 current scene/controller/bridge/URDF/SRDF/launch/kinematics；确认 current `ACTIVE_CUBE_COUNT=8`、`kTasks[0].cube_index=6`、current target Y=`−0.120`、基座 `±0.60`、桌顶 `.200`。
4. 读取早期单件 scene/controller/bridge/URDF；确认早期 count/ID 为 1/`Cube_01`，桌顶 `.050`、target z=`.110`、基座 `±.50`，并确认 `20b708e` 的 `.001 m` gap、`.280 m` lift、`.002 m` alignment gate。
5. 检查工作树和候选文档 diff；没有修改任何历史 source/config、没有生成 build/install/log，也没有写入 Isaac/ROS 状态。

这些检查证明文本/源码内部的静态不一致；它们不证明任一配置已经物理通过。

## 5. 已批准并建立的 NEW fixture

G1-0B 独立检出目录为
`/home/ubuntu2004/Documents/Codex/2026-10-25/files-pasted-by-the-user-task/work/g1-0b-fixture`，
起始 branch/HEAD/remote 均为 `side-suction-palletizing` /
`2ef2edaa0eca86684a0d045f90cdfe35317565f4`。原用户工作目录未切换、清理或覆盖。
本次交付只新增 `fixtures/task28_g1_0_new_single_cube_v0/` 并更新本文档；既有
Task24/26/27 源文件、CMake 目标、launch、ACM 和吸附参数均保持原样。

| 交付 | 路径与作用 |
|---|---|
| 入口说明 | [fixture README](../../fixtures/task28_g1_0_new_single_cube_v0/README.md) |
| 机器清单 | [fixture_manifest.json](../../fixtures/task28_g1_0_new_single_cube_v0/fixture_manifest.json)，记录 source/config/fixture hashes、原始资产身份、frame、环境依赖、UNKNOWN 和限制 |
| Isaac scene | [task28_single_cube_scene.py](../../fixtures/task28_g1_0_new_single_cube_v0/scene/task28_single_cube_scene.py)，独立根、单 Cube、基座/HOME/工具/桌面/物理设置 |
| bridge | [task28_single_cube_bridge.py](../../fixtures/task28_g1_0_new_single_cube_v0/bridge/task28_single_cube_bridge.py)，严格 count/ID/order/asset 检查，默认仅遥测 |
| controller 输入 | [task28_single_cube_task.py](../../fixtures/task28_g1_0_new_single_cube_v0/controller/task28_single_cube_task.py)，不可变单任务规格，拒绝错误 ID/count/task index/Cube index，不实现规划或机器人命令 |
| MoveIt 独立包 | [task28_single_cube_description](../../fixtures/task28_g1_0_new_single_cube_v0/moveit/task28_single_cube_description/README.md)，包位于 `ros_ws/src` 外，独立 CMake/launch/state merger/environment publisher |
| 冻结规划模型 | [task28_single_cube_frozen.urdf](../../fixtures/task28_g1_0_new_single_cube_v0/moveit/task28_single_cube_description/urdf/task28_single_cube_frozen.urdf)，launch 直接读取，避免运行时加载另一版模型 |
| 原始资产核验 | [fr3_offline_asset_inspection.json](../../fixtures/task28_g1_0_new_single_cube_v0/assets/fr3_offline_asset_inspection.json)，离线 pxr 解析，未启动 Kit/Isaac/ROS |
| 未来启动协议 | [launch_protocol.md](../../fixtures/task28_g1_0_new_single_cube_v0/launch_protocol.md)，明确静态再生成、独立 build/install、场景/桥接/MoveIt 顺序，未在本任务执行启动步骤 |

### 5.1 单对象和参数冻结

| 参数 | 冻结值 |
|---|---|
| 场景根 | `/World/Task28G10SingleCube`，机器人、桌面、physicsScene、ROSGraph 均在该根下 |
| 唯一 Cube | `/World/Task28G10SingleCube/Supply/Cube_01`；count=1，task index=0，Cube index=0 |
| ROS / MoveIt identity | namespace `/task28_g1_0`；PoseArray `world/[Cube_01]`；CollisionObject ID=`Cube_01`；table ID=`task28_table` |
| Cube | 边长 `[.120,.120,.120] m`，质量 `.800 kg`，dynamic，xyzw=`[0,0,0,1]` |
| 初始 / 目标 | `[.650,0,.260]` / `[.820,-.075,.260] m` |
| 基座 | left `[.650,-.600,0]`，right `[.650,.600,0] m`；基座旋转 identity |
| 桌面 | center `[.550,0,.100]`，size `[1.200,.800,.200] m`，顶面 `.200 m` |
| HOME | `[0,-.7853981634,0,-2.3561944902,0,1.5707963268,.7853981634] rad`；两臂相同 |
| link8→TCP 平移 | left `[0,-.155,.080]`，right `[0,.155,.080] m` |
| link8→TCP 局部旋转 | left `Rz(-π/2)`，right `Rz(+π/2)`；与世界抓取姿态分开记录 |
| 世界抓取姿态 xyzw | left `[√½,√½,0,0]`，right `[√½,-√½,0,0]` |
| 吸附 | 每侧一个 Surface Gripper；capture `.003 m`，force/torque break `1e6`，bend=0，stiffness=`1e4`，damping=`1e3`，gravity 不禁用，retry=true；门限未改变 |
| 工具几何 | current 70×70 mm manifold、四个 radius/length=10 mm Cup、80 mm 竖杆、130 mm 横杆；tool xacro 保持原字节 |
| NEW 场景构造设置 | metre/Z-up，gravity `(0,0,-9.81) m/s²`，60 physics steps/s，TGS、CCD、stabilization；这是明确的新构造设置，不是历史运行参数恢复 |

场景拒绝外部任务、物理、碰撞或 ActionGraph 根，不清理其他任务。
bridge 只接受精确的 scene 标记和 `Supply/Cube_01` 子项；默认不构造 gripper、
不创建关节/吸附命令订阅，也不写 ArticulationAction。MoveIt launch 关闭轨迹执行，
RViz 默认关闭，不启动 controller 或机器人驱动。只有 controller 的任务输入接口被
实现；运动算法/执行器不属于本工单。

### 5.2 资产身份与 hash 绑定

主 Isaac FR3 USD 已通过 HTTPS 只读获取并离线解析：

- 身份：`https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Robots/Franka/FR3/fr3.usd`
- 大小：17,462,793 bytes。
- SHA256：`3feceb47747b49f908f9719299132deb9759f2d5edb390d14f0f3ed62c63255f`。
- defaultPrim=`fr3`，metre/Z-up；无 sublayer/reference/payload，碰撞点和物理 joint 数据嵌入主文件。
- 唯一外部资产 `OmniPBR.mdl` 为视觉材质，身份/hash=`UNKNOWN`；未填造值或换模型。

scene/bridge 要求显式本地 `TASK28_FR3_USD` 及与上述冻结值相等的 SHA256，
并核对文件内容；任一缺失或不匹配即拒绝，不搜索 fallback。
外部 `franka_description` 2.8.1 的公共 xacro、FR3 YAML、link0–7 STL/DAE 共
25 个相关文件均已记录并校验 hash；原始 joint-limits YAML 另存字节一致快照。
manifest 记录 Isaac/ROS/MoveIt/OMPL/FCL、Python/xacro 和离线生成器依赖。
运行 overlay、DDS/ROS domain、未来随机算法 seed/时间预算尚无运行记录，明确为
`UNKNOWN` 或 future per-run 必填项。

## 6. Isaac / MoveIt 物理与几何核对及保守处理

离线 USD joint frame 与 FR3 YAML 的 joint1–8 运动学原点/轴在 `1e-6 m/rad`
内一致；base、HOME、table、Cube、link8 工具安装和 TCP 也已在 scene/bridge/
controller/MoveIt 源文件间核对。两项原始模型差异没有被隐藏：

1. **joint limits 不完全相同。** USD 的若干限位比 2.8.1 YAML 更窄。
   两套原始值、USD drive 和限速均保存在资产审计及 manifest。新规划 URDF
   使用 lower 的 max、upper 的 min，以及 velocity/effort 的 min。
   `render_robot_description.py` 验证 14 个关节完整，拒绝任何扩大原始范围的值。
   HOME 在交集内；未修改原 USD、外部 YAML 或旧实验限位。
2. **原始碰撞网格不完全相同。** link0–7 的 USD 点云与原 STL 的 AABB 最大差
   约 `0.335–5.528 mm`；不能宣称字节相同或相互包含。新 Task28 规划网格是
   两方点集的凸包，再外扩约 `1 µm` 以抵消 float32 输出舍入。
   输出后逐面复核：所有源点至少位于面内 `0.5 µm`，实际最差约 `0.997 µm`；
   8 个网格均封闭、正体积，8 个外部点负例被拒绝。此模型是保守规划包络，
   既包含 USD authored convexHull 源点，也包含原 STL 全部三角形顶点。
   它没有替换物理 USD，也不冒称 PhysX cooking/contact 与 FCL 已实测等价。

[碰撞来源与包含证明](../../fixtures/task28_g1_0_new_single_cube_v0/moveit/task28_single_cube_description/meshes/conservative_collision_provenance.json)
保留 source point snapshot、两方源 hash、frame identity 证据、生成算法与环境。
[生成器](../../fixtures/task28_g1_0_new_single_cube_v0/assets/generate_conservative_envelopes.py)
可从冻结点云临时再生成并逐字节比对；现有文件不被覆盖。
renderer 只对预期的 16 个 arm collision URI 作替换，要求原 origin/scale 为 identity。
工具碰撞保持原样，SRDF hash 与旧文件完全相同：
`473c380d9c8468c4f356ae556c3d7870bc78b307e92cab38cce467c7853bded1`。
**没有新增 allowed collision pair，没有删除 payload，没有关闭 FCL。**

Isaac 原 USD 自带的隐藏 hand/finger rigid body 惯性仍保留；场景只按现有侧吸方式
禁用其视觉/碰撞。MoveIt `hand=false` 是运动学/碰撞描述，不能由此声称动力学
质量相同。此差异已记录，后续动力学/物理实验需要单独评估。

环境发布器初始添加 `task28_table` 和 `Cube_01`，随后将有效的单 entry PoseArray
更新为同一个 world collision object。它保留载荷几何，但不是闭链/AttachedBody
validator；未来 validator 可以且必须在候选 Cube 状态下包含该 payload。
当前 state adapter 会保留初始/上次有效状态，尚无 freshness/两臂同步时间门禁，
所以不能把其存在当成可执行路径安全证明。

## 7. 有界静态一致性和负例结果

主要检查命令：

```bash
python3 fixtures/task28_g1_0_new_single_cube_v0/tests/static_check.py
```

结果：**PASS**。同一 checker 还以 `--asset-file` 核对已获取的原 USD，
并以 `--xacro /opt/ros/humble/bin/xacro` 离线再生成完整规划 URDF；结果与冻结
文件一致。`xacro` 只展开 XML，没有启动 ROS node/launch 或 MoveIt。
碰撞生成器的 `--check` 临时再生成 8 个 STL，逐字节一致。

| 检查 | 结果 |
|---|---|
| 正例 | scene/bridge/controller/MoveIt 单 Cube、pose/order/frame、geometry/mass、bases/HOME、局部/世界 TCP、suction、joint limit 交集一致 |
| source/config hash | fixture 文件、旧路径来源、25 个外部模型文件与主 USD hash 匹配 |
| ACM / payload | 原 SRDF/tool/kinematics 字节一致；无新增豁免，Cube world object 持续保留 |
| bundle 负例 17 个 | 错 Cube ID、task index、Cube index、count、TCP 平移/局部旋转、缺 config、缺 asset identity/hash、错 frame/mass、扩大 ACM、新 allowed pair、遗漏 payload、放宽位置/速度/吸附门限均拒绝 |
| task 输入负例 4 个 | 错 ID、count、task index、Cube index 均拒绝 |
| renderer 负例 5 个 | 扩大位置/速度/力矩、NaN、错误碰撞源文件均拒绝 |
| mesh 负例 8 个 | 每个 link 的外部点均拒绝；独立 stdlib 重算法向、闭合和包含关系通过 |
| 非执行边界 | 未 build/install，未启动 Isaac/Kit/ROS/MoveIt，未发机器人或吸附命令，未做物理实验 |

manifest 不做循环自 hash；其身份由本次 Git commit 绑定。文件 hash 列表覆盖
fixture 源码、配置、审计、测试与网格；提交前检查 diff/branch，推送后核验远端 SHA。
本任务未实现 P4-NR、QP、P2、RL 或 B1。

## 8. G1-1 仍需满足的最低前置条件

1. 取得 G1-1 单独授权；使用本次冻结的 manifest/URDF/保守网格/有效限位，
   不沿用旧 Task24 持件 FCL 删除 payload 的路径。
2. 实现离线 FK/IK、闭链残差与 payload-preserving FCL 接口；覆盖双臂、工具、
   Cube 和 table，保持 joint/suction/ACM 门限，并能拒绝错误 ID、法向、count、
   碰撞和不可达目标。此项是 G1-1 的工作，本任务没有提前实现。
3. 在任何未来规划/执行前补充缺失、陈旧、无效和不同步的 Cube/TCP/joint state
   拒绝门禁；不能使用当前 adapter 保留的旧位姿作为实时有效输入。
4. 任何后续 runtime 检查都要另行授权并记录构建/overlay、DDS/ROS domain、
   命令、seed/时间预算、fixture commit 与 loaded asset hashes；未知项继续标
   `UNKNOWN`。视觉 MDL 可单独补 hash，不得借此更换机器人或碰撞模型。
5. 运行时 namespace/scene 插入、PhysX cooking/contact、隐藏手爪惯性、吸附和
   物理误差尚未验证；物理实验必须晚于 validator 门禁，不能由静态 PASS 推断。

## 9. 停止点

**最终状态：`G1_0B_CONDITIONALLY_READY`。** 新 fixture 的静态规格和实现已冻结，
仅视觉材质 hash 为非关键 UNKNOWN；运行验证与上述后续门禁均保留明确边界。
历史 Task24-H 结果继续作为历史报告证据保存。

本任务在 G1-0B 结束；不自动开始 G1-1，不执行启动协议，等待后续授权。
