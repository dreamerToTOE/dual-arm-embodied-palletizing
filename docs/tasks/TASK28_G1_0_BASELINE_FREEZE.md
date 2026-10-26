# Task28 G1-0｜Task24-H 单 Cube 侧吸基线恢复与冻结审计

审计日期：2026-10-26（Asia/Seoul）
仓库：`dreamerToTOE/dual-arm-embodied-palletizing`
分支：`side-suction-palletizing`
起始提交：`b2bf8abc42fa88b8f8e590af00c4d6fedf236942`

## 最终判定

**`HISTORICAL_FIXTURE_NOT_RECOVERED`**
**最终状态：`G1_0_BLOCKED`**

历史 Task24-H 的最终报告、早期单件实现和后续八件实现都能在 Git 中找到，但没有一个提交同时包含：

1. 报告所述成功运行的全部源码和配置；
2. 精确的单 Cube scene/bridge/controller/MoveIt 输入；
3. Isaac FR3 USD、外部 `franka_description`、ROS/MoveIt overlay 和运行命令的 hash 绑定；
4. 对应两次 `0.184 / 0.115 mm` 结果的原始完整日志。

因此不能把某一个历史提交冒称为 Task24-H 的精确可执行复现，也不能在本任务中静默采用新 fixture。下文给出可审阅的新 fixture 提案；它没有写入源码，没有冻结，也没有生成机器可执行 manifest。需要研究者批准后才能采用。

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

## 5. 未冻结的新 fixture 提案（需要批准）

下面是一个**NEW baseline proposal**，不是 Task24-H reproduction，也没有写入 repository。它的目的是为后续 G1-1 提供一个可独立审阅的起点；批准前不得把它作为实验命令或成功基线使用。

```json
{
  "fixture_id": "task28_g1_0_new_single_cube_v0",
  "status": "PROPOSED_NOT_FROZEN",
  "provenance": "NEW_BASELINE; NOT_TASK24_H_REPRODUCTION",
  "source_commit": "b2bf8abc42fa88b8f8e590af00c4d6fedf236942",
  "scene_namespace": "/World/Task28G10SingleCube",
  "cube": {
    "id": "Cube_01",
    "pose_world_m": [0.650, 0.000, 0.260],
    "size_m": [0.120, 0.120, 0.120],
    "mass_kg": 0.800,
    "pose_array_order": ["Cube_01"]
  },
  "target_world_m": [0.820, -0.075, 0.260],
  "bases_world_m": {
    "left": [0.650, -0.600, 0.000],
    "right": [0.650, 0.600, 0.000]
  },
  "table": {
    "center_world_m": [0.550, 0.000, 0.100],
    "size_m": [1.200, 0.800, 0.200],
    "top_z_m": 0.200
  },
  "home_q_rad": [0.0, -0.7853981634, 0.0, -2.3561944902, 0.0, 1.5707963268, 0.7853981634],
  "side_tcp": {
    "left_link8_translation_m": [0.0, -0.155, 0.080],
    "right_link8_translation_m": [0.0, 0.155, 0.080],
    "left_world_quaternion_xyzw": [0.7071067812, 0.7071067812, 0.0, 0.0],
    "right_world_quaternion_xyzw": [0.7071067812, -0.7071067812, 0.0, 0.0]
  },
  "suction": {
    "one_surface_gripper_per_arm": true,
    "grip_threshold_m": 0.003,
    "force_break_N": 1000000.0,
    "torque_break_Nm": 1000000.0,
    "bend_angle_rad": 0.0,
    "stiffness": 10000.0,
    "damping": 1000.0,
    "disable_gravity": false
  },
  "collision_policy": {
    "acm_changes": false,
    "payload_must_remain_in_validator": true,
    "historical_current_path_payload_gap": true
  },
  "environment": {
    "isaac_family": "4.5",
    "ros": "humble",
    "moveit_ompl": "record_at_freeze_time",
    "isaac_usd_hash": null,
    "external_franka_description_hash": null,
    "moveit_overlay_hash": null
  },
  "approval_required": true
}
```

提案选择 current 侧吸 URDF/bridge 的已读参数、一个真实 Cube、current 官方 HOME 和报告目标 `(0.820,-0.075,0.260)`，但它**不继承**历史 PASS 的真实性，也不声称 current source/asset 已经在该 fixture 上执行过。`isaac_usd_hash`、外部 Franka 描述、MoveIt overlay 和运行命令必须在批准后的单独冻结步骤中填写；本任务不写入 manifest，因为该 fixture 尚未成功恢复或获批。

## 6. G1-1 开始前的最低验收条件

在开发闭链/IK validator 前，需要先由研究者批准“恢复历史”还是“采用 NEW baseline”。无论选择哪条路线，至少满足：

1. **单一对象契约**：Isaac stage、bridge PoseArray、controller index、MoveIt CollisionObject 的数量均为 1，唯一 ID 和顺序均为 `Cube_01/index=0`；无 `Cube_07` 隐式索引。
2. **完整参数 manifest**：scene、bridge、controller、URDF/SRDF、launch、kinematics、外部 Franka 描述、Isaac FR3 USD、ROS/MoveIt overlay、命令、环境变量和 hash 全部记录。
3. **frame 一致**：world、base、link8、side_suction、TCP、Cube object frame、PoseArray frame_id 和规划 world shift 明确；两 TCP 的平移/旋转从同一版本 scene/URDF 派生。
4. **固定吸附条件**：一侧一个 Surface Gripper；capture、D6 break、bend、stiffness、damping、gravity、retry 和 release 语义预先冻结，不在 validator 开发中调参。
5. **碰撞静态门禁**：保留现有 ACM 和 joint limits；离线 validator 必须把 Cube payload 放入候选对象状态，覆盖 robot/tool/object/table，且至少能拒绝错误 ID、错误法向、错误 count、故意碰撞和不可达目标。
6. **规划/执行分离**：G1-1 只做离线/静态 IK、FK、闭链残差和 FCL 接口；不复用历史 PASS 作为新算法成功，不启动 Isaac，不写 joint/suction 命令。
7. **版本停止条件**：若无法填写任一 asset/source/environment hash，继续标为 `UNKNOWN` 并停止历史复现主张，不把报告数值复制到新 fixture 的结果栏。

## 7. 停止点与后续请求

G1-0 已完成历史搜索和静态核对，但**没有冻结任何 fixture**。目前不能安全地提交机器可执行的“Task24-H 精确 baseline manifest”，也不能在未批准的情况下采用上面的 NEW baseline。

需要研究者先选择并批准：

- **恢复路径**：提供/确认缺失的 Task24-H 运行源码、scene/bridge、Isaac USD、MoveIt overlay 和原始日志，使其可以完成 hash 闭合；或
- **新基线路径**：批准 `task28_g1_0_new_single_cube_v0` 提案，并允许另一个明确工单创建独立 fixture、manifest 和静态验证。

在该选择和批准到达前，停止 G1-0，不开始 G1-1。
