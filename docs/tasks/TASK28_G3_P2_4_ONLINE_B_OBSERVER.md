# Task28 G3-P2-4：B-only 在线外部力矩观测器

**最终状态：`G3_P2_4_PARTIAL`。** 本任务建立了隔离的单 FR3 Isaac 观测采集器、因果动力学候选观测器、预注册实验和可复核证据。静态已知施力的候选估计通过；无载慢速运动也产生了完整候选序列，但由于 Isaac `get_measured_joint_efforts()` 的运动中源语义尚未独立校准，动态源门拒绝，未运行已载运动。这个结果不授予吸附力观测、双臂内力闭环、ROS/机器人执行或物理安全结论。

## 范围、起点和边界

本次开始时本地与 `origin/side-suction-palletizing` 均为 `39a506b6ccd4ee0a498b9f1192b2ee9ba92517e8`，工作树没有与本任务无关的修改。使用的 FR3 USD 是冻结资产，SHA-256 为 `3feceb47747b49f908f9719299132deb9759f2d5edb390d14f0f3ed62c63255f`。没有修改冻结 fixture、SRDF/ACM、G1/NR/QP 或 Task24/26/27；没有发送 ROS 或机器人命令。

采集器只创建 `/World/ProbeFR3` 单臂场景。它没有 Cube、吸盘或 D6 约束，加载的是 link7 虚拟 TCP 处的已知世界系力，作为独立验证 oracle；oracle 在观测器调用完成后才读取，未作为观测器输入。加载工况之间分别 reset，不能当成同一时刻的 loaded/unloaded 传感器配对。

## API 与测量语义审计

本地 Isaac Sim 4.5.0 安装源码已归档到 `evidence/api_preflight.json`，包含路径、行号和 SHA。预检确认：

| 信号 | 本任务实际含义 | 允许的结论 |
|---|---|---|
| `get_joint_positions()/get_joint_velocities()` | 当前 articulation 的 9 个 DOF，名称顺序在线记录 | 可作状态输入；仍需检查时序、有限性和单位 |
| `get_mass_matrices()` | 当前模型的 9×9 广义质量矩阵 | 可进入模型方程；不是实机惯量标定 |
| `get_coriolis_and_centrifugal_forces()`、`get_generalized_gravity_forces()` | 模型补偿项，按 `+c`、`+g` 放在方程左侧 | 只代表仿真模型源项 |
| `get_measured_joint_efforts()` | `get_dof_projected_joint_forces()` 的求解器 DOF 投影力；文档不保证为纯 actuator torque，运动中可能包含 drive、摩擦、限位和约束贡献，并按前一积分步平均 | 只能作为待验证候选输入，不能称为已校准的 `tau_J` 或外力矩测量 |
| `get_applied_joint_efforts()` | 显式 `set_joint_efforts` 的施加值 | 不是位置驱动器实际输出的替代观测 |
| 关节加速度 | 安装的 articulation wrapper 没有公开 getter | 观测器只使用当前与上一帧速度的因果后向差分，不把它称为直接加速度测量 |
| D6/Suction reaction | 现有 bridge 未发布 D6 reaction；G0 审计记录 D6 handle 为私有且未找到可靠 reaction getter | `CLOSED`、break 阈值或未掉落均不能推出吸附力/内力 |

FR3 前 7 个关节是 revolute，单位为 rad、rad/s、rad/s²、N·m；两个手指是 prismatic，单位为 m、m/s、m/s²、N。质量矩阵的每个元素是“第 i 个广义力/第 j 个广义加速度”，因此 arm 计算保留完整 9×9 矩阵和 9 维加速度，最后只取前 7 个力矩投影到 TCP。手指耦合没有被静默删除。

## 观测器契约与方程

对每个时间戳 (t_k)，观测器只接受当前 `q_k,dq_k,M_k,c_k,g_k,tau_k,J(q_k)` 以及上一帧速度历史，不接受 load oracle、paired unloaded frame、未来样本或工况拟合。第一帧状态为 `WARMUP`。严格递增的 evidence identity、时间、frame、DOF 顺序、单位、有限性、质量矩阵对称正定、采样年龄/偏斜和 Jacobian rank/condition 均为硬门；拒绝帧不会推进历史。

采用明确符号约定：

```text
M(q) qdd + c(q,dq) + g(q) = tau_actuator_on_robot + tau_external_on_robot
tau_external_on_robot = M qdd + c + g - tau_measured
J(q)^T w_tcp_on_robot ≈ tau_external_on_robot[0:7]
```

`qdd` 由 ((dq_k-dq_{k-1})/(t_k-t_{k-1})) 因果计算。TCP wrench 是满秩、奇异值和重构残差均过门后的最小范数候选；残差大或矩阵病态时返回拒绝状态。观测器输出的 `SOURCE_UNVALIDATED_CANDIDATE` 是来源状态，不能由调用者字符串改成校准通过。

## 预注册实验与实际运行

实验矩阵、阈值、资产 hash、DOF 名称、dt=1/120 s、采样预算和阶段顺序冻结在 `evidence/experiment_preregistration.json`（SHA-256 `17df28596f336bb00909decb640d1d8b2f713a5ad1d47649e005a4c1c88e0e82`）：

1. 静态：`zero_home/offset/heldout`，以及 `±1 N` 世界 X 力的 home/offset 共 7 个工况，每个 600 步，只评估末 60 步。
2. `motion_zero`：home 构型，关节 1 的 0.12 rad 预注册平滑运动，960 步。它在静态门通过后作为诊断运行。
3. `motion_plus`：只有在无载运动的动态源语义独立通过后才能运行；本次未运行。

采集器 SHA-256 为 `2b1f3d20e8b1130cba7c580c0f84fd321fb9ca9dc86f599ddd63ed4079b2d26e`。采集记录同时保存实际 DOF 名称/索引、q/dq/effort、M/c/g、当前 link7 FK 检查、施力点、时间区间和 PhysX 警告。`fr3_link8` 与 `fr3_hand_tcp` 的负质量/无效惯量警告被原样保留，没有修补资产。

静态与运动的完整压缩原始 JSON、oracle、状态和 SHA 在 `offline/task28_p2_4/evidence/{static_raw,motion_zero_raw}/`；解压后可直接重放。评估器 SHA-256 为 `e1c0cdf235ec861d0b14ac5ac7f3a18c64d696f5ff2835b6d846a4fb8f531193`。oracle 只在 causal observer pass 之后读取。

## 结果

静态评估命令退出码为 0，7/7 工况通过形状、顺序、时序、Jacobian、候选 wrench 和事后 oracle 比较；这只是同一仿真模型内的**静态候选源一致性**。末 60 帧之外的完整评估仍保留。主要最大值如下：

| 阶段/工况 | 最大关节力矩误差 | 最大力误差 | 最大力矩误差 | 最大 Jacobian 重构残差 | 结论 |
|---|---:|---:|---:|---:|---|
| 静态 7 工况末 60 帧总体 | 0.0001370 N·m | 0.0001735 N | 0.0001616 N·m | 0.00000805 N·m | 候选静态门通过 |
| `motion_zero`（959 个候选帧） | 0.009108 N·m | 0.011339 N | 0.007064 N·m | 0.002996 N·m | 动态源门拒绝 |

无载运动的观测器状态为 1 个 `WARMUP`、959 个 `CANDIDATE_ESTIMATE`，但评估结果明确包含 `INDEPENDENT_DYNAMIC_SOURCE_SEMANTICS_NOT_VALIDATED`。低于静态式误差阈值不等于通过，因为当前 effort 是求解器投影量，且 effort 的前步平均时间支撑与当前 q/dq/M/c/g 不相同；运动中的驱动、摩擦、限位、约束、lag 和数值微分尚未被独立分离。采集器在写完完整 960 步文件后于 USD 资源关闭等待阶段被中断，`collector_status.json` 仍为 `COLLECTION_COMPLETE_NOT_EVALUATED`；证据清单记录该外层退出码为 130，评估退出码为 1，不将其伪装成成功退出。

`motion_plus` 没有运行，因此没有任何“已载动态”结果，也没有把单臂 link7 虚拟力外推为 Cube、吸盘或双臂内力。历史 G1 几何失败、G3-P2-3 原始双臂 BLOCKED 以及 G3-P2-3-R1 仅虚拟回放 PASS 的边界全部保持不变。

## 独立负控与回归

`offline/task28_p2_4/tests/test_observer.py` 的 46 项聚焦测试通过，覆盖：

- 满质量矩阵耦合和合成 Newton 方程的已知 wrench；
- TCP 列空间外扰、rank deficiency、病态条件数；
- 错误 DOF 顺序、frame/TCP、混合单位、prismatic 手指、非有限值和非正定矩阵；
- 重复 evidence、过期/乱序/未来时间、异步样本、非法 provenance；
- 第一帧 warmup、因果加速度、未验证 source 只能给 candidate；
- 错误符号不能悄悄翻转，重放旧 effort identity 会拒绝。

原有跨任务回归命令（不包含本任务目录）仍为 **731 passed，exit 0，98.23 s**，输出保存在 `/tmp/task28_p2_4_regression.log`；任务聚焦命令为 **46 passed，exit 0，0.11 s**。两者的命令、环境和 hash 见 `evidence/regression_record.json` 与 `evidence/artifact_manifest.json`。本任务没有改动原有测试的验收标准。

## 尚未满足的条件

在继续任何 P2-5、在线桥接或物理试验前，至少需要：

1. 针对同一 Isaac 版本和同一 articulation 配置，独立验证运动中 projected effort 的符号、时间平均、drive/friction/limit/constraint 分量和当前状态 lag；不能用同一动力学 oracle 拟合自己。
2. 对惯量警告作模型有效性评估，或提供独立、已校准的动力学/力矩来源；静态 oracle 一致性不等于实机 FR3 观测精度。
3. 若要宣称有限吸附或内部力，需要真实/可靠 frame-defined wrench、D6 reaction 或经校准的替代传感，并完成 grasp map、作用反作用和失效/重捕获事件验证。
4. 保持 G2-QP-3 已记录的 side-prefixed joint names、articulation DOF names、legacy topic 与新 fixture namespace、disabled-by-default command processing、反馈 freshness 和实际 servo period 风险；本任务没有激活或修复 bridge。
5. 任何双臂结论都必须重新通过未放宽的 G1 SE(3)、payload collision 和支持/附着门；单臂候选观测器不能替代这些门。

复现实验只需使用报告中固定的四条命令，并将输出目录设为不存在的临时目录；`static_evaluation.json.gz` 与 `motion_zero_evaluation.json.gz` 是逐帧结果，`*_summary.json` 是不含逐帧冗余的快速检查。该任务停在 `G3_P2_4_PARTIAL`，不自动开始下一阶段。
