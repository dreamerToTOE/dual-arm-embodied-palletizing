# R3-P2B-0 同次运行机器人状态与视觉历史契约

日期：2026-11-07。起始提交 `b2f7b9e6048a3efe50bd4c6eea77e767c306dde5`。
分支：`research/stage-b-r3-multisensor-plan`。

**最终决定：PARTIAL。** 同次运行的实测 q/dq、独立 FK、RGB-D、时间元数据与离线关联接口已实现并可确定性重放；FK 和源时钟核验通过。但完整因果时间集合包含缺失的初始化前机器人历史，且移动段的跨命令边界插值没有通过冻结精度门槛。没有把未知图像时间、插值拒绝或静止样本通过包装成精确同步 PASS。停止在 P2B-0。

## 1. 范围、复用与改动位置

先完整阅读根 AGENTS.md、P0/P1、P2A 三份报告、R1/R2 时间报告及现有采集和历史接口。复用 `time_alignment.RobotHistory` 的 q/dq 插值与重新计算 FK、`RobotModel` 的冻结 URDF、P1/R2 的像素估计及原后壁相机。

原一遍执行器包含 Stage A，physical07 侧压执行还保留了 side-LIFT 失败，因此没有为了本接口重跑完整任务。只运行一次同步/运动学探针：Cube_01 从桌面支撑状态开始，z=0.26 m、尺寸 0.12 m、桌面 z=0.20 m；Cube 使用已披露的运动学位置刺激。双 FR3 只施加局部第七关节 ±0.015 rad、周期 3 s 的位置目标，读取实际动力学响应。**它不是吸附、推压或接触验证。**

相机 eye=[1.7,-0.35,0.8]、target=[0.82,0,0.3]、1280×720、K 和 camera-world 矩阵保留现有配置；裁剪/编码网格与 R2 一致。未改动 Task26/27 源文件、驱动参数、控制器、安全条件及成功标准。所有新增文件均在本目录内。

| 文件 | 作用 |
|---|---|
| `config/p2b0_frozen.json` | 采集前冻结的频率、运动、时间策略及数值门槛 |
| `scripts/capture_p2b0_history.py` | 同次运行视觉、物理步实测双臂状态；oracle 分文件 |
| `history_contract.py` | 身份/时钟校验、候选关联、插值与明确拒绝 |
| `scripts/replay_p2b0_history.py` | 像素预测先写出，独立参考后评分；保留失败比较器 |
| `scripts/audit_p2b0_physical07.py` | 原始长缺失段提取及无原始大文件的离线复核 |
| `tests/test_history_contract.py` | 24 项新增正常/负面用例 |
| `evidence/r3_p2b0_sync_holdout01/` | 原始配对 ROI、代表图、状态、oracle、源码快照和清单 |
| `evidence/p2b0_replay/` | 契约输出、逐样本独立评分、汇总与哈希 |
| `evidence/p2b0_interpolation_failure/` | 初版失败数值和历史回执；最终脚本保留可重现比较器 |
| `evidence/p2b0_physical07/` | 独立历史运行的去 GT 观测提取和来源哈希 |

## 2. 来源、单位与时间定义

| 信息类别 | 通道、生产者和获取阶段 | 含义与限制 |
|---|---|---|
| 直接模拟测量 | `SingleArticulation.get_joint_positions/get_joint_velocities`，live PhysX joint tensors；post-step、pre=False、order=0 | 每臂按 fr3_joint1…7 排列，q rad、dq rad/s。绝不以目标替代反馈 |
| 运动学计算 | 冻结 `RobotModel.fk(side,q)`；同一实测 q | `world_from_TCP`，列向量，平移 m、旋转 3×3 矩阵；不是 Cube 位姿 |
| 控制命令 | `research_local_wrist_stimulus`，issue-time、sequence、positions 单列 | 下一步前下发，状态行保存当时最近命令；不是 q 测量 |
| 相机观测 | 同一次 `get_current_frame(clone=True)` 的 rgba 与 distance_to_image_plane | 同 RP、frame_id、两文件 SHA；深度为光轴距离，不是射线距离 |
| 推断量 | 既有 ArUco/P1/R2 RGB-D 箱面拟合 | 只输入图像、K、外参和固定几何；status、corners、残差/退化诊断均保留 |
| 评估专用 GT | `oracle_tcp.jsonl` 中独立 live PhysX link8 tensor @ fixed tool-local；`oracle*.jsonl` 的 Cube/场景状态 | 不传入历史接口和像素估计；只在预测文件封存后独立评分 |
| 不可用 | 新探针的 suction observation/command、tool/wall contact、force | 明确 None/UNAVAILABLE；未创建 Surface_Gripper，不能称 OPEN 或已附着 |

FK 工具变换由初始 USD 的 `inverse(world_from_link8) @ world_from_TCP` 取得，独立 URDF 已包含同一工具 TCP。评估参考每步重新读取 PhysX link8 的 position 和 xyzw quaternion，组合工具变换；没有用当前 USD TCP 冒充独立动力学参考，也没有用标记/Cube GT 调 FK。夹具清单和源文件哈希随清单保存并由 RobotModel 校验。

每条观测以 `(run_id, epoch_id, time_base_id)` 定义时钟身份。
主时钟为从 timeline.play 后每次物理回调累加 dt 的时间；372 行都与 Core simulation time **完全相等**。步号 163–534，dt=0.0166666675359 s，0 缺步。

`timeline_s` 只是原始诊断字段。本配置中 timeline−physics 的 median/max=8.806337/11.577167 s，**不作为可直接互换的时钟**。SDK `rendering_time` 与使用安装版原生 `omni::fabric::RationalTime` 调 `get_sim_time_at_time()` 的 Fabric 映射时间在 75 帧中差值均为 0。ReferenceTime 分子/分母及 Dispatcher 值原样保存；ReferenceTime 的数值不是物理秒，须经安装版 API 映射。

源时钟一致不等于可见 Cube 的精确内容时刻已知。R1/R2 的结论和失败保持有效，没有减去 33.333 ms，也没有把 SDK/Fabric 相等提升为曝光时刻已证实。

## 3. 时间集合与关联契约

`measurement_time_s=null`，`online_fusion_eligible=false`，75/75 帧均为 UNCERTAIN。
本次新增机器人运动改变了采集配置 fingerprint，未重新做场景时钟传递标定。因此完整可容许集合取**当前 epoch 的闭合因果区间 `[0, callback_entry_physics_time]`**；这是保守集合，不是精确测量的置信区间。回调入口不早于其图像内容是此集合的因果前提；重置/旧缓冲帧不可沿用到另一 epoch。

每帧保留：

- `admissible_interval_s`：连续区间本体；没有中点或人为裁切。
- `physics_grid_candidates`：区间内物理网格点及 `robot_state_ref`。引用以三元身份和 physics_step 指向 `robot_states.jsonl`，后者含实测 q/dq 和重新计算的 FK。引用为 null 表示缺少机器人样本。
- `interpolation_domains_s`：可查询连续候选的相邻有效采样区间。**网格不是完整时间集合**；任意非网格候选用 `SynchronizedHistory.at(identity,t)` 获得 q、dq、FK，不能用网格离散化消除歧义。
- `unsupported_prefix_s=[0,2.71666680835)`：手柄初始化前无 q/dq。区间内部缺失样本或命令边界也不能插值。
- `historical_r2_clock_prior`：原 R2 [-1,0] ticks 条件先验作为 fingerprint 不匹配的参考单列，不约束完整区间。历史位置重访别名仍包含在完整因果集合中，不以“最近时刻”删掉。
- SDK/Fabric/ReferenceTime、回调入口/完成 wall、reader arrival 的 wall/physics、writer start/complete wall 分别保留。writer 完成不当成拍摄时间。

关联结果 75/75 有部分机器人支持，0/75 有完整区间支持。网格候选共 28,502 个，16,352 个有实测状态引用（57.3714%），12,150 个无支持。区间宽度 median/P95/max=6.333334/8.553334/8.800000 s；没有以较小条件先验覆盖这个不确定性。

`ALIGNED` 仅表示该候选时刻的机器人历史可计算，不表示该候选就是图像真实时刻。插值 q 后重新 FK，不直接插值 TCP 矩阵。`value_source` 区分精确历史样本和线性插值；后者明确 `INTERPOLATION_ACCURACY_NOT_GUARANTEED`。

| 异常 | 规则 |
|---|---|
| 缺失 q/dq、非有限值、错误维度/来源 | 拒绝样本；不回退命令；缺失步之间禁止默认插值 |
| 非网格候选 | 仅同身份、相邻有效步、bracket≤0.034 s，且命令 receipt 相同才线性插值；离散量保持过去值 |
| 降采样验证 | 仅评估器显式指定 expected_step_stride=2；不是将缺失步静默当正常 |
| 命令发生变化 | 非端点插值返回 UNKNOWN；实测端点仍可查询 |
| 越界 | UNKNOWN，不外推；初始化前、容量驱逐后的历史不补造 |
| 不确定时间 | 保留完整集合；不输出精确时间或 Cube 预测 |
| 过期 | callback→reader 模拟时间>0.15 s 则拒绝；内容年龄另存区间，不能据 callback 新鲜宣称内容新鲜 |
| 重复帧 | 同身份/frame_id/内容为 duplicate；同 frame_id 内容不同为 conflicting duplicate；均不二次关联 |
| dropout | 不生成伪视觉更新，不删机器人状态，不推断附着/滑移 |
| reset | 新 epoch；禁止跨 epoch 插值。相同时钟身份下倒退/重复步拒绝 |
| 时钟缺失/混用 | 无身份、SDK/Fabric 不一致、callback 与物理索引不一致、跨 run 查询均 UNKNOWN |

## 4. 冻结条件、结果及失败诊断

配置在仿真及任何 holdout GT 评分前冻结，没有拟合时间偏移、FK 参数或阈值。一次运行：90 次 warmup、36 次编码对照、150 次动态 app.update；两次物理步/动态 update，RGB-D 每两次 update 保存一次，实际视觉约 15 Hz。原始动态输入和所有 372 行状态均来自此运行。

| 项目 | 数值 |
|---|---|
| 真正关节运动 | 左/右 joint7 peak-to-peak=0.029366314/0.029366195 rad |
| 连续状态 | 372 行、744 arm-samples、0 缺步、q/dq 有限 |
| 独立 FK 位置差 median/P95/max | 0.000238/0.000360/0.000541 mm |
| 独立 FK 旋转差 median/P95/max | 5.1884e-7/7.7173e-7/1.06824e-6 rad |
| 动态 RGB/depth 编码配对 | 75/75；稳定独立编码对照 18/18 |
| ArUco / Cube pose | 75/75 检出；74 PRESENT、1 UNCERTAIN（support rejection） |
| 置信度 | 既有 detector heuristic 0.92369–0.98823；不是概率标定或附着置信度 |
| callback→reader 模拟延迟 | median/P95/max 均 66.666670 ms |
| writer wall 耗时 median/P95/max | 24.453657/30.988808/48.382200 ms；不是图像年龄 |
| 人工 dropout | 隐去 1 s 内 15 帧；60 行机器人状态保留，0 缺步 |

配对证据为同一帧容器和独立编码 ROI 的 RGB/depth 一致；**不是任意场景对象具有统一时间的全局证明**。frame 248 的标记检测置信度仍高，但 P1 unconstrained support check 得 tilt=126.35°、support residual=84.15 mm，因此保持 UNCERTAIN，没有依据桌面 GT 强制通过。

### 真实留出插值：失败保持可重现

从实际 60 Hz 测量序列只保留偶数索引作为 30 Hz 端点，另外 185 个真实奇数索引为留出目标，比较实际 q/dq 及独立 PhysX TCP；不是把插值生成值当真值。最末无双侧括号的样本不纳入，禁止外推。

初版无命令边界检查的线性插值：

| 量 | median | P95 | max | 冻结 max 门槛 |
|---|---:|---:|---:|---:|
| q 误差 rad | 2.76864e-5 | 4.59790e-5 | 8.01384e-5 | 2e-4 |
| dq 误差 rad/s | 0.00288169 | 0.00474714 | 0.00478314 | 0.001 |
| FK→独立 TCP 误差 mm | 0.004334 | 0.007349 | 0.058442 | 0.020 |
| 旋转误差 rad | 2.71160e-5 | 4.626999e-5 | 1.222917e-4 | 2e-4 |

step234 的最大 TCP 插值误差跨越 step233 无命令、step235 首次位置目标；后续更新也是每 2 步改变位置目标，dq 不满足线性假设。没有发现 FK 坐标系或时间偏移错误来解释该误差。初始 source/summary/残差回执保存在 `p2b0_interpolation_failure/`；最终回放仍使用原 `RobotHistory` 输出同一 unchecked baseline，185 行残差与初始文件逐字节一致。

修复仅增加可观察命令变化时的拒绝，不改变控制器、不放宽门槛。最终 35/185 接受（18.9189%），150/185 拒绝（81.0811%），原因均为 `command_transition_interpolation_unsupported`。接受的 35 个稳定样本 q/dq 误差为 0，TCP max=0.000318 mm。**它们不是移动插值精度的证明。**

拒绝策略是在观察首次残差后加的诊断修复，不是新的盲测成功；同一原始序列重放用于验证行为。本轮不继续加仿真刷 PASS。最终汇总的 interpolation check 仍为 false，故总评 PARTIAL。

## 5. physical07 长缺失与 suction/contact 审计

原始 physical07 在本机实际可访问。完整原始根目录、trace/timeline/visual/contact SHA 见 `evidence/p2b0_physical07/provenance.json`；原始 trace 27,530,255 bytes。提取只保留观测 q/dq、吸盘软件反馈/请求、控制 receipt 与图像 receipt，剔除 Cube GT、历史 USD TCP 和 contact oracle，便于不依赖本机大文件复核。

独立历史区间 step14569–15441（242.816679–257.350013 s）：88/88 图像已采集并配对但未检出，观测缺失跨度 14.533334 s，右删失。873/873 q/dq 有限，约 60 Hz，0 缺步；两臂 873/873 suction Boolean，805 个不同 command sequence。该结果说明机器人观测历史能跨越视觉缺失保留，**不证明精确曝光对齐、附着或 Cube 无滑移**。

当前 PhysicalTrace 代码的 suction_state 来自 `Surface_Gripper.is_closed()`，每 post-step 读取约 60 Hz；是模拟软件闭合/附着反馈，不是压力、密封或独立接触传感器。suction_command 是最后请求，语义不同。历史生产者精确版本不完整，不能把当前代码打点顺序补写成旧运行已验证事实。

tool-Cube、wall-Cube pair 和 contact impulse 在 `contact_events.jsonl` 是 PhysX 模拟 oracle，事件驱动并标记 trace 当时 step/time；回调顺序可能影响一步关联，未将它认证为精确传感时刻。impulse 单位 Ns，除 dt 也仅是 oracle 平均力，非工具力传感器。新探针无这些观测，旧 physical07 contact 也没有进入估计输入。**没有把 FK TCP 与 Cube 绑定为刚性关系。**

## 6. 重现与证据路径

以下命令从仓库根目录运行；输出目录必须不存在。新仿真只在需要重采时运行，CPU 重放不需要 Isaac、原始 physical07 大文件或网络。

```bash
# 可选重采：安装版 Isaac4.5 + 已有 R1 native RationalTime binding
DISPLAY=:1 OMNI_KIT_ACCEPT_EULA=YES /home/ubuntu2004/isaacsim-4.5.0/python.sh \
  research/stage_b_multisensor_r3/scripts/capture_p2b0_history.py \
  --run-id r3_p2b0_sync_reproduction

# 提交证据的 CPU 重放，先 observable、后 oracle 评分
python3 research/stage_b_multisensor_r3/scripts/replay_p2b0_history.py \
  --run research/stage_b_multisensor_r3/evidence/r3_p2b0_sync_holdout01 \
  --out /tmp/p2b0_replay_new

python3 research/stage_b_multisensor_r3/scripts/audit_p2b0_physical07.py \
  --data research/stage_b_multisensor_r3/evidence/p2b0_physical07 \
  --out /tmp/p2b0_physical07_new.json

python3 -m pytest -q research/stage_b_multisensor_r3/tests \
  research/stage_b_vision_contact_r2b/tests/test_aruco.py \
  research/stage_b_vision_contact_r2b/tests/test_cube_marker_geometry.py \
  offline/task28_g1_1/tests/test_independent_fk.py \
  offline/task28_g1_1/tests/test_model.py
```

完整证据前缀 `research/stage_b_multisensor_r3/evidence/`：

- `r3_p2b0_sync_holdout01/manifest.json`：采集配置、源、数据 SHA、K/外参、joint order、工具偏置；`config_snapshot.json`/`capture_snapshot.py` 为采集前配置/源码，`simulation.log` 保留原始诊断和 completion。
- 同目录 `robot_history.jsonl`、`physics_history.jsonl`、`observations.jsonl`、`rgb/`、`depth/`、`representatives/` 是主要观测；`oracle_tcp.jsonl`、`oracle.jsonl`、`oracle_scene_probes.jsonl` 是隔离参考。
- `p2b0_replay/robot_states.jsonl`、`visual_history.jsonl`、`associations.jsonl` 是契约；`observable_prediction_manifest.json` 记录 oracle_inputs=[] 及封存预测 SHA；`evaluation_*.jsonl`、`summary.json` 是后评分。
- `p2b0_cpu_tests.txt`：71 passed。负面用例覆盖 stale/重复/冲突帧、缺/异时钟、missing q、reset、跨 run、无效来源、非有限值、GT 注入、命令边界和越界。
- `p2b0_replay_verification.json`：两次独立输出目录，8 个输出 SHA 完全相同；嵌套 runtime_ms 也剔除，采集时真实 wall 时间仍保留。`p2b0_clean_replay_verification.json` 记录暂存归档的独立目录重放核验。
- `p2b0_physical07_summary.json`：旧运行独立审计；`p2b0_interpolation_failure/failure_manifest.json`：失败裁剪范围及重现方式。

没有修改旧 R3 结果或 Task26/27 行为。确定性证明针对本机记录与列出的 Python/OpenCV/NumPy/SciPy 版本，不声称跨任意 BLAS/渲染设备逐位相同。

## 7. P2B-1 建议入口条件（本轮未执行）

可以复用本契约与精确历史采样点，但应先解决或限制移动段插值：在新独立轨迹验证命令切换邻域，或明确只消费有实测支持的候选，保留拒绝与时间集合。若要求完整候选的机器人覆盖，采集时须在相机可能关联的最早时刻前开始记录已初始化关节，或建立独立证据缩小图像时间集合；不能删掉初始化缺口。

后续任何 attachment 条件必须有独立的观测与适用性验证；不能把 is_closed、工具靠近或 FK 推导为已知 Cube。尚未实现融合、Cube 在线预测、接触切换、shadow integration、RL、控制器修改、Stage A/离桌/侧翻研究。
