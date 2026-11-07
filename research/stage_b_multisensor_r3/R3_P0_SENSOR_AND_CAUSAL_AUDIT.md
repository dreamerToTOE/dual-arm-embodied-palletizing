# R3-P0 传感器与因果数据审计

日期：2026-11-07。分支 `research/stage-b-r3-multisensor-plan`，起点
`8af7ee236f7b06afe893ca1efc7b0126b798f4c7`。完整读取本分支 AGENTS.md 和
R3_RESEARCH_PLAN.md 后执行，仅覆盖 P0/P1。使用独立 worktree，既有 R2 证据和
其他工作树修改均未覆盖。机读表和逐源 SHA-256：`r3_p0_sensor_manifest.json`。

## 1. 本地真实可用性

| run | RGB / depth 文件 | runtime 行 | contact 回调行 | 现成视觉 corners |
|---|---:|---:|---:|---:|
| physical05 | 55 / 55 | 9,133 | 38,282 | 55 检出 |
| physical06 | 1,492 / 1,492 | 14,299 | 64,780 | 原先无检测导出；R3 每三帧重新检测 |
| physical07 | 1,550 / 1,550 | 15,479 | 68,018 | 1,296 检出 / 1,550 receipt |

physical06/07 的大文件在：
`.../work/stage-b-r2b-sensor-pipeline-repair/research/stage_b_vision_contact_r2b/evidence/r2b_rear_wall_physical06/07/`。
physical05 在 `.../work/stage-b-r2b-calibration-phase-audit/.../r2b_rear_wall_physical05/`。
完整绝对路径、文件哈希、三组 RGB/depth 抽样 shape/dtype/finite fraction 已写入机读表。
注意真实图像目录是 `r2b_camera/rgb` 与 `r2b_camera/depth`，不是 evidence 根目录。
Git 中缺少大文件不等于本地不可用；现有大文件未复制、删除或重写。

physical07 runtime SHA-256：
`3b6781b0c98be8a32332e454b80a9b7a8c29cac63f39efeced19c319b780fcee`；
contact SHA-256：`ed9724983833ce843ddd58be31416b6f037210c279ebe36b3226272090860df9`。

## 2. 生产者、消费者和身份

| 字段 | 生产代码及语义 | 消费者 / R3 用途 |
|---|---|---|
| RGB | `isaac/scripts/task28_task27_one_pass.py::R2BPhysicalCamera.capture_if_due` 请求 viewport PNG，异步写盘 | `visual/aruco_pose.py::detect_aruco`；R3 仅用像素 corners |
| Depth | 同一类 `depth_camera.get_depth()`，distance-to-image-plane，后续渲染更新时读回 | 可选模拟相机传感器；P1 不融合 |
| ArUco corners | `research/stage_b_vision_contact_r2b/visual/aruco_pose.py`，DICT_4X4_50 / ID23 / 默认 detector | 旧 `qualify_physical_camera.py`；R3 V0/V1 共用完全相同 corners |
| K/T | 相机 metadata 中 authored 参数和 USD 行向量矩阵；旧 K 计算不等于有效 render K | R3 从 SDK 规则固定 K，读回验证；T 继续固定 rear_11 |
| q/dq | `PhysicalTrace._post_step` 中 articulation joint positions/velocities，前 7 关节 | 模拟关节反馈；本 P1 不用作 Cube 估计 |
| TCP | `PhysicalTrace._tcp` 直接读取 `side_suction_tcp` prim 世界变换 | 模拟机器人 link pose，**不是独立重新计算的 q→URDF FK**；P2 再核验 FK |
| suction command | bridge `_last_commanded` | 模拟执行命令；初始 false 可能是默认状态 |
| suction state | `Surface_Gripper.is_closed()` | 模拟闭合反馈，不能独立证明无滑移附着 |
| controller command | runner 包装 feed `_apply_joint_command`，保存 sequence/side/positions、消息 stamp 与 `time.time()` | 最新 receipt 存入 runtime；目标 q 不当作实测 q |
| Cube pose/velocity | `PhysicalTrace` 对 Cube rigid pose/linear velocity 读回 | **evaluation oracle**；R3 只在输出预测后评分 |
| contact | PhysX callback：collider identity、点、normal、impulse、separation，threshold=0 | **evaluation oracle**；pair/impulse/penetration 语义分开 |
| 力/effort | bridge/另一个 torque experiment 有可选模拟 API，但 physical07 未记录 joint reaction/effort force 流 | physical07 此通道 UNAVAILABLE，P1 不用 |
| attachment confidence | physical07 没有独立测量来源 | UNAVAILABLE / UNKNOWN；P2 不能循环证明附着 |

现有 Task26 bridge `_publish` 发出 simulator `/task27/cube_poses`，C++
`ros_ws/src/fr3_dual_palletize/src/task26_truck_box_push_in.cpp::CubeBuffer` 消费它；
Task27 executor 编译复用该代码。因此旧操作控制器仍依赖 GT。R3 的像素估计是独立、
无写入的实验路径，没有宣称 Task27 控制已经去 GT，也没有修改其成功判据。

## 3. 时间轴与同步限制

runtime 约 60 Hz；physical07 camera receipt 间隔中位数 0.1667 s、最大 0.2333 s。
1,550 个相机 receipt 都能在 trace 中找到同 physics_step/time_s；这证明 **receipt
关联**，不证明曝光同步。采集函数在 request 后执行 nested app.update，physics 仍
推进；physical07 receipt→render-wait 中位数 0.1333 s、最大 0.2000 s，writer wall
latency 中位数 0.1663 s、最大 0.7308 s。depth 在更新后读取，不能称同曝光 RGB-D。

controller receipt 的 epoch wall time 与 camera monotonic wall time 不可直接相减；
command 没有权威曝光/物理步关联。physical07 15,479 行中 13,615 行有 command，
在相机帧上 1,347 有、203 缺失；缺失保留 UNKNOWN。

physical05 的 55 个 receipt 可按 step 找到 runtime，但 time_s 原点不一致（55 个都
不相等），且最长无采样间隔 90.65 s。它只作开发/失败诊断，不进入独立动态指标。
physical06/07 均能按 step/time 精确关联，但 exposure-level 误差仍需保留。

PhysX callback 使用当前 `self.step/time_s`；与 post-step callback 的顺序未证明，
存在一步分配歧义。非零 impulse 是模拟冲量证据；penetration 是接触 proxy，不能直接
称测得加载力。旧 `LOADED_SUPPORTED` 是混合 proxy，不能提升为 measured loaded contact。

## 4. 发现的污染路径与 R3 隔离

1. **GT 速度泄漏**：旧 `observer/runtime_adapter.py` 把 `cube_velocity_m_s` 复制为
   `observed_cube_velocity_x_m_s`，再进入 controller_reference。旧 causal payload
   排除 contact pairs 的测试没有排除该 GT 速度。R3 不导入该 adapter，输入白名单不含
   Cube pose/velocity/contact，不能沿用旧“全部 causal”结论。
2. **GT phase 标签**：旧 `qualify_physical_camera.py::phase` 使用 Cube xyz 和 suction；
   R3 丢弃 `visual_detections.phase`，只允许 executor 日志 receipt interval。
3. **K 不一致**：旧 evaluator 计算 `fx=711.11, fy=600`。Isaac Camera SDK 根据
   horizontal aperture 和 resolution 推导有效 vertical aperture，读回为 20.25 mm，
   `fx=fy=711.1111`。本轮独立静态 readback 和像素投影确认该差异。cal24 K 是另一份
   冻结外部标定证据，其 camera-center 误差不是 Cube 在线误差。
4. **marker front**：legacy `make_marker` cell front 实际为 Cube 中心上方 61.5 mm，
   原 evaluator 使用 61 mm；R3 给 V0/V1 同时固定 61.5 mm，无 GT offset 修正。

`observations.jsonl` 与 `oracle.jsonl` 分文件；评估器先保存所有预测再打开 oracle。
committed compact corners、K/T、模型配置与 labels 足够重放 A/B，不依赖本地大文件。
P0 门：**PASS（数据身份与未知项明确）**，历史污染已揭示，旧控制器保持原样。
