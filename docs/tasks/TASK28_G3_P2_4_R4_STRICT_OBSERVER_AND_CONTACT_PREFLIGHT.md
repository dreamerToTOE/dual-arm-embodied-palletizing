# TASK28 G3-P2-4-R4：严格 Observer Source Adapter 与 S2 预检

R4 基于 `bfbbab1606e3afe7f06a48aeee4437c3d6cd1aba`，只新增离线 source
adapter、R14 source replay、S2 预检和 P2-3 几何审计。R12/R13 控制器、Task24/26/27、fixture、USD、吸盘、D6、Cube 和历史证据均未改动。

新增 [source_adapter.py](/home/ubuntu2004/Documents/Codex/2026-10-28/github-task28-g2-p5-qp-5/work/dual-arm-embodied-palletizing/offline/task28_g3_p2_4_r4/source_adapter.py) 将 simulation drive/applied/projected/reaction、model-inferred torque、estimated external torque 和真实 FR3 `tau_J` 分成不同 `SourceKind`。每个 accepted sample 保留物理定义、符号、N*m 单位、joint 顺序、joint frame、source timestamp、采样周期、freshness、quality 和 evidence hash。Isaac 来源固定为 `SIMULATION_CANDIDATE`，不能改标为真实 `tau_J`；模型推算和外力估计不能作为 actuator effort；拒绝不会推进 adapter cursor。

R14 no-contact 240 samples 经过 drive/applied/projected 三个独立 canonical descriptors，三路均 240/240 accepted；结果仍是 simulation candidate，`simulation_tau_J_equivalence=false`，没有 contact oracle、硬件或 ROS 证据。

S2 结论为 `NOT_READY / PREFLIGHT_ONLY_NO_CONTACT_RUN`。静态 fixture containment 与 G1-0B static checks 通过，但缺少同场景 observer source qualification、独立 contact force oracle、安全初始无穿透状态、连续局部 swept envelope、contact pair/frame/sign/time 记录、force/energy/motion budget 和 pre-command failure latch。因而本 R4 未启动 Isaac 接触、Cube 运动、Surface Gripper/D6、ROS 或真实机器人。

P2-3 几何审计结论为 `S3 BLOCKED`。形式 9 个窗口在冻结 G1 bilateral closure gate 前全部拒绝；zero case 左/右抓取平移残差为 `0.441309/0.333326 mm`，左右相对平移 `0.169917 mm`、旋转 `0.001336417 rad`，超过 `0.100 mm/0.001 rad`。P2-3-R1 的 virtual wrench replay 不能覆盖共同 object/anchor identity、双侧接触 wrench、有限接触模型或连续安全。

机器证据：

- [source_registry.json](/home/ubuntu2004/Documents/Codex/2026-10-28/github-task28-g2-p5-qp-5/work/dual-arm-embodied-palletizing/offline/task28_g3_p2_4_r4/evidence/source_registry.json)
- [r14_source_replay.json](/home/ubuntu2004/Documents/Codex/2026-10-28/github-task28-g2-p5-qp-5/work/dual-arm-embodied-palletizing/offline/task28_g3_p2_4_r4/evidence/r14_source_replay.json)
- [s2_preflight.json](/home/ubuntu2004/Documents/Codex/2026-10-28/github-task28-g2-p5-qp-5/work/dual-arm-embodied-palletizing/offline/task28_g3_p2_4_r4/evidence/s2_preflight.json)
- [s3_geometry_preconditions.json](/home/ubuntu2004/Documents/Codex/2026-10-28/github-task28-g2-p5-qp-5/work/dual-arm-embodied-palletizing/offline/task28_g3_p2_4_r4/evidence/s3_geometry_preconditions.json)

R4 下一步是建立新的、独立的 single-arm no-Cube contact fixture，先记录 contact oracle、frame/sign/time、局部连续安全 envelope、power/energy/penetration budget 和 fail-closed latch，再考虑 S2。原始 Cube 接近路径继续为 `CONTINUOUS_SAFETY_NOT_ESTABLISHED`。
