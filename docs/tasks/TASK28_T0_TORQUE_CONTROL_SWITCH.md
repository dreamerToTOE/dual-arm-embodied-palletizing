# TASK28-T0：双 FR3 关节力矩控制接管

基线为 `bfbbab1606e3afe7f06a48aeee4437c3d6cd1aba`，分支为
`side-suction-palletizing`。本次新增运行时 torque 入口，保留历史位置控制
Bridge、Task24/26/27 场景脚本、Task28 fixture、FR3 USD、L 型侧吸工具、Cube、
桌面、碰撞几何和吸附配置。没有保存 USD，也没有修改任何原场景构建文件。

T0 GUI 运行选用仓库中已批准的 `task28_g1_0_new_single_cube_v0` 场景
（由 Task24-H 几何派生，保留单 Cube、双 FR3、L 型工具和 TGS/60 Hz 约束）。
它是当前 Task28 A/G1 阶段的冻结入口，不是重新创建的简化场景。Task24 当前
8-Cube、Task26 车厢和 Task27 包装入口均只做了只读审查，文件未改动。

新增代码：

- [offline/task28_t0/torque_interface.py](/home/ubuntu2004/Documents/Codex/2026-10-28/github-task28-g2-p5-qp-5/work/dual-arm-embodied-palletizing/offline/task28_t0/torque_interface.py)：7 个 arm DOF 的严格 pair、单位、关节顺序、时间游标、力矩上限、变化率和 fail-closed gate。
- [isaac/scripts/task28_t0_torque_bridge.py](/home/ubuntu2004/Documents/Codex/2026-10-28/github-task28-g2-p5-qp-5/work/dual-arm-embodied-palletizing/isaac/scripts/task28_t0_torque_bridge.py)：独立 torque sink；运行时将 arm drive mode 设为 `force`、arm Kp/Kd 清零，保留两个手指 DOF 的原始 gain/mode，并用完整 9 维 effort buffer 写入以避免清零手指命令。它拒绝已注册的历史 position sink，不创建 Surface Gripper/D6，不订阅 position command。
- [isaac/scripts/task28_t0_gui_demo.py](/home/ubuntu2004/Documents/Codex/2026-10-28/github-task28-g2-p5-qp-5/work/dual-arm-embodied-palletizing/isaac/scripts/task28_t0_gui_demo.py)：非 headless Isaac Sim GUI runner。fixture profile 直接调用 R8 `R8ExclusiveEffortAdapter` 的 ownership、limits、immediate readback 和 post-step cursor，并复用 R8 `controller.py` 的 gravity-compensated impedance torque 计算。

GUI 回放已实际启动 Isaac Sim 4.5 GUI，X11 `DISPLAY=:1`、RTX 3070 和 TGS fixture 均加载成功。共执行 90 个 physics frame：60 帧保持段，30 帧 HOME 邻域小动作；`frozen_mass_hold` 回放的保持段最大关节位置漂移约 `0.0481 rad`，动作段相对首帧的最大漂移约 `0.0104 rad`，所以这只是控制接管的初步 GUI 检查，不是 G1 精度验收。逐帧记录见
[gui_task24_torque_frames.jsonl](/home/ubuntu2004/Documents/Codex/2026-10-28/github-task28-g2-p5-qp-5/work/dual-arm-embodied-palletizing/offline/task28_t0/evidence/gui_task24_torque_frames.jsonl)，评估见
[gui_evaluation.json](/home/ubuntu2004/Documents/Codex/2026-10-28/github-task28-g2-p5-qp-5/work/dual-arm-embodied-palletizing/offline/task28_t0/evidence/gui_evaluation.json)。结果包括：

- 90/90 frame 完成，R8 pre-command、immediate readback 和 post-step cursor 均通过；
- `position_write_count=0`，`suction_commands=0`，`d6_commands=0`；
- 左右 arm 7 DOF 的 requested/applied effort 每帧一致，projected effort 单独记录为 solver 输出，不称为传感器 torque 或 FR3 `tau_J`；
- 运行时 scene geometry signature 前后相同，arm gain/mode 在停止时恢复，未写入 USD；
- 原始 HOME→Cube 接近路径、Cube 附着搬运、D6、真实机器人、ROS 控制命令和 Sim2Real 均未执行。

GUI 进程、Isaac 版本和 Kit 日志见
[gui_process_proof.json](/home/ubuntu2004/Documents/Codex/2026-10-28/github-task28-g2-p5-qp-5/work/dual-arm-embodied-palletizing/offline/task28_t0/evidence/gui_process_proof.json)。X11 截图文件为
[gui_viewport_scene_loaded.png](/home/ubuntu2004/Documents/Codex/2026-10-28/github-task28-g2-p5-qp-5/work/dual-arm-embodied-palletizing/offline/task28_t0/evidence/gui_viewport_scene_loaded.png)
和
[gui_viewport_torque_final.png](/home/ubuntu2004/Documents/Codex/2026-10-28/github-task28-g2-p5-qp-5/work/dual-arm-embodied-palletizing/offline/task28_t0/evidence/gui_viewport_torque_final.png)。由于冻结 fixture 没有 authored light，GUI viewport 在当前环境中保持黑色网格；我没有添加灯光 prim、修改 USD 或修改场景配置，因此截图未能证明画面内清晰可见两台 FR3、L 工具和 Cube。这个可视门禁仍为未完成，T0 不能宣称完全验收通过。

离线 T0 gate 测试为 5 passed；fixture 静态检查和已有 R8 回归保持独立运行。G1 下一步可直接调用 `start_torque_bridge(scene_profile="task28_g1_0", demo=False)`，向 `/task28_t0/{left,right}/torque_command` 发送带有 `frame_id=task28_t0_dual_sync`、7 个 `fr3_joint1..7` 名称和 `JointState.effort` 的同步 pair。G1/G2 算法尚未在本任务中重新运行，原始连续安全状态继续保持 `CONTINUOUS_SAFETY_NOT_ESTABLISHED`。
