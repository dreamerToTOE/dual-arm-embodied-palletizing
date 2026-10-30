# TASK28-T0：双 FR3 关节力矩控制接管

基线为 `bfbbab1606e3afe7f06a48aeee4437c3d6cd1aba`，分支为
`side-suction-palletizing`。本次新增运行时 torque 入口，保留历史位置控制
Bridge、Task24/26/27 场景脚本、Task28 fixture、FR3 USD、L 型侧吸工具、Cube、
桌面、碰撞几何和吸附配置。没有保存 USD，也没有修改任何原场景构建文件。

T0 GUI 运行按当前选择改用仓库原有的 Task27 场景入口
`isaac/scripts/task27_five_cube_center_insert_scene.py`。它复用 Task26 的场景
构造，建立独立 `/World/Task27` 根、双 FR3、L 型侧吸工具、三面车厢墙、X 向导轨、
五个逐件供料 Cube 和中心插入标记；没有创建替代场景。Task27 wrapper 实际通过
绝对路径加载 `/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/`
下的 Task26 源文件，其 SHA256 与当前 checkout 的 Task26 源一致；这一实际加载源已
写入 Task27 机器证据。Task28 G1 fixture 的旧 GUI 证据保留为历史回放，不再作为
本次 T0 的 active scene 选择。

新增代码：

- [offline/task28_t0/torque_interface.py](/home/ubuntu2004/Documents/Codex/2026-10-28/github-task28-g2-p5-qp-5/work/dual-arm-embodied-palletizing/offline/task28_t0/torque_interface.py)：7 个 arm DOF 的严格 pair、单位、关节顺序、时间游标、力矩上限、变化率和 fail-closed gate。
- [isaac/scripts/task28_t0_torque_bridge.py](/home/ubuntu2004/Documents/Codex/2026-10-28/github-task28-g2-p5-qp-5/work/dual-arm-embodied-palletizing/isaac/scripts/task28_t0_torque_bridge.py)：独立 torque sink；运行时将 arm drive mode 设为 `force`、arm Kp/Kd 清零，保留两个手指 DOF 的原始 gain/mode，并用完整 9 维 effort buffer 写入以避免清零手指命令。它拒绝已注册的历史 position sink，不创建 Surface Gripper/D6，不订阅 position command。
- [isaac/scripts/task28_t0_gui_demo.py](/home/ubuntu2004/Documents/Codex/2026-10-28/github-task28-g2-p5-qp-5/work/dual-arm-embodied-palletizing/isaac/scripts/task28_t0_gui_demo.py)：非 headless Isaac Sim GUI runner，默认选择 `task27`，也保留显式 `--scene-profile task28_g1_0` 的历史 fixture 复现入口。Task27 使用 T0 profile-neutral ownership、limits、applied-effort readback 和 fail-closed gate；R8 fixture-only certificate 不会伪装成 Task27 已验证证据。场景构建后通过 Isaac 内置 `SetLightingMenuModeCommand(lighting_mode="Default")` 启用 viewport session-layer light rig；它只影响显示，不保存 USD。

GUI 回放已实际启动 Isaac Sim 4.5 GUI，X11 `DISPLAY=:1`、RTX 3070 和 Task27 TGS 场景均加载成功。仓库记录的完整 GUI 启动方法在
[ISAACSIM_LOCAL_CONTROL.md](/home/ubuntu2004/Documents/Codex/2026-10-28/github-task28-g2-p5-qp-5/work/dual-arm-embodied-palletizing/docs/operations/ISAACSIM_LOCAL_CONTROL.md)：先加载 ROS 2 环境并运行 `isaac-sim.sh`，再在 Script Editor 启用 `isaacsim.code_editor.vscode`。T0 的可复现实验入口用同一 X11 GUI 运行时通过 `python.sh` 启动 `SimulationApp(headless=False)`，不使用 headless Kit。共执行 90 个 physics frame：60 帧保持段，30 帧 HOME 邻域小动作；`frozen_mass_hold` 回放的保持段最大关节位置漂移约 `0.0481 rad`，动作段相对首帧的最大漂移约 `0.0105 rad`，所以这只是控制接管的初步 GUI 检查，不是 G1 精度验收。逐帧记录见
[gui_task27_torque_frames.jsonl](/home/ubuntu2004/Documents/Codex/2026-10-28/github-task28-g2-p5-qp-5/work/dual-arm-embodied-palletizing/offline/task28_t0/evidence/gui_task27_torque_frames.jsonl)，评估见
[gui_task27_evaluation.json](/home/ubuntu2004/Documents/Codex/2026-10-28/github-task28-g2-p5-qp-5/work/dual-arm-embodied-palletizing/offline/task28_t0/evidence/gui_task27_evaluation.json)。结果包括：

- 90/90 frame 完成，Task27 profile-neutral torque gate、applied-effort readback 和 post-step cursor 均通过；
- `position_write_count=0`，`suction_commands=0`，`d6_commands=0`；
- Task27 profile-neutral gate 逐帧记录左右 arm 7 DOF 的 requested/applied effort，projected effort 单独记录为 solver 输出，不称为传感器 torque 或 FR3 `tau_J`；Task27 不使用 fixture-only R8 certificate；
- 运行时保护签名覆盖 Task27 五个 Supply Cube、三面车厢墙、目标标记、桌面、物理场景和 ROS graph，前后相同，arm gain/mode 在停止时恢复，未写入 USD；
- 原始 HOME→Cube 接近路径、Cube 附着搬运、D6、真实机器人、ROS 控制命令和 Sim2Real 均未执行。

GUI 进程、Isaac 版本和 Kit 日志见
[gui_task27_process_proof.json](/home/ubuntu2004/Documents/Codex/2026-10-28/github-task28-g2-p5-qp-5/work/dual-arm-embodied-palletizing/offline/task28_t0/evidence/gui_task27_process_proof.json)。X11 截图文件为
[gui_task27_torque_scene_loaded.png](/home/ubuntu2004/Documents/Codex/2026-10-28/github-task28-g2-p5-qp-5/work/dual-arm-embodied-palletizing/offline/task28_t0/evidence/gui_task27_torque_scene_loaded.png)
和
[gui_task27_torque_final.png](/home/ubuntu2004/Documents/Codex/2026-10-28/github-task28-g2-p5-qp-5/work/dual-arm-embodied-palletizing/offline/task28_t0/evidence/gui_task27_torque_final.png)。两张文件是 Isaac active viewport 的 `1280×720` capture，清晰可见 Task27 双 FR3、L 型侧吸工具、桌面、三面车厢墙和导轨；五个供料 Cube 按原始设计停在桌下 `z=-5 m`，所以当前视角不显示。runner 使用 Isaac 内置 `Default` viewport light rig；它挂载在 USD session layer，仅用于 GUI 显示，没有添加场景 light prim、修改 USD 或修改物理配置，因此 geometry signature 前后仍一致，可视门禁已通过。

离线 T0 gate 测试为 7 passed；汇总见
[task27_regression_summary.json](/home/ubuntu2004/Documents/Codex/2026-10-28/github-task28-g2-p5-qp-5/work/dual-arm-embodied-palletizing/offline/task28_t0/evidence/task27_regression_summary.json)。fixture 静态检查和已有 R8 回归保持独立运行。后续若从 Task27 进入 G1/G2，必须新建 Task27 identity、几何/闭链/碰撞/source gates，并重新验证；Task27 torque GUI 通过不等于既有 `task28_g1_0` G1/G2 离线验收。原始连续安全状态继续保持 `CONTINUOUS_SAFETY_NOT_ESTABLISHED`。
