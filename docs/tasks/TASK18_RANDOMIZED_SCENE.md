# Task18：Randomized Scene + Ground Truth Task Input

状态：🟡 代码与离线接口验证完成；待 Isaac Sim 端到端验收（2026-09-12）。

## 目标

把“初始位置已知并写死”升级为“每次仿真随机生成，控制系统只能从运行时输入获得物体状态”。

第一阶段不引入视觉误差，使用 Isaac Ground Truth 代替未来相机感知输出。

## 场景随机化

每个 episode 随机：

```text
x / y
可选 yaw
box type
box count
```

必须满足：

- 不初始重叠；
- 在至少一台机械臂可达区；
- 与桌边保持安全边界；
- 大件若要求双臂紧协调，则两个吸附点均应处于双臂可行工作区。

## 运行时输入接口

建议统一发布：

```text
DetectedObjectArray / BoxStateArray
```

每个对象至少包含：

```text
id
pose
size
mass / class（若仿真已知）
timestamp
```

未来相机模块只需替换该输入源，Task19+ 不关心数据来自 GT 还是 D455。

## Random Seed

支持：

```text
seed = fixed   # 回归复现
seed = random  # 压力测试
```

每次实验记录 seed，失败场景可一键重放。

## 第一版实现

### 自包含随机 Isaac 场景

新增 `isaac/scripts/task18_randomized_scene.py`。它在空 Stage、Timeline Stop
状态执行时会复用 Task06-A/B，自动准备：

```text
双 FR3 + compact suction
桌面 /World/Table
/physicsScene
/World/Task06ROSGraph
随机 /World/Task18Box_01 ... N
```

因此不依赖此前打开过 Task06 或 Task15。随机类型包括 small/medium/slim 单臂
物体与 `large_shared_box` 双臂物体；每个 episode 随机生成 3--6 件、至多一个
大件。采样器使用 Task15 已成功取料区的保守可达 envelope，并逐一检查：

- 外接圆保守 no-overlap，物体间最小额外间隙 20 mm；
- 距桌边至少 50 mm；
- 单臂物体位于对应侧的可达子区；
- 大件位于双臂公共可行区，并带有左右两个吸附候选。

`TASK18_SEED=<uint32>` 复现布局；`TASK18_SEED=random` 每次生成一个新的 seed，
实际 seed 会打印并写入运行时消息。脚本中的 `python3 ... --validate` 不导入
Isaac，可检查同 seed 确定性、不同 seed 差异和布局安全性。

### 统一 Ground Truth 契约

新增 ROS 消息：

```text
fr3_dual_palletize/msg/BoxState
fr3_dual_palletize/msg/BoxStateArray
```

其中每个 `BoxState` 含 `stamp/id/pose/size/mass/payload_class/allowed_modes`
以及命名局部 `grasp_candidates`；数组消息还有统一 `header` 与 episode `seed`。
`isaac/scripts/task18_ground_truth_bridge.py` 动态扫描 `/World/Task18Box_*`，
读取 USD custom metadata 和每个 physics step 的真实世界位姿，按 10 Hz 发布：

```text
/task18/box_states   fr3_dual_palletize/msg/BoxStateArray
```

Bridge 不会发布机器人或吸盘命令。ROS 侧新增
`task18_runtime_input_demo`：只从该 topic 构造 Task17 `BoxSpec`、MoveIt
`CollisionObject` 和 `world_box_pose * local_grasp_pose` 的 pick pose，因此不读取
源码固定初始坐标。未来 D455 只需替换 publisher，Task19+ 继续使用同一接口。

## 已验证

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing
python3 isaac/scripts/task18_randomized_scene.py --validate
python3 -m py_compile \
  isaac/scripts/task18_randomized_scene.py \
  isaac/scripts/task18_ground_truth_bridge.py

cd ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fr3_dual_palletize --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 interface show fr3_dual_palletize/msg/BoxStateArray
```

实际结果：布局器通过 T18-01/T18-02 的确定性检查；`seed=20260912` 生成 3 件
无重叠、在边界内的物体，`seed=20260913` 布局不同。ROS 包编译通过，消息可由
ROS 发现。另用一次发布的 `BoxStateArray` fixture 验证：运行时 adapter 成功生成
30 mm collision object，且由 pose `(0.420, -0.180, 0.065)` 和局部抓取点
`(0, 0, 0.015)` 得到 runtime pick `(0.420, -0.180, 0.080)`。

补充压力测试：已连续生成并检查 512 个 seed，均无采样失败、初始重叠或越界，且
覆盖 `large_shared_box`。另以 `yaw=90 deg` 的 ROS fixture 验证位姿组合：局部
`(+0.015, 0, +0.010)` 正确得到世界 pick `(0.400, 0.115, 0.110)`。Isaac 4.5
内嵌 Python 已能在 source overlay 后导入 `BoxState/BoxStateArray`；无窗口完整
scene smoke test 因本机资源服务启动阻塞而中止，未作为 Isaac 端到端 PASS 计入。

## Isaac 端到端验收步骤

终端一：必须先 source 新生成的消息环境，再启动 Isaac，确保 Script Editor 能
导入 `fr3_dual_palletize.msg`。

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing
source /opt/ros/humble/setup.bash
source ros_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export TASK18_SEED=20260912
/home/ubuntu2004/isaacsim-4.5.0/isaac-sim.sh
```

在 Isaac Script Editor、Timeline Stop 执行：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task18_randomized_scene.py").read())
```

确认 Console 记录 `Task18 actual seed`、对象数量和每个对象 pose 后，点击 Play，
再在同一 Script Editor 执行 bridge：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task18_ground_truth_bridge.py").read())
```

终端二运行只读验证器（不需要启动 MoveIt，也不执行机器人）：

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 run fr3_dual_palletize task18_runtime_input_demo --ros-args \
  -p timeout_sec:=15.0 \
  -p expected_seed:=20260912 \
  -p min_boxes:=3
```

预期末行：

```text
Task18 PASS：已从运行时 Ground Truth 生成 N 个 BoxSpec、CollisionObject 与 pick pose；未读取任何固定初始坐标，也未执行机器人命令。
```

改为压力测试时，只需在重新启动 Isaac 前执行：

```bash
export TASK18_SEED=random
```

把 Console 输出的实际 seed 重新赋给 `TASK18_SEED`，即可一键重放该失败场景。

## 验收标准

```text
T18-01  同一 seed 可复现完全相同的 Box 场景
T18-02  不同 seed 产生不同初始布局
T18-03  控制节点不读取源码中的固定初始坐标
T18-04  Pick pose 由运行时 Box pose 生成
T18-05  失败日志记录 seed + BoxSpec + pose
T18-06  Task16 benchmark 可批量运行随机场景
```

## 验收状态

- ✅ T18-01：纯布局器同一 seed 产生相同 scene metadata 与 pose。
- ✅ T18-02：相邻不同 seed 产生不同布局。
- ✅ T18-03：`task18_runtime_input_demo` 只订阅 `/task18/box_states`，无固定初始坐标。
- ✅ T18-04：运行时 pose 与局部 grasp candidate 组合生成 pick pose。
- ✅ T18-05：每条 adapter 日志记录 `seed + BoxSpec + pose`，可按 seed 重放。
- 🟡 T18-06：待 Task19 把随机 `BoxStateArray` 接入批量 MoveIt benchmark 后完成。
- 🟡 Isaac 端到端：待按上述步骤在 Isaac Sim 4.5 实机运行 bridge 后完成。

## 边界

- 暂不加入 RealSense / 点云；
- 暂不自动生成 placement pose；
- 本 Task 可继续使用外部给定目标位姿，以单独验证“随机 pick”。
