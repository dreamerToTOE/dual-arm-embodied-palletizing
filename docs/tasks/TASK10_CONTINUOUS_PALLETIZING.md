# Task10：松协调连续多箱码垛 Demo

状态：🟡 已实现并编译；等待 Isaac 运行时验收。

## 目标

在 Task09 已验证的单批次联合安全执行之上，完成四个 30 mm 小箱体、两批次的连续双臂码垛：

```text
Batch 1: BoxA (left) + BoxB (right)
  -> FCL 联合检查
  -> local wait / SAFE_EGRESS
  -> 实际双臂抓放

Batch 2: BoxC (left) + BoxD (right)
  -> 第一批 A/B 仍作为 MoveIt World 碰撞物
  -> FCL 联合检查
  -> local wait / SAFE_EGRESS
  -> 实际双臂抓放
```

不使用 Task07 的无协调并行执行路径；两批均只消费 Task08-B/FCL 复检为 `SAFE` 的联合候选。

若 Isaac 当前仍有 Task07/09 的 `task07_dual_suction_bridge.py`，Task10 Bridge 会在启动时关闭它并清理同一末端残留的 Task07 Surface Gripper Joint；无需同时运行两套 Bridge。

## 场景参数

| Box | 执行批次 / 机械臂 | 初始 center (m) | 目标 center (m) |
| --- | --- | --- | --- |
| A | 1 / left | `(0.320, -0.250, 0.065)` | `(0.820, +0.120, 0.065)` |
| B | 1 / right | `(0.320, +0.250, 0.065)` | `(0.820, -0.120, 0.065)` |
| C | 2 / left | `(0.420, -0.250, 0.065)` | `(0.740, +0.120, 0.065)` |
| D | 2 / right | `(0.420, +0.250, 0.065)` | `(0.740, -0.120, 0.065)` |

第一批刻意沿用 Task08 的交叉目标，用于验证 local wait；第二批目标位于 `x=0.740 m`，与第一批目标的 `x=0.820 m` 相差 80 mm。所有箱体为动态刚体、Collider、质量 0.20 kg。

## 新增接口

Isaac：

```text
isaac/scripts/task10_continuous_palletizing_scene.py
isaac/scripts/task10_quad_suction_bridge.py

/task10/left/suction_command
/task10/left/suction_state
/task10/right/suction_command
/task10/right/suction_state
/task10/box_poses    # [BoxA, BoxB, BoxC, BoxD]
```

ROS：

```text
fr3_dual_palletize/task10_continuous_demo
```

`task10_continuous_demo` 的默认 `execute:=false` 仅生成和验证两批候选；`execute:=true` 才可能发布 `/left/joint_command`、`/right/joint_command` 和吸盘命令。

## 验收启动流程

### 1. Isaac Sim：Timeline Stop

依次运行：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task06_dual_fr3_scene.py").read())
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task06_dual_ros_graph.py").read())
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing-task10-continuous-demo/isaac/scripts/task10_continuous_palletizing_scene.py").read())
```

### 2. Isaac Sim：Timeline Play

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing-task10-continuous-demo/isaac/scripts/task10_quad_suction_bridge.py").read())
```

确认 Console 显示：

```text
Task10 四箱双吸盘 ROS bridge 已启动
PUB  : /task10/box_poses [BoxA, BoxB, BoxC, BoxD]
```

### 3. MoveIt2 终端

```bash
source /opt/ros/humble/setup.bash
source ~/lmy/dual-arm-embodied-palletizing/ros_ws/install/setup.bash
ros2 launch fr3_dual_compact_suction_description moveit_dual_compact_suction.launch.py
```

### 4. 只读预检（推荐先执行）

```bash
cd ~/lmy/dual-arm-embodied-palletizing-task10-continuous-demo/ros_ws
source /opt/ros/humble/setup.bash
source ~/lmy/dual-arm-embodied-palletizing/ros_ws/install/setup.bash
source install/setup.bash

ros2 run fr3_dual_palletize task10_continuous_demo --ros-args \
  -p execute:=false \
  -p wait_step_sec:=0.20 \
  -p max_wait_sec:=15.0
```

### 5. 真实连续执行

仅在第 4 步两批均输出 `FCL SAFE` 后执行：

```bash
ros2 run fr3_dual_palletize task10_continuous_demo --ros-args \
  -p execute:=true \
  -p wait_step_sec:=0.20 \
  -p max_wait_sec:=15.0
```

## 通过条件

```text
Task10 BATCH_1_CROSS：FCL SAFE
Task10 BATCH_1_CROSS PASS：实际执行完成
Task10 BATCH_2_WITH_PLACED_OBSTACLES：FCL SAFE
Task10 BATCH_2_WITH_PLACED_OBSTACLES PASS：实际执行完成
Task10 PASS：两批次连续松协调任务全部完成
```

还需保存最终 `/task10/box_poses`，计算 A/B/C/D 各自的水平与三维目标误差。若任何一批无安全解、吸盘未闭合、MoveIt Attached/World 同步失败或执行失败，节点立即停止后续批次，不尝试未经验证的恢复轨迹。

## 构建记录

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing-task10-continuous-demo/ros_ws
source /opt/ros/humble/setup.bash
source /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws/install/setup.bash
colcon build --packages-select fr3_dual_palletize --symlink-install
```

结果：编译、链接、安装 `task10_continuous_demo` 成功。现有 `Pose` 聚合初始化 warning 与既有 Task07/08 相同；没有新的编译错误。
