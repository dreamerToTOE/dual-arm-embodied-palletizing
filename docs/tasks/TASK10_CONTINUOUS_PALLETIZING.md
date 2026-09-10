# Task10：松协调连续多箱码垛 Demo

状态：✅ 已完成 Isaac 运行时验收（2026-09-10）。

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
| C | 2 / left | `(0.380, -0.250, 0.065)` | `(0.740, -0.120, 0.065)` |
| D | 2 / right | `(0.380, +0.250, 0.065)` | `(0.740, +0.120, 0.065)` |

第一批刻意沿用 Task08 的交叉目标，用于验证时间协调；第二批保持左右各自通道，验证在 A/B 已放置为障碍物时仍能连续完成安全任务。第二批目标位于 `x=0.740 m`，与第一批目标的 `x=0.820 m` 相差 80 mm。所有箱体为动态刚体、Collider、质量 0.20 kg。

第二批源位取 `x=0.380 m`，而非最初的 `x=0.420 m`：实际验证发现后者会让 Isaac 左臂进入接近物理关节限位的构型，造成吸盘中心偏离箱体顶部并使 3 mm 阈值下的吸附失败。新的源位距已验收的 Task07 源位 `x=0.320 m` 为 60 mm；A/C 与 B/D 的箱体净距均为 30 mm。

Task10 的协调器优先只在 `LIFT_TO_PRE_PLACE` 插入局部等待。RRTConnect 的空间路径具有随机性：若 FCL 发现冲突发生在抓取前，则该等待点无法改变冲突。因此 Task10 显式启用二级 `TASK_START_HOME` 策略，让一臂从双方已验证安全的 HOME 保持后再启动；完整轨迹和全部抓放事件同时平移，且候选仍必须经 FCL 复检为 `SAFE` 才能执行。此扩展默认不影响 Task09。

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

`task10_continuous_demo` 的默认 `execute:=false` 仅生成和验证两批候选；在每批预检结束后，它只在 MoveIt World 中写入该批的**计划 release pose**，使下一批看到正确的已放置障碍物。它不发布任何 `/left/joint_command`、`/right/joint_command` 或吸盘命令，也不改写 Isaac 中的动态箱体。`execute:=true` 才进行真实抓放并使用 Isaac Ground Truth 回写。

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
  -p max_wait_sec:=30.0
```

### 5. 真实连续执行

仅在第 4 步两批均输出 `FCL SAFE` 后执行：

```bash
ros2 run fr3_dual_palletize task10_continuous_demo --ros-args \
  -p execute:=true \
  -p wait_step_sec:=0.20 \
  -p max_wait_sec:=30.0
```

## 运行时验收结果

最终一次只读预检（`execute:=false`）通过：

```text
Batch 1: FCL SAFE
  strategy=LOCAL_WAIT_RIGHT
  wait_stage=TASK_START_HOME
  wait=4.400 s

Batch 2: FCL SAFE
  strategy=SIMULTANEOUS
```

随后在同一干净 Isaac 场景执行真实两批任务。每次执行前都重新规划并重新做 FCL 检查；RRTConnect 的候选路径虽有差异，但均由协调器转换为安全时间表：

```text
Batch 1:
  FCL SAFE, LOCAL_WAIT_LEFT at LIFT_TO_PRE_PLACE, wait=1.000 s
  execution: events=8, candidate=28.588 s, physical pause=3.716 s, wall=32.809 s

Batch 2 (A/B 已作为 MoveIt World 碰撞物):
  FCL SAFE, LOCAL_WAIT_LEFT at TASK_START_HOME, wait=0.600 s
  execution: events=8, candidate=22.001 s, physical pause=3.716 s, wall=26.219 s

Task10 PASS：两批次连续松协调任务全部完成。
```

最终 `/task10/box_poses` Ground Truth，以表格中的目标中心为基准：

| Box | 最终 center (m) | 水平误差 | 三维误差 |
| --- | --- | ---: | ---: |
| A | `(0.818837, 0.117698, 0.065000)` | 2.579 mm | 2.579 mm |
| B | `(0.819923, -0.118935, 0.065000)` | 1.068 mm | 1.068 mm |
| C | `(0.740301, -0.120192, 0.065000)` | 0.357 mm | 0.357 mm |
| D | `(0.738574, 0.119332, 0.065000)` | 1.575 mm | 1.575 mm |

四箱均完成 `SUCTION_ON -> ATTACH -> SUCTION_OFF -> DETACH`；第一批放置后的 A/B 被作为第二批的真实 MoveIt World 碰撞物保留，未通过扩大 ACM 或忽略箱体碰撞规避检查。

## 通过条件

```text
Task10 BATCH_1_CROSS：FCL SAFE
Task10 BATCH_1_CROSS PASS：实际执行完成
Task10 BATCH_2_WITH_PLACED_OBSTACLES：FCL SAFE
Task10 BATCH_2_WITH_PLACED_OBSTACLES PASS：实际执行完成
Task10 PASS：两批次连续松协调任务全部完成
```

已保存最终 `/task10/box_poses` 并完成 A/B/C/D 误差统计。若后续运行中任何一批无安全解、吸盘未闭合、MoveIt Attached/World 同步失败或执行失败，节点仍会立即停止后续批次，不尝试未经验证的恢复轨迹。

## 构建记录

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing-task10-continuous-demo/ros_ws
source /opt/ros/humble/setup.bash
source /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws/install/setup.bash
colcon build --packages-select fr3_dual_palletize --symlink-install
```

结果：编译、链接、安装 `task10_continuous_demo` 成功。现有 `Pose` 聚合初始化 warning 与既有 Task07/08 相同；没有新的编译错误。
