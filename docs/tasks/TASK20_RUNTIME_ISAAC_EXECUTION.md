# Task20-C：Runtime 松/紧协调的 Isaac 单任务物理执行验收

状态：✅ 首次真实 Isaac 紧协调 runtime 单任务执行已通过（2026-09-14）。
多箱连续物理调度仍不在本 Task20-C 的验收范围内。

## 目标与边界

Task20-A/B 已证明运行时 `BoxStateArray -> PlacementSpec -> Router -> Motion/FCL`，但
它们均为只读流程。Task20-C 第一次把通过门禁的候选发送到 Isaac；它严格限制为：

```text
一次仅执行一个 runtime Box（max_tasks = 1）
```

因此它不是“多箱连续物理码垛已经验收”的声明。每个后续物体必须在前一物体实际
释放、落稳、Ground Truth 回写后重新规划和预检。

本次初始物理验收选择 Task18 的固定 replay seed `20260922`。该 episode 有一个
`large_shared_box` 与四个远离共同取放通道的其它 runtime Box；控制器按体积优先
选择大件，并保留其它 Box 为 MoveIt World 碰撞物。目标区域限定为
`x=[0.535, 0.765] m, y=[-0.165, 0.165] m`，Task19 仍在该区域内产生 PlacementSpec。
该窗口的几何中心 `(0.650, 0.000)` 是 Task15 已实际验收的双吸盘共同放置工作区。

## 执行链路

```text
Task20 acceptance Scene (self-contained Task06 + Task18 seed=20260922)
        |
        +-- Task18 Ground Truth bridge --> /task18/box_states
        |
        +-- Task20 dual-suction bridge <--> /task20/{left,right}/suction_*
                                                |
MoveIt / OMPL + Task19 + Task21 Router + Task21 SharedObjectPlanner
        |                                      |
        +-- private SharedBox FCL / TCP gate --+
                                               ↓
SEQUENTIAL_PRE_CONTACT -> DUAL_CONTACT -> dual SUCTION ON -> COMMON_LIFT
-> COMMON_TRANSPORT -> COMMON_DESCENT -> dual SUCTION OFF -> settle
-> Ground Truth writeback -> PREPLANNED_COMMON_RETREAT
```

共同物体不会被伪造为同时 attach 到两个 MoveIt link。MoveIt 在接触后的共同阶段
不能表达这种双 attach；因此 `SharedObjectPlanner` 在私有 FCL scene 内保留真实 Box
碰撞体，只允许它与两只紧凑吸盘的任务定义接触。物理释放后才以 Isaac Ground Truth
将 Box 写回 MoveIt World，然后执行已经在释放前预规划好的退出轨迹。未扩大全局 ACM。

## 新增/修改文件

- `isaac/scripts/task20_runtime_acceptance_scene.py`：全新 Isaac 会话的一键自包含
  场景入口，固定回归 seed，但复用 Task18 的通用构造器。
- `isaac/scripts/task20_runtime_suction_bridge.py`：双 Surface Gripper、双 TCP
  Ground Truth 与落稳物体锁定 bridge；物体路径由 Task18 metadata 动态发现。
- `ros_ws/src/fr3_dual_palletize/src/task20_runtime_execute.cpp`：真实单任务执行器。
- `ros_ws/src/fr3_dual_palletize/src/shared_object_executor.cpp`：紧协调阶段同步驱动、
  吸盘状态、Ground Truth 误差和回写。
- `ros_ws/src/fr3_dual_palletize/launch/task20_runtime_acceptance.launch.py`：物理验收
  的安全参数入口。
- `palletize_primitive.*`：为 loose 路由复用既有完整 TaskTrajectory event 执行语义。

## 已完成回归

```text
colcon build --packages-select fr3_dual_palletize --symlink-install
```

通过。对 Task18 seed `20260922` 的真实 runtime 输入，在实机同构 MoveIt 双臂模型中：

```text
SEQUENTIAL_PRE_CONTACT              PASS
DUAL_CONTACT                        Cartesian=1.0000 + private FCL PASS
COMMON_LIFT                         Cartesian=1.0000 + private FCL PASS
COMMON_TRANSPORT                    Cartesian=1.0000 + private FCL PASS
COMMON_DESCENT                      Cartesian=1.0000 + private FCL PASS
PREPLANNED_COMMON_RETREAT           Cartesian=1.0000 + private FCL PASS
```

此外，在没有 Task20 Isaac Bridge 的隔离 ROS 回归中，该候选在所有规划门禁通过后
以 `Task20 Isaac dual-suction bridge 未就绪` 退出；没有进入发布关节命令阶段。

## Isaac 实际验收结果

用户按本文场景入口、两个 bridge 和执行 launch 运行固定 replay `seed=20260922`。
Router 选择最大运行时物体 `task18_box_01`，输出 `TIGHT_SHARED_OBJECT`；六个实际阶段
全部完成，进程以 exit code 0 正常退出。

```text
DUAL_CONTACT       box_error = 0.034 mm
COMMON_LIFT        box_error = 0.501 mm
COMMON_TRANSPORT   box_error = 0.329 mm
COMMON_DESCENT     box_error = 0.332 mm

placement error       = 0.337 mm
placement orientation = 0.000 deg
max stage error       = 0.501 mm
max relative TCP      = 0.160 mm
wall duration         = 48.060 s
```

最终日志：

```text
Task20-C tight EXECUTION PASS: stages=6 ...
Task20-C PASS route=TIGHT_SHARED_OBJECT object=task18_box_01 ...
Task20-C EXECUTION PASS: runtime route completed in Isaac
```

上述数值均低于本 Task 的 `10 mm / 5 mm / 0.052 rad` 物理门禁。场景画面也确认了
双 FR3 共同搬运、同步释放以及双臂退出；这不是仅由 MoveIt 轨迹或 private-FCL 推断的
成功结果。

## Isaac 验收步骤

### 1. MoveIt 终端

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch fr3_dual_compact_suction_description \
  moveit_dual_compact_suction.launch.py
```

### 2. 从同一 ROS 环境启动 Isaac Sim

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
cd /home/ubuntu2004/isaacsim-4.5.0
./isaac-sim.sh
```

若本机的 Isaac 4.5 安装目录不同，只替换最后两行。必须先 source workspace，
否则 Isaac 中的 `task18_ground_truth_bridge.py` 无法导入 `BoxStateArray`。

### 3. Isaac Script Editor（按顺序）

保持 Timeline **Stop**，运行：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/"
          "task20_runtime_acceptance_scene.py").read())
```

确认 Console 出现 `Task20-C Isaac runtime acceptance scene ready` 与
`fixed replay seed = 20260922`。然后点击 Timeline **Play**，依次运行：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/"
          "task18_ground_truth_bridge.py").read())

exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/"
          "task20_runtime_suction_bridge.py").read())
```

确认第二个 bridge 打印 `Runtime Box ids`，且 `Surface Gripper: threshold=3 mm`。

### 4. 执行终端

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch fr3_dual_palletize task20_runtime_acceptance.launch.py
```

## 通过判据

1. 两臂先后到达 `SEQUENTIAL_PRE_CONTACT`；
2. `DUAL_CONTACT` 后两只吸盘均为 `CLOSED`；
3. 共同抬升、运输和下降期间，各阶段 `box_error <= 10 mm`，相对 TCP 误差
   `<= 5 mm`；
4. 双吸盘同步 `OPEN` 后，落稳 Box 的位置误差 `<= 10 mm`、姿态误差 `<= 0.052 rad`；
5. 最终打印：

```text
Task20-C tight EXECUTION PASS
Task20-C EXECUTION PASS
```

若任一 bridge、Ground Truth、吸附或误差门禁失败，节点会停止后续阶段并发出双吸盘
`OFF`；这应记录为未通过，而不是通过修改 ACM 或忽略碰撞继续执行。
