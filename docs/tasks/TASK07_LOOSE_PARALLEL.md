# Task07：松协调双臂并行基线

## 状态

✅ **已本机联合运行验收通过**

Task07 最终结果：左右两个 Task04 primitive 在 `StartGate` 后真正并发执行，两边抓取、搬运、放置与释放均成功。

实际结果：

```text
LEFT / BoxA  SUCCESS, e_xy ≈ 1.38 mm
RIGHT / BoxB SUCCESS, e_xy ≈ 1.03 mm
Task07 SUCCESS
wall_time ≈ 20.693 s
```

## 本 Task 只做什么

Task07 不重新验证 Task04 已经完成的单臂抓放、吸盘、AttachedCollisionObject、桌面避障等基础能力。

唯一新增目标：

```text
Left FR3  -> BoxA -> Task04 primitive
Right FR3 -> BoxB -> Task04 primitive

两个完整 primitive 真正并发执行
```

第一轮故意让两条通道分开，不在 Task07 提前加入冲突解决算法。

## 复用模块

单臂 primitive 时序继续固定为：

```text
PRE_PICK
-> remove target from MoveIt world
-> Cartesian CONTACT
-> SUCTION ON
-> MoveIt attach
-> LIFT
-> PRE_PLACE
-> Cartesian PLACE
-> Attached + suction ON 时先规划 RETREAT
-> SUCTION OFF
-> settle
-> Isaac Ground Truth
-> MoveIt detach + actual pose addWorld
-> cached RETREAT
```

Task07 新建可参数化 ROS2 包：

```text
ros_ws/src/fr3_dual_palletize/
```

核心：

```text
PalletizePrimitive
  + PrimitiveConfig
  + StartGate
```

左右两臂只是同一个 primitive 的两个实例，不复制两份单臂控制器。

## Task07 场景

Isaac：

```text
BoxA pick   = (0.320, -0.250, 0.065)
BoxA target = (0.820, -0.250, 0.065)

BoxB pick   = (0.320, +0.250, 0.065)
BoxB target = (0.820, +0.250, 0.065)
```

两条通道关于桌面中心对称，Task07 只用来证明并行执行。

脚本：

```text
isaac/scripts/task07_parallel_scene.py
isaac/scripts/task07_dual_suction_bridge.py
```

ROS topic：

```text
Left:
  /left/joint_command
  /task07/left/suction_command
  /task07/left/suction_state

Right:
  /right/joint_command
  /task07/right/suction_command
  /task07/right/suction_state

Ground Truth:
  /task07/box_poses   # [BoxA, BoxB]
```

MoveIt 继续直接复用 Task06：

```text
left_arm
right_arm
共享 Planning Scene
Task06 table CollisionObject
```

## 并行机制

左右 primitive 各自规划第一段 PRE_PICK，然后在 `StartGate` 汇合：

```text
Left PRE_PICK planned  --\
                         StartGate -> PARALLEL GO
Right PRE_PICK planned --/
```

之后两条完整 Task04 primitive 独立向前执行。

Task07 不做时空冲突判断。这个能力明确留给 Task08。

## 只验收以下内容

```text
1. 左右 PRE_PICK 在 gate 后可同时开始实际运动
2. 左右完整 primitive 均成功结束
3. BoxA 最终到左侧目标
4. BoxB 最终到右侧目标
5. 左右 suction / attach / Planning Scene 更新不串线
```

不单独重复测试：

```text
单臂 HOME
单吸盘 ON/OFF
单臂穿桌避障
单臂抓放阶段逐项正确性
```

这些能力已经由 Task04~06 覆盖。

## Task07 通过后

立即进入 Task08：

```text
把两条任务主动引入共同工作区
-> 两条单独轨迹都可行
-> 同时执行存在臂-臂 / 臂-携带物冲突
-> 增加时空冲突检测
```

Task08 开始才是松协调避障与调度的核心。

---

## 启动 / 复现指令

### 1. Isaac Sim：恢复 Task06 双臂基础设施

Timeline **Stop** 时依次运行：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task06_dual_fr3_scene.py").read())
```

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task06_dual_ros_graph.py").read())
```

### 2. Isaac Sim：加载 Task07 场景

仍保持 **Stop**：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task07_parallel_scene.py").read())
```

然后点击 **Play**，再运行：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task07_dual_suction_bridge.py").read())
```

### 3. MoveIt2

```bash
cd ~/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fr3_dual_compact_suction_description moveit_dual_compact_suction.launch.py
```

### 4. Task07 主节点

另开终端：

```bash
cd ~/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run fr3_dual_palletize task07_parallel_demo
```

统一启动总表见：

```text
docs/STARTUP_GUIDE.md
```
