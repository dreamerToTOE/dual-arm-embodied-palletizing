# Task07：松协调双臂并行基线

## 状态

🟡 **代码已完成，待本机联合运行验收**

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
