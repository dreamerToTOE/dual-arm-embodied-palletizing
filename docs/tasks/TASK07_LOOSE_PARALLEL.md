# Task07：松协调双臂并行基线

## 状态

✅ **本机联合运行验收通过**

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

## 本机验收结果

```text
LEFT / BoxA  SUCCESS
RIGHT / BoxB SUCCESS
Task07 SUCCESS
wall_time = 20.693 s
```

最终放置误差：

```text
BoxA e_xy = 1.38 mm
BoxB e_xy = 1.03 mm
```

日志中左右 `PRE_PICK` 均先完成规划并在 `StartGate` 等待，随后同时打印 `PARALLEL GO`，两条完整 primitive 并发向前执行，因此 Task07 目标已经满足。

## 只验收以下内容

```text
1. 左右 PRE_PICK 在 gate 后可同时开始实际运动：PASS
2. 左右完整 primitive 均成功结束：PASS
3. BoxA 最终到左侧目标：PASS
4. BoxB 最终到右侧目标：PASS
5. 左右 suction / attach / Planning Scene 更新不串线：PASS
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

职责拆分：

```text
Task08 = 发现冲突：预测两条带时间轨迹是否在同一时刻进入危险距离/碰撞状态
Task09 = 主动消解冲突：等待、优先级、时间偏移、必要时局部重规划/绕行
```

因此真正的双臂主动避障/冲突解决从 Task09 开始；Task08 先把“什么时候、在哪里、哪两个对象会冲突”检测可靠。
