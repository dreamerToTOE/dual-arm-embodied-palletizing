# 总体项目规划 — 双机械臂具身智能码垛

> **本文件作为项目工程路线的主要记录。**
>
> 当前阶段优先把“双臂混合码垛协调系统”做出来，不提前绑定最终论文各章节写法。论文结构在工程主线稳定后再整理。

## 1. 当前工程目标

项目面向混合尺寸箱体码垛，以 Franka FR3 + Isaac Sim + ROS 2 Humble + MoveIt 2 为主要平台。

当前核心目标不是先组织论文，而是完成一套统一的双臂码垛系统：

```text
同一套固定末端执行器
        ↓
根据箱体尺寸 / 质量 / 单臂可搬运性
        ↓
      模式选择
      ↙      ↘
   松协调    紧协调
      ↓        ↓
两臂分别抓    两臂共同抓
不同箱体      同一大箱体
      ↓        ↓
并行码垛     协同搬运与放置
```

## 2. 固定末端执行器决策

Task 03 已证明普通 Franka 二指夹爪在高密度码垛中存在结构性限制：

```text
箱体本体可以进入目标空位
但两侧 finger 需要额外侧向空间
→ C_insert = FAIL
```

因此后续工程主线固定为：

> **左右 FR3 始终使用同一种固定的顶部吸附式末端执行器。**

### 小箱体 / 松协调

每个箱体由一台机械臂从顶部单点吸附：

```text
Left FR3  → Box A
Right FR3 → Box B
```

两臂可独立或并行执行，通过时空碰撞检测和局部重规划解决冲突。

### 大箱体 / 紧协调

两台机械臂从箱体上表面两个分离吸附点共同吊运同一个箱体：

```text
     Left suction        Right suction
          ↓                   ↓
      ┌──────────────────────────┐
      │        Large Box         │
      └──────────────────────────┘
```

紧协调阶段先做工程闭环：

```text
双吸附点建立
→ 两臂同步抬升
→ 共同搬运
→ 共同下降
→ 同步释放
→ 两臂退出
```

后续是否加入力 / 负载分配 / 柔顺控制，根据实际仿真表现和工程需要再决定。

## 3. 紧凑吸盘与 MoveIt / Isaac 模型统一

第一版不模拟真空流体细节，采用：

```text
紧凑刚性吸盘工具几何
+
吸附 ON / OFF 状态
+
Isaac Surface Gripper
+
MoveIt AttachedCollisionObject
```

Task05 已确认：Franka 官方 `cobot_pump` 的环境碰撞包络与本项目的紧凑吸盘不一致，会在 B-A-C 高密度插入中产生假碰撞。

因此项目建立自定义：

```text
fr3_compact_suction_description
```

几何与 Isaac 基线保持一致：

```text
stem radius = 6 mm
stem length = 99 mm
cup radius  = 10 mm
cup length  = 6 mm
TCP offset  = 105 mm
```

当前兼容策略继续保持：

```text
MoveIt planning tip = fr3_link8
suction TCP target -> +0.105 m -> fr3_link8 pose target
```

这样不破坏已经验证的 Task04 基础码垛事件单元。

## 4. 基础码垛事件单元（项目级约束）

从 Task04 起，后续所有单臂基础抓放默认复用同一时序，不为每个 Task 重新设计抓放逻辑：

```text
HOME / 当前安全状态
→ PRE_PICK
→ 临时移除待抓物 MoveIt World 碰撞体
→ Cartesian 到 CONTACT
→ CONTACT 后 SUCTION ON
→ 确认吸附
→ MoveIt AttachedCollisionObject
→ LIFT
→ TRANSFER / PRE_PLACE
→ Cartesian PLACE
→ 仍保持 Attached + SUCTION ON，先规划 RETREAT
→ SUCTION OFF
→ 等待物体 settle
→ 读取 Isaac Ground Truth
→ MoveIt detach
→ 按实际 pose 加回 Planning Scene
→ 执行预先规划好的 RETREAT
→ 下一事件
```

后续 Task 的变化主要发生在该事件单元外部：目标位置、双臂调度、冲突处理、同物体协同等。

## 5. 模块化开发原则

从 Task06 起，工程采用“稳定模块复用 + 只验证新增能力”的方式推进，避免重复造轮子。

```text
[SingleArmPalletizePrimitive]  ← Task04 已验证
         ↑ 参数化复用
  ┌──────┴──────┐
  │             │
Left Arm     Right Arm
  │             │
  └──────┬──────┘
         ↓
[DualArm Coordinator]
  ├─ 并行调度
  ├─ 共享 Planning Scene
  ├─ 时空冲突检测
  ├─ 等待 / 优先级
  └─ 局部重规划
```

验证策略：

```text
单臂内部已验证能力 → 不重复逐项回归
双臂新增接口/命名/状态隔离 → 只在首次集成时验证
双臂同时执行/互碰/冲突处理 → 作为后续重点
```

例如 Task06 已经确认左右 joint/TF/MoveIt group 不串线后，不再单独重复做“左臂 HOME、右臂 HOME、左吸盘 ON/OFF、右吸盘 ON/OFF”四组孤立测试；这些能力直接在 Task07 的双臂并行抓放中一起验收。

## 6. 工程实现路线

| Task | 工程里程碑 | 主要验收目标 | 状态 |
|---|---|---|---|
| 00 | FR3 + Isaac + ROS 2 + MoveIt 基线 | MoveIt 轨迹可在 Isaac FR3 上执行 | ✅ 已完成 |
| 01 | 二指夹爪单臂 Pick & Place 基线 | 完整稳定确定性抓放循环 | ✅ 已完成 |
| 02 | Placement Skill | 参数化放置、到达/插入/释放/退出判定 | ✅ 已完成 |
| 03 | B-A-C 二指夹爪可执行性 | 验证高密度放置中的侧向夹爪干涉 | ✅ 已完成 |
| 04 | 顶部吸盘单臂基线 | FR3 使用紧凑顶部吸盘稳定吸取、搬运、放置；形成基础码垛事件单元 | ✅ 已完成 |
| 05 | 吸盘高密度放置验证 | 同一 B-A-C 场景中顶部吸盘完成二指夹爪无法完成的垂直插入；完成 MoveIt/Isaac 模型对齐 | ✅ 已完成 |
| 06 | 双 FR3 + 双吸盘基础设施 | 双臂场景、独立 ROS 控制链、唯一 TF/joint、双臂 MoveIt groups、统一 Planning Scene | ✅ 已完成 |
| 07 | 松协调并行码垛 | 复用同一个 Task04 primitive，两臂分别搬不同箱体并真正同时执行 | ✅ 已完成 |
| 08 | 松协调时空冲突检测 | 臂-臂、臂-携带物、路径时序冲突检测 | ✅ 已完成 |
| 09 | 松协调局部重规划 | 冲突时通过等待 / 主从 / 局部重规划解决 | 🟡 进行中 |
| 10 | 松协调完整码垛 Demo | 多个小箱体连续并行码垛 | 计划中 |
| 11 | 双吸盘共物体基线 | 两臂在大箱顶部两个吸附点同时抓取 | 计划中 |
| 12 | 紧协调同步搬运 | 双臂共同抬升、运输、下降，同步约束稳定 | 计划中 |
| 13 | 紧协调放置与释放 | 大箱共同放置、同步释放、两臂安全退出 | 计划中 |
| 14 | 紧协调改进控制 | 根据需要加入相对位姿、柔顺、负载分配或力反馈 | 计划中 |
| 15 | 松 / 紧协调模式选择 | 根据箱体状态自动选择单臂或双臂操作 | 计划中 |
| 16 | 完整混合码垛 Demo | 同一任务中小箱松协调 + 大箱紧协调连续运行 | 计划中 |
| 17 | 恢复与异常处理 | 规划失败、吸附失败、冲突等恢复行为 | 计划中 |
| 18 | 批量仿真实验 | 为后续论文整理提供定量数据 | 计划中 |
| 19 | RGB-D 感知 | Ground Truth 稳定后再决定是否加入 | 可选 |

## 7. Task03 → Task05 的工程结论

Task03：

```text
A 本体几何可放入 B/C 间隙
→ 修正 MoveIt finger 状态后
→ C_insert 正确检测到夹爪与邻箱干涉
→ 二指夹爪无法完成该高密度放置
```

Task05：

```text
同一 B-A-C 几何
→ 顶部紧凑吸盘
→ PRE_PLACE -> PLACE 成功
→ BoxA 稳定进入 B/C 中间
→ 自定义 MoveIt 紧凑吸盘模型联合仿真通过
```

最终结论：高密度放置主线采用顶部紧凑吸盘，不再围绕二指夹爪设计补救策略。

## 8. Task06 工程结论

Task06 已完成双臂基础设施：

```text
Isaac 双 FR3 长边布局
+
左右独立 joint_state / joint_command / TF
+
MoveIt 双臂唯一 joint/link 前缀
+
left_arm / right_arm / dual_arm
+
14 关节统一 /joint_states
+
标准 /tf
+
MoveIt Planning Scene 桌面
```

因此 Task06 不再继续做重复的单臂功能验证。

下一步直接进入 Task07：

```text
Left FR3 运行 Task04 primitive(BoxA)
Right FR3 运行 Task04 primitive(BoxB)
→ 两条任务同时启动
```

先用无冲突路线证明真正的并行执行，再在 Task08 主动制造双臂轨迹冲突，进入项目当前最重要的“同时控制 + 双臂避障/冲突检测”阶段。

## 9. 工程目录

项目根目录：

```text
~/lmy/dual-arm-embodied-palletizing
```

ROS 2 工作空间：

```text
~/lmy/dual-arm-embodied-palletizing/ros_ws
```

稳定旧 Task 源码继续保留作为回归基线，不直接覆盖。

新功能优先抽象成可参数化模块，而不是复制一份新的单臂控制器；只有双臂协调层、共享场景层和紧协调层新增独立逻辑。

## 10. 当前原则

```text
工程先于论文结构。
模块复用优先于重复验证。
只验证新增能力，不重复证明稳定模块。
```

当前不提前锁定“第几章写什么”。先把：

```text
固定顶部吸盘
→ 单臂抓放
→ 双臂松协调
→ 双臂顶部双吸盘紧协调
→ 松/紧模式切换
→ 完整混合码垛
```

完整做通。

系统稳定后，再根据最终实际完成的方法、实验结果和创新点组织硕士论文结构。
