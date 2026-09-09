# Task08：松协调双臂时空冲突检测

## 状态

🟡 **Task08-A 完整候选已实现；交叉场景已完成，Task08-B 冲突检测器待实现。**

## 1. 本 Task 只解决什么

Task07 已经证明：同一个 Task04 `PalletizePrimitive` 的左右实例可以真正并发完成两个不同箱体的抓放。

Task08 不再重复验证单臂抓放、吸盘、桌面避障、Planning Scene、左右通信隔离等能力。

Task08 唯一新增目标：

```text
Left candidate trajectory
+
Right candidate trajectory
+
两条轨迹的时间参数
+
双臂完整碰撞模型（含携带箱体）
        ↓
预测“同时执行”时是否发生冲突
        ↓
ConflictReport
```

本 Task **只检测，不解决**。等待、优先级、延迟启动、局部重规划明确放到 Task09。

---

## 2. 为什么 MoveIt 单臂规划成功仍然可能发生双臂冲突

`left_arm` 和 `right_arm` 分别规划时，MoveIt 可以看到另一台机械臂当前的静态姿态，因此每条轨迹单独都可能是合法的。

但如果两条轨迹同时执行：

```text
左臂把“右臂当前姿态”当静态障碍
右臂把“左臂当前姿态”当静态障碍

实际执行时两者都在移动
→ 单独合法 != 同时合法
```

因此 Task08 必须把两条轨迹放到同一时间轴上，再用完整 14-DOF 双臂状态进行联合碰撞检查。

---

## 3. 继续复用的稳定模块

不改 Task04 canonical primitive 的内部事件逻辑：

```text
PRE_PICK
→ CONTACT
→ SUCTION ON
→ ATTACH
→ LIFT
→ TRANSFER / PRE_PLACE
→ PLACE
→ 先规划 RETREAT
→ SUCTION OFF
→ settle
→ GT 回写
→ RETREAT
```

Task08-A 不再把空载与携物段拆成单独的检测输入，而是将 canonical primitive 的全流程拼接为一个 `TaskTrajectoryCandidate`：

```text
HOME -> PRE_PICK -> CONTACT -> ATTACH -> LIFT
-> PRE_PLACE -> PLACE -> DETACH -> RETREAT
```

候选包含完整 `JointTrajectory`，并以绝对相对时间记录 `ATTACH` / `DETACH`。因此 Task08-B 可以在每个采样时刻判断 Box 是 World object、Attached object 还是已释放物，而不是假定整段轨迹始终携带 Box。

---

## 4. Task08 场景设计

Task07 的分离通道保留为 SAFE 对照：

```text
Left  y ≈ -0.25
Right y ≈ +0.25
```

Task08 新增 `conflict` 场景，把两个完整任务主动引入中央公共工作区，使候选中的携物搬运段交叉或高度重叠。

当前交叉场景：

```text
BoxA pick   = (0.320, -0.250, 0.065)
BoxA target = (0.820, +0.120, 0.065)

BoxB pick   = (0.320, +0.250, 0.065)
BoxB target = (0.820, -0.120, 0.065)
```

两条名义 XY 路径交叉位置约：

```text
(0.658, 0.000)
```

最终标准仍然不是某组固定坐标，而是必须满足：

```text
左轨迹单独规划：PASS
右轨迹单独规划：PASS
两条轨迹同时预测：CONFLICT
```

这样证明的是真正“双臂动态冲突”，而不是普通静态障碍碰撞。

---

## 5. 新模块：SpatioTemporalConflictDetector

计划新增到：

```text
ros_ws/src/fr3_dual_palletize/
```

核心接口概念：

```cpp
struct TimedTrajectory
{
    trajectory_msgs::msg::JointTrajectory trajectory;
    double start_delay;
    CarryState carry_state;
};

struct ConflictEvent
{
    double time;
    std::string type;
    std::string body_a;
    std::string body_b;
};

struct ConflictReport
{
    bool conflict;
    double first_conflict_time;
    std::vector<ConflictEvent> events;
};

class SpatioTemporalConflictDetector
{
public:
    ConflictReport check(
        const TimedTrajectory& left,
        const TimedTrajectory& right);
};
```

Task09 直接消费 `ConflictReport`，不再重新做碰撞分析。

---

## 6. 时间轴与轨迹插值

Task07 的 Isaac 关节命令执行器按约 10 ms 周期插值和发布关节位置。

Task08 第一版直接复用同样的轨迹插值语义：

```text
Δt = 0.01 s
```

在统一时间轴：

```text
t = 0, 0.01, 0.02, ... max(T_left, T_right)
```

每个采样时刻：

```text
qL(t) = 左轨迹分段线性插值
qR(t) = 右轨迹分段线性插值
```

若一条轨迹先结束，则保持其最终姿态直到另一条结束。

第一版不额外引入连续碰撞检测算法；预测器与实际 100 Hz 关节命令时间分辨率保持一致。后续只有在实验中出现采样漏检风险时才升级连续检测或自适应细分。

---

## 7. 联合 RobotState

每个时间采样点创建同一个双臂 RobotState：

```text
left_fr3_joint1..7  <- qL(t)
right_fr3_joint1..7 <- qR(t)
```

然后在 Task06 的完整双臂 Planning Scene 中检查碰撞。

不能分别检查两个单臂状态后再合并结果，因为 Task08 的重点正是“两个同时运动的机器人之间”的碰撞。

---

## 8. 携带箱体必须进入预测模型

Task08 不只检查裸机械臂。

`ATTACH` 到 `DETACH` 之间，BoxA / BoxB 已经被吸盘抓住，因此预测状态必须复用 Task04/07 的 AttachedCollisionObject 几何和相对位姿，而不是把箱子继续当 world 静态物体。两个事件之外，检测器使用候选记录的初始 / 计划释放 pose 将 Box 作为 world object 处理。

检测对象包括：

```text
ARM_ARM
  left robot <-> right robot

ARM_OBJECT
  left robot <-> right carried BoxB
  right robot <-> left carried BoxA

OBJECT_OBJECT
  carried BoxA <-> carried BoxB
```

各自机械臂内部 self-collision、机械臂与桌面碰撞仍由原有 MoveIt 单臂规划负责；若这些出现则视为上游规划失败，不作为 Task08 的研究重点重复统计。

吸盘与自己携带物的 intentional contact 继续使用现有 touch_links 语义忽略。

---

## 9. CollisionReport 输出

Task08 运行时至少输出：

```text
TRAJECTORY_LEFT_DURATION
TRAJECTORY_RIGHT_DURATION
SAMPLES_CHECKED

SAFE
```

或：

```text
CONFLICT
first_time = 2.31 s
pair = left_fr3_link6 <-> right_fr3_link5
type = ARM_ARM
```

若同时存在多个冲突 pair，可以记录多个 event，但协调层首先使用 `first_conflict_time`。

另外记录 detector wall time，作为以后 Task18 实验数据候选；Task08 当前不设固定性能指标。

---

## 10. 安全距离

第一版核心验收以 MoveIt/FCL 的真实几何碰撞为准。

同时保留可配置：

```text
safety_margin
```

用于后续把“尚未接触但已经过近”定义为 `NEAR_CONFLICT`。开发阶段可先使用一个保守小余量，但不把该值写成论文最终参数；Task18 再根据实验统一确定。

如果当前 MoveIt Humble 接口获取最小距离足够稳定，则 Task08 同步实现 `NEAR_CONFLICT`；如果接口兼容性增加无谓复杂度，则先完成确定性的 exact collision，安全距离扩展不阻塞 Task08 收口。

---

## 11. Task08 执行逻辑

```text
Left  完整 TaskTrajectoryCandidate
Right 完整 TaskTrajectoryCandidate
             ↓
分别单独规划 PASS，且不下发 Isaac
             ↓
统一时间轴上的 ATTACH / DETACH 状态切换
             ↓
SpatioTemporalConflictDetector
             ↓
      ┌──────┴──────┐
    SAFE          CONFLICT
      │               │
允许并发执行      禁止危险并发执行
                  只输出报告
```

Task08 不在检测到冲突后自行决定谁让行。

在 conflict 验收场景中，检测到冲突后不执行危险的同时 transfer；Isaac 可直接 reset 场景，不在本 Task 额外实现恢复动作。

---

## 12. Task08 只验收这些新增能力

必须满足：

```text
1. Task07 SAFE 通道经过 detector -> NO_CONFLICT
2. Task08 conflict 通道：left candidate 单独规划 PASS
3. Task08 conflict 通道：right candidate 单独规划 PASS
4. 两条 candidate 按同一 t0 同时预测 -> CONFLICT
5. 输出 first_conflict_time + collision pair + conflict type
6. 冲突被发现后，危险的双臂 transfer 不下发到 Isaac
7. ATTACH / DETACH 之间的 carried BoxA / BoxB 纳入碰撞预测
```

不重复测试：

```text
单臂吸盘能否抓住
单臂 MoveIt 是否避桌
单臂 AttachedCollisionObject 是否工作
左右 topic / TF 是否隔离
Task07 并行执行是否成立
```

---

## 13. 与 Task09 的边界

Task08 输出：

```text
ConflictReport
```

Task09 输入该报告后才实现：

```text
方案 A：start delay / 等待
方案 B：固定或动态优先级让行
方案 C：时间重排
方案 D：必要时对其中一臂局部重规划
```

因此 Task08 的完成标准是：

> **系统已经能在真正执行前准确指出“两条单独合法的轨迹为什么不能同时执行”。**

而不是已经会主动绕开。

---

## 14. 启动 / 复现指令

### 1. Isaac Sim：恢复 Task06 双臂基础设施

Timeline **Stop** 时依次运行：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task06_dual_fr3_scene.py").read())
```

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task06_dual_ros_graph.py").read())
```

### 2. Isaac Sim：加载 Task08 交叉场景

仍保持 **Stop**：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task08_cross_conflict_scene.py").read())
```

点击 **Play** 后继续复用 Task07 双吸盘 Bridge：

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

### 4. Task08 ROS 主节点

当前 `SpatioTemporalConflictDetector` 与协调接口仍在实现中，因此暂时没有最终 `ros2 run` 入口。实现完成后把最终命令补在这里和 `docs/STARTUP_GUIDE.md`。

不要运行：

```bash
ros2 run fr3_dual_palletize task07_parallel_demo
```

因为 Task07 demo 内部仍使用 Task07 无冲突目标坐标。
