# Task09 头脑风暴：松协调冲突消解策略

> 本文件用于记录 Task09 后续可尝试的协调方法、算法想法与实现顺序。
>
> 它不是当前验收规范，也不代表这些方法都必须实现。后续可根据实验结果逐步选择、裁剪和升级。

---

## 1. 当前问题

Task08 已经能够在统一时间轴上，对左右两条完整任务轨迹进行 10 ms 联合采样，并通过 MoveIt / FCL 检测：

```text
ARM_ARM
ARM_OBJECT
OBJECT_OBJECT
```

Task09-A 当前采用整条轨迹的 start delay：

```text
simultaneous candidate -> CONFLICT
-> DELAY_LEFT / DELAY_RIGHT
-> 搜索一个整体启动延迟
-> Task08-B 再验证
```

这种方法实现简单、适合作为基线，但在部分交叉场景中可能退化为：

```text
Left :  完整执行结束
Right:                    再开始完整执行
```

这样虽然安全，但基本失去双臂松协调并行执行的效率优势。

因此后续重点应从：

> “整条任务谁先做、谁后做”

逐渐转向：

> “只对真正发生冲突的时间窗口进行最小必要协调”。

---

# 2. 方法 A：整条轨迹 Start Delay

## 思路

保持两条关节路径完全不变，只对其中一条完整轨迹增加启动偏移：

```text
Left :  |----------------------->
Right:       |----------------------->
             ^ delay
```

形式上可写为：

```text
qR'(t) = qR(t - Δt)
```

Task09-A 已经采用该思路。

## 优点

- 实现最简单；
- 不修改 MoveIt 规划出的空间路径；
- 不破坏 Task04 已验证的抓放事件；
- 可以直接复用 Task08-B 作为最终安全验证器；
- 适合作为后续方法的 baseline。

## 缺点

- 协调粒度太粗；
- 容易退化成完全串行；
- 只利用“启动时间”一个自由度；
- 冲突可能只持续几百毫秒，却导致另一臂等待数秒。

## 定位

保留，不删除。

它适合作为：

```text
Baseline 1: Full-task start delay
```

后续实验可用于和更高级方法比较节拍。

---

# 3. 方法 B：冲突时间窗口内的局部等待

这是当前最值得优先尝试的方法。

## 3.1 核心思想

双臂任务虽然全程同时运行，但真正冲突通常只发生在局部时间段。

例如：

```text
Left :  PICK ---- LIFT ---- TRANSFER -------- PLACE
                             XXXXX

Right:  PICK ---- LIFT ---- TRANSFER -------- PLACE
                             XXXXX
```

其中只有中央公共工作区对应的某一小段发生冲突。

因此没有必要让右臂从 HOME 就开始等待。

可以改成：

```text
Left :  PICK ---- LIFT -------- TRANSFER -------- PLACE

Right:  PICK ---- LIFT ---- WAIT ---- TRANSFER --- PLACE
                         ^
                    只在必要位置等待
```

即：

- 两臂仍可并行完成抓取；
- 两臂仍可并行完成吸附和 LIFT；
- 只有准备进入冲突区域的机械臂暂停；
- 优先臂通过后，让行臂继续执行。

这更符合“松协调”的含义。

---

## 3.2 需要 Task08 提供的信息

Task08 当前已经有：

```text
first_conflict_time
collision pair
conflict type
```

后续建议进一步提取完整冲突窗口：

```text
conflict_start_time
conflict_end_time
```

即：

```text
[t_conflict_start, t_conflict_end]
```

如果一条任务中存在多个互不连续的冲突段，未来可以扩展为：

```text
ConflictWindow[0]
ConflictWindow[1]
...
```

第一版暂时只处理第一个冲突窗口即可。

---

## 3.3 在轨迹中插入 WAIT

假设右臂在时间 `t_wait` 进入公共工作区之前暂停，等待 `Δt`：

```text
qR'(t) = qR(t),                         t < t_wait
qR'(t) = qR(t_wait),                    t_wait <= t < t_wait + Δt
qR'(t) = qR(t - Δt),                    t >= t_wait + Δt
```

直观上：

```text
原轨迹：
A ---- B ---- C ---- D ---- E

修改后：
A ---- B ---- C ---- C ---- C ---- D ---- E
              <--- WAIT --->
```

注意：

`C` 不应该任意选择。

等待点最好满足：

1. 当前姿态本身无碰撞；
2. 不位于双方最狭窄的交叉区域；
3. 尽量在进入共享工作区之前；
4. 携物时保持箱体安全悬空；
5. 不破坏 ATTACH / DETACH 事件语义。

第一版可以优先选择：

```text
LIFT 完成后 / TRANSFER 开始前
```

作为候选等待点。

---

# 4. 方法 C：自动搜索最小等待时间

这是目前最值得保留并优先实现的算法想法。

## 4.1 目标

不是人工指定：

```text
右臂等 2 s
```

而是系统自动搜索：

```text
最小需要等待多久才能安全？
```

设等待时间为：

```text
Δt
```

目标：

```text
minimize Δt
```

约束：

```text
Task08-B(left_candidate, coordinated_right_candidate) == SAFE
```

也可以写成：

```text
Collision(qL(t), qR'(t; Δt)) = false,  for all sampled t
```

---

## 4.2 最简单的搜索算法

第一版完全不需要复杂优化器。

可以枚举：

```text
Δt = 0.00
     0.05
     0.10
     0.15
     ...
```

每次：

```text
1. 在让行臂轨迹中插入 WAIT
2. 重新生成统一时间轴
3. 调用 Task08-B
4. 如果仍 CONFLICT -> 增加 Δt
5. 如果 SAFE -> 返回当前 Δt
```

伪代码：

```cpp
for (double delay = 0.0;
     delay <= max_wait;
     delay += wait_step)
{
    auto candidate = insertLocalWait(
        yielding_arm,
        wait_time,
        delay);

    auto report = detector.check(
        priority_arm,
        candidate,
        sample_period);

    if (!report.conflict)
    {
        return MIN_SAFE_WAIT(delay);
    }
}

return NO_SOLUTION;
```

第一版建议：

```text
wait_step = 0.05 ~ 0.10 s
```

具体值后续再通过实验确定，不作为论文最终参数预先固定。

---

## 4.3 二阶段搜索

如果后续希望更精细，可以先粗搜再细搜。

例如：

```text
阶段 1：0.2 s 步长

找到：
1.0 s -> CONFLICT
1.2 s -> SAFE

阶段 2：在 [1.0, 1.2] 内细搜
步长 0.01 s
```

甚至可以使用二分搜索。

最终得到：

```text
minimum_safe_wait = 1.07 s
```

这种结果非常适合后续实验统计。

---

## 4.4 左右臂都应该尝试

不要固定永远右臂等待。

对于一次冲突，可以分别求：

```text
Δt_left_min
Δt_right_min
```

然后比较：

```text
如果 Δt_left_min < Δt_right_min
    DELAY / WAIT LEFT
否则
    DELAY / WAIT RIGHT
```

进一步还可以加入任务代价：

```text
cost = waiting_time
     + λ1 * predicted_task_delay
     + λ2 * priority_penalty
```

第一版可以只比较最小等待时间。

---

# 5. 方法 D：固定空间路径，只做时间缩放 / Retiming

局部 WAIT 的缺点是机械臂会出现明确的“停顿”。

下一步可以保持 MoveIt 已规划的空间路径：

```text
q(s)
```

不变，只重新设计时间参数：

```text
s(t)
```

例如：

```text
正常速度
   ↓
接近公共区时减速
   ↓
让优先臂先通过
   ↓
恢复正常速度
```

示意：

```text
WAIT 方法：
-------> | STOP | ------->

Retiming：
-------> ---> -> ---> ------->
          gradually slower
```

## 优点

- 比完全停车更平滑；
- 仍然不改变空间路径；
- 可以降低机械冲击；
- 更适合以后加入速度 / 加速度约束；
- 有潜力作为论文中的优化方法。

## 可研究目标

```text
minimize total makespan
```

同时满足：

```text
collision free
|q_dot| <= q_dot_max
|q_ddot| <= q_ddot_max
```

甚至可以进一步增加：

```text
jerk
energy
smoothness
```

这一阶段已经开始接近 QP / MPC / trajectory optimization。

---

# 6. 方法 E：固定 / 动态优先级

时间协调必须回答：

```text
谁先通过公共工作区？
```

第一版可以固定：

```text
Left > Right
```

但后续可升级为动态优先级。

候选因素：

```text
1. 谁距离当前目标更近
2. 谁剩余任务时间更短
3. 谁已经携带箱体
4. 谁更接近公共工作区
5. 谁的最小等待时间更小
6. 箱体优先级
7. 当前码垛层 / 放置顺序约束
```

一种简单策略：

```text
分别计算：

cost_wait_left
cost_wait_right

选择总代价更小的方案。
```

注意：

优先级只负责决定“谁让”，真正安全仍然必须由 Task08-B 再验证。

---

# 7. 方法 F：局部空间路径重规划

如果时间协调代价过大，例如：

```text
minimum_safe_wait > max_acceptable_wait
```

或者：

```text
NO_SOLUTION
```

再考虑改变空间路径。

## 基本思想

```text
Task08
  ↓
CONFLICT
  ↓
时间协调失败 / 等待代价过高
  ↓
选定 priority arm
  ↓
固定 priority arm 的计划
  ↓
对 yielding arm 局部重新规划
  ↓
Task08 再验证
```

---

## 7.1 为什么不能简单把另一臂当前姿态当障碍

另一机械臂是动态障碍：

```text
Obstacle = O(t)
```

而普通 MoveIt / OMPL 更擅长：

```text
Obstacle = O
```

即静态障碍。

如果只把左臂某一个时刻的姿态加入 Planning Scene：

```text
Right 绕开 left(t = 2.5 s)
```

但右臂真正到达该区域时，左臂可能已经移动到别处。

所以这种方法虽然能工作，但比较保守，也不是真正的时空联合规划。

---

## 7.2 一个工程上可行的折中：Swept Volume / 保守占用区

可以提取优先臂在冲突窗口：

```text
[t_start, t_end]
```

内经过的所有姿态。

把这段运动扫过的空间近似成一个保守占用区域：

```text
priority arm swept volume
```

然后把这个区域作为临时静态障碍，让 MoveIt 对另一机械臂重新规划。

流程：

```text
ConflictReport
    ↓
冲突时间窗口
    ↓
priority arm 冲突段采样
    ↓
构建保守空间占用
    ↓
MoveIt / OMPL replan yielding arm
    ↓
Task08-B 联合时空复检
```

即使重规划使用了保守近似，最终安全性仍由 Task08-B 统一判断。

---

# 8. 方法 G：真正的时空联合规划

更长期可以研究真正的：

```text
space + time planning
```

不再先生成两条独立路径再协调，而是直接在联合状态 / 时间空间中搜索。

例如状态可以抽象为：

```text
(qL, qR, t)
```

或者：

```text
(sL, sR)
```

其中 `sL / sR` 是左右轨迹进度。

这种方法理论上最完整，但：

- 状态空间明显增大；
- 和现有 Task04 / Task08 架构差异较大；
- 工程实现复杂；
- 当前阶段不建议优先实现。

先记录为长期研究方向。

---

# 9. 推荐的实现顺序

当前建议按下面顺序推进，而不是直接做复杂联合重规划。

```text
Task09-A
Full-task Start Delay
        ↓
已有 baseline
        ↓
Task09-B
Conflict-window Local Wait
        ↓
自动搜索 minimum safe wait
        ↓
Task09-C
Trajectory Retiming / Time Scaling
        ↓
时间协调仍失败？
     /        \
   No          Yes
   ↓            ↓
 执行      Local Spatial Replan
                 ↓
            Task08-B 再验证
```

推荐优先级：

```text
1. 保留 Task09-A 整体 start delay 作为 baseline
2. 增加完整 conflict window 提取
3. 实现局部 WAIT
4. 实现 minimum safe wait 自动搜索
5. 比较 WAIT LEFT / WAIT RIGHT
6. 做任务总时间 / 等待时间实验
7. 再考虑 retiming
8. 最后再考虑空间局部重规划
```

---

# 10. 建议新增的数据结构

未来可以在 `ConflictReport` 基础上逐渐增加：

```cpp
struct ConflictWindow
{
    double start_time_sec;
    double end_time_sec;
    std::string type;
    std::string body_a;
    std::string body_b;
};
```

协调结果可以单独定义：

```cpp
struct CoordinationResult
{
    bool valid;
    bool safe;

    std::string strategy;
    // START_DELAY / LOCAL_WAIT / RETIMING / REPLAN

    std::string yielding_arm;
    // LEFT / RIGHT

    double wait_start_time_sec;
    double wait_duration_sec;

    double original_makespan_sec;
    double coordinated_makespan_sec;

    ConflictReport verification_report;
};
```

这样 Task09 后续换算法时，不需要破坏 Task08 的检测接口。

---

# 11. 建议实验指标

后续比较协调策略时，不应该只统计：

```text
SAFE / CONFLICT
```

还建议记录：

```text
1. 原始并行 makespan
2. 协调后 makespan
3. 额外等待时间
4. 相比完全串行节省的时间
5. 相比 Task09-A full delay 节省的时间
6. detector 调用次数
7. 协调算法 wall time
8. 是否发生 replan
9. replan 成功率
10. 最小安全间距（若后续稳定支持 distance query）
```

例如：

```text
T_parallel_ideal
T_serial
T_start_delay
T_local_wait
T_retiming
```

可以定义效率损失：

```text
coordination overhead = T_method - T_parallel_ideal
```

也可以比较并行收益：

```text
parallel gain = T_serial - T_method
```

这些指标以后很适合直接进入论文实验部分。

---

# 12. 当前最推荐的下一版算法

如果只选择一个方法继续实现：

> **Conflict-window Local Wait + Automatic Minimum Safe Wait Search**

原因：

```text
实现难度        中低
对已有代码改动  小
复用 Task08      很充分
可解释性        强
工程可行性      高
论文表达        清晰
相比完全串行    有直接效率收益
```

核心流程可以固定为：

```text
Left candidate + Right candidate
            ↓
         Task08-B
            ↓
        CONFLICT
            ↓
提取 first conflict / conflict window
            ↓
确定候选等待点
            ↓
分别尝试 WAIT_LEFT / WAIT_RIGHT
            ↓
自动搜索最小 Δt
            ↓
每个候选重新调用 Task08-B
            ↓
选择最小代价 SAFE 方案
            ↓
      CoordinationResult
```

整个 Task09 的设计原则可以概括为：

> **检测保持统一，协调逐渐升级；任何新协调策略产生的候选轨迹，最终都重新交给 Task08-B 的 MoveIt / FCL 时空检测器做安全验收。**

这样可以避免每增加一种协调算法，就重新发明一套碰撞安全逻辑。

---

# 13. 后续可能的论文演化路线

项目工程版本：

```text
FCL 时空检测
+
最小局部等待
```

进一步：

```text
FCL 时空检测
+
动态优先级
+
时间缩放
```

再进一步：

```text
FCL / distance prediction
+
QP / trajectory optimization
```

长期：

```text
联合时空规划
或
MPC 双臂在线协调
```

因此目前不需要一次把所有方法都实现。先把“全局等待”升级成“局部最小等待”，就已经能形成一条清晰、可验证、可继续扩展的松协调技术路线。
