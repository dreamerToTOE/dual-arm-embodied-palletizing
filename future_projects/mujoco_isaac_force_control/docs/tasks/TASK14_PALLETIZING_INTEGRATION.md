# Task14：与双机械臂码垛项目集成（可选最终阶段）

## 定位

这一阶段不是本力控项目完成的必要条件。只有 Task00--13 已稳定，并且原码垛项目的 FCL / 随机多尺寸 / placement 主线成熟后再做。

## 目标

保留原码垛项目的高层结构：

```text
perception / GT
→ placement planner
→ loose/tight router
→ MoveIt / task trajectory reference
```

仅在 TIGHT 紧协调阶段，将原来的纯位置同步执行替换/增强为：

```text
reference object trajectory
        ↓
dual-arm impedance / force controller
        ↓
wrench allocation + internal-force control
        ↓
tau_L / tau_R
        ↓
Isaac execution
```

## 第一版边界

- 松协调小件仍使用现有经典轨迹执行；
- 只对共同大箱体启用力控；
- MoveIt/FCL 仍负责高层几何与安全参考；
- 力控不负责重新做路径规划。

## 研究问题

可以比较：

```text
原位置同步 baseline
vs
Cartesian impedance
vs
QP wrench allocation + internal force
```

扰动实验：

```text
payload mass change
one-arm disturbance
trajectory timing mismatch
contact stiffness change
```

## 验收

- 大箱共同搬运成功；
- 物体跟踪误差、relative TCP error、wrench、joint torque 都有记录；
- 力控相对位置 baseline 的收益必须用数据证明；
- 若无显著收益，不强行把复杂控制并入最终码垛主线。
