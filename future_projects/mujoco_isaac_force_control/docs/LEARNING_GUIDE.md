# 学习路线与阶段门禁

本文件用于保证项目按教学顺序推进。**不要因为后面的 QP / 双臂更“像研究”，就跳过前面的控制基础。**

## Phase 0：认识力矩接口

包含：Task00--02

必须学会：

```text
q / qdot / tau
PD
重力影响
控制频率
日志与曲线
```

进入下一阶段前，应该能独立解释：

> 为什么一个 joint torque 会让机器人加速？为什么 Kp/Kd 会影响振荡和稳态？

---

## Phase 1：从关节空间到笛卡尔空间

包含：Task03--05

必须学会：

```text
M(q), C(q,qdot), g(q)
Jacobian
J^T wrench mapping
Cartesian impedance
```

进入下一阶段前，应该能独立解释：

> `xdot = J qdot` 和 `tau = J^T W` 分别解决什么问题？

---

## Phase 2：真正接触环境

包含：Task06--07

必须学会：

```text
contact force
force reference
position/force hybrid control
stiffness / damping / contact oscillation
```

进入下一阶段前，应该能独立完成：

> 让 FR3 保持指定法向力压住平面，同时沿切向移动。

---

## Phase 3：双臂共同物体

包含：Task08

必须学会：

```text
object frame
grasp frame
relative TCP geometry
左右 wrench
双控制器互相作用
```

进入下一阶段前，不要求最优力分配，只要求共同搬运 baseline 稳定且所有受力量可观测。

---

## Phase 4：双臂力/力矩分配

包含：Task09--10

必须学会：

```text
object wrench
grasp matrix G
QP variable / objective / constraint
joint torque limit
internal force
null space
```

进入下一阶段前，应该能独立解释：

> 为什么满足 `Gf = W_object` 的左右力通常不唯一？为什么这个自由度可以用于负载分配和内力控制？

---

## Phase 5：鲁棒性

包含：Task11

目标：证明算法不是只对一个标称参数有效。

---

## Phase 6：跨仿真器

包含：Task12--13

目标：Controller Core 不改，只更换 Adapter，在 Isaac 中复现。

---

## Phase 7：系统集成（可选）

包含：Task14

只有当前双臂码垛项目的经典主线也成熟后，再把力控接入紧协调。

## 每个 Task 的学习习惯

每一项按以下顺序做：

```text
1. 先解释物理意义
2. 写最小公式
3. 做一个最小实验
4. 画曲线
5. 改一个参数并预测结果
6. 再运行验证预测
7. 写结论
```

如果只能让代码跑起来但无法预测“增大 Kp / 改变质量 / 改变接触刚度后会发生什么”，则该 Task 不算真正完成。
