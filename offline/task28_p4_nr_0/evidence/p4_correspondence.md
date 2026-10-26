# P4 NR 方法对应只读审计

本审计只读取本地 P4 源码及已保存复现报告，没有执行 P4 数值、编译、随机搜索或仿真。完整来源 SHA256、行号和差异见同目录机器证据。历史 ABB 结果只能作为算法来源证据，不能成为新双 FR3 fixture 的成功证据。

## 真实 NR 是什么

作者源 `CKCplanning/proj_classes/kdl_class.cpp:68–139` 的 `GD` 实际调用 KDL `ChainIkSolverVel_pinv` 与 `ChainIkSolverPos_NR(...,10000,1e-5)`，不是仅凭函数名推断的梯度下降。它把双 ABB 机械臂和 rod 串成 12 关节闭链反链，普通 NR 允许两臂全部关节变化。

已独立合格的 P4 复现把物理残差和 Jacobian 重建为：

`T_target = T_left * G_rod * diag(-1,-1,1,1)`

`g(q) = [p_right - p_target; Log(R_right R_target^T)]`

`J = U diag(s) V^T`

`q_next = q - V diag(1/s_i if s_i > 1e-12*s_max else 0) U^T g`

对应 `physical_nr.py:109–136,171–212` 和 `physical_nr_abb_cpp_draft.hpp:193–215,257–340`。更新无阻尼、无 line search、无步缩放；旋转 Jacobian 包含离流形的 SO(3) inverse left Jacobian。收敛门分别检查位置绝对最大分量 `<=1e-5 mm` 和旋转矩阵元素误差 `<=1e-5`，收敛后做周期图、重新检查物理闭合及原关节限位。

该复现明确标为 `RECONSTRUCTED_PHYSICAL_FULL_NR_CPP`。相较原 KDL，残差图、奇异值 cutoff、单位、周期图及停止门不同，不能声称逐位复现原 KDL 迭代、投影映射或速度。

## 搜索与局部连接中真正可转移的部分

作者 `RRT_NR.cpp:146–212` 采用 goal-biased ambient joint-space sampling、nearest、range 截步、NR 投影，再调用 RBS 局部连接，通过后才添加树节点和 parent。原 goal bias 为 `.05`。达到预先合格 goal 的特殊分支可跳过顶层投影；新实现无需复制这项捷径。

`StateValidityCheckerNR.cpp:149–175` 的局部连接取关节算术中点，对中点 NR 投影并检查碰撞，然后递归两半。原 full-joint L2 叶距阈值 `.05 rad`，最大深度 `150`（header:189–190）。这是投影局部连接，不是独立线性插值天然保持闭链的假设。原实现已保存完整回溯以重新检查实际中间节点。

因此本阶段可称为 **P4-inspired NR** 的实质关系应至少包括：显式 full-SE(3) 残差与 Jacobian、全关节 SVD NR 投影、由运行中的有界树搜索产生节点，以及投影/重构中间状态的局部连接。仅 generic RRT 加现有独立 IK、或预生成路径之后命名，不足以建立该关系。

## 双 FR3 和单 Cube 的必要适配

- 旧 `q∈R12` 的 6 行相对闭链约束一般留下 6 维流形；新双 FR3 `q∈R14` 一般留下 8 维，其中物体 6DOF 和两臂各 1 冗余。不得套用 ABB 六轴解析 branch 或 PCS 固定主动臂门。
- 使用完整冻结 `T_world_object=X`、`G_left=T_object_leftTCP`、`G_right=T_object_rightTCP`。两臂目标分别是 `X G_left` 和 `X G_right`，替换 rod+flip。
- 按本任务物体级 SE(3) 采样要求，固定采样 X 后可堆叠两臂目标残差成 `g_X(q)∈R12`，对 `J_X∈R12×14` 作同样 SVD NR；两臂均为变量，最小范数增量保留冗余连续性。物体采样替代原 ambient joint sampler 是明确工程适配。
- FK/J 必须来自冻结 FR3/TCP/world-base 模型；米制与冻结 G1-1 容差应明确登记。不得把原毫米/旋转矩阵停止门直接移植，也不得放松新门。
- 若采用有限差分 Jacobian、更低迭代/递归预算、每步限位拒绝、信赖域、偏置采样、不同 nearest metric 或停止规则，应逐项写成工程适配，不能混称作者原数值轨迹。
- 原碰撞策略与模型不迁入本 fixture；复用现有 Task28 FCL、保守几何、工具/载荷、ACM 和接触例外。接受节点/边仍经 G1-1、G1-2A-R1、B0、B1 独立门；原 RBS 或新有限采样均不提供连续证书。

本地 P4 的 `baselines/p4_closed_chain` 是较早 14DOF 工程实现。P4 自身源码审计把它列为 `REFERENCE_ONLY`，因为每步限位拒绝、满秩拒绝、停滞和单位策略不同。它可作工程参考，不可取代上述作者/重建方法来源证明。
