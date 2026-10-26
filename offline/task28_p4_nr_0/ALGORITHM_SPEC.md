# G1-P4-NR-0：实现前算法规格

规格在新 planner/projection/validation 实现前保存。来源详见 evidence/p4_correspondence.json/md；目标是 P4-inspired object-conditioned NR-RRT，不声称原KDL或ABB复现的数值轨迹。

- 状态为同一Cube_01的 X=T_WO∈SE(3)、q=(qL,qR)∈R14、离线接触阶段。保持G1-1固定X→左右grasp变换。全链自由闭链残差为6维；给定X时堆叠两侧位置与SO(3) log残差为12维，14关节保留2个冗余自由度。禁止PCS固定主动臂或解析branch替代。
- 采样为确定性Generator(seed)控制的物体translation和小SO(3)旋转扰动，含显式goal bias。桌面支持只在冻结supply/target footprint成立。使用垂直lift/placement连接原语满足原接触合同，transport可搜索三维/旋转；lift高度与transport分支由搜索产生，不读取旧A-stage waypoint或预先求完路径。
- NR重构使用全链Jacobian/SVD Moore–Penrose方向，q←q−J†r。米/弧度与原P4单位不同；采用现有G1-1严格IK复核阈值，残差必须再经FK/闭链验证。为了有限FR3范围，可使用有界步长/回溯作为显式工程适配，不裁剪关节、不得扩大limits。冷启动和冗余种子只从冻结HOME、限位中点及固定随机生成器产生；局部重构优先用邻近已接受节点。
- 树扩展为nearest→有限range object steer→NR→单状态门禁→局部连接；候选只能从实际已接受父边回溯。局部连接继承P4投影中点RBS思想：物体geodesic中点与平均joint seed投影并递归，直到规定最大对象/关节距离，然后调用原G1-2B-0有限采样门禁；预算耗尽或失败拒绝，不能endpoint-only。
- 全部新状态走G1-1完整FK/限位/闭链以及含两臂、工具、Cube、table的原414-pair FCL，通过G1-2A/R1几何接触合同。供料支持→lift→transport→place→目标支持由原R1显式三帧合成契约检查；规划点和插值点不能冒充实际测量，细分点不贡献确认。
- 对每一完整候选独立调用G1-2B-0各段与G1-2B-1(max_depth=0的有界首次调查)。joint_limits/closed_chain/collision/combined原输出单独保留，不升级未决。若连续证明未获证，只标OFFLINE_CANDIDATE_UNCERTIFIED，execution_authorized=false。
- 冻结尝试、投影迭代、重启、节点数、连接深度与采样总数；固定至少3个planning seeds。终止为候选全部离线sampled门禁通过，或者明确budget/precondition/rejection失败。时间只为CPU wall测量，不以超时后不保存失败。
- 不改变fixture、机器人、原验证器、ACM、吸附/closure/关节阈值。不引入新collision backend，不执行Isaac/ROS/MoveIt/机器人，不实现PCS/RSS/QP/force/RL。连续证明不作无限细化。

确切预算与工作空间将在 run manifest 于正式数值实验前冻结；开发控制和正式seed运行分开记录。
