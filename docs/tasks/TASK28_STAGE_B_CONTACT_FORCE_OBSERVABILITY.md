# TASK28 Stage B：接触力可观测性探针

## 结论：PASS（隔离仿真探针）；B0_PARTIAL 不变

在 Isaac Sim 4.5 的原 Task27 场景中，原有 Cube_01 与一枚按原吸杯碰撞几何复制的独立刚体完成了 **+Y 接近、接触、小载荷和退回**。该探针测得成对 PhysX 接触反力、独立接触报告与 Cube 动量变化相互一致；这不是机器人执行、完整侧压、传感器标定或接触力闭环控制的证明。

## 接口与物理含义

主测量为 Isaac Sim 4.5 `isaacsim.core.api.sensors.rigid_contact_view.RigidContactView`：传感体为 `/World/Task27/Supply/Cube_01`，filter 为 `/World/Task28ContactProbe/Cup`，读取 `get_contact_force_matrix(dt=physics_dt)`。源码表明底层为 PhysX 接触冲量，传入实际物理步长后换算成牛顿；结果是 **作用于 Cube 的工具/Cube 成对法向接触力**。在本实验里用几何法向、另一条报告路径和动量检查确认了世界坐标及符号。

第二通道 `omni.physx.get_physx_simulation_interface().subscribe_contact_report_events` 返回碰撞体路径、接触点、法向及世界坐标冲量。本次 collider0 是 Cup，其冲量方向为工具受力 −Y；Cube 受反作用为 +Y。它与矩阵属于同一 PhysX 求解器的不同读取接口，**不是独立物理引擎**。可见的零冲量接触报告可能只是碰撞检测 contact offset 内的候选点，故本报告以力模大于 `1e-5 N` 定义观察到有载接触；此数值是本探针分析阈值，不是标定阈值。

Cube/table 或 Cube/wall 反力需要以相应碰撞体路径另行筛选；它们不是本矩阵的工具/Cube 结果。本次 Cube 在桌下供料区，记录到的 Cube 碰撞报告只含 Cup 对，未记录桌面或墙接触。关节执行力矩是电机命令/反馈量，不能等同于接触力；Surface Gripper 吸附约束反力是另一种约束反力，本探针未建立吸附，也没有测量该量。

## 装置与实验

- 不更改场景文件或既有 Cube 几何：原有 Cube_01 位于 `(-0.6, -0.3, -5.0) m`，质量 `0.8 kg`，供料区原有关闭重力。为了动量核对，本次临时场景将 Cube 线性阻尼设为 0。未对 Cube 写位置、姿态或速度。
- 新建一个独立刚体，只复制原吸杯圆柱尺寸：半径 `0.010 m`、长度 `0.010 m`，轴向 Y；初始与 Cube −Y 面相距 6 mm，质量 `0.2 kg`，关闭重力。这是单杯替身，不含其余三杯、支架、FR3 或吸附约束的动力学。
- 施力只作用于 Cup 中心，世界坐标 +Y 为接近方向；位置/速度弹性驱动力限幅 **±1 N**。若测得接触力超过 5 N、任一物体 Y 位移超过 50 mm 或速度超过 0.15 m/s 则停止。测得驱动力最大绝对值 0.489 N；它不是接触力。
- 初始分离 0.5 s；按 6 mm/s 的目标接近；`1.7–2.1 s` 以 Cube 相对位置跟随进行有界加载；随后 Cup 退回，至 `3.5 s` 结束。目标偏差用于弹性驱动，并非写入刚体位姿或设定实际穿透。动力学步骤约 `1/60 s`。时间为本次采样起点后的累计仿真时间，另记录墙钟纳秒。
- 步后回调逐步读取两刚体的 PhysX `dynamic_control.get_rigid_body_pose`/`get_rigid_body_linear_velocity` 和按当步 dt 换算的成对力。对应接触报告在同一步步后快照之前到达：日志中报告的 `step_at_callback + 1` 对应快照 `step`，无跨步挪用。

主记录： [v4 逐物理步力、位姿及速度](../../offline/task28_stage_b_contact/evidence/task28_force_observability_v4/timeseries.jsonl)、[碰撞体路径和原始点冲量](../../offline/task28_stage_b_contact/evidence/task28_force_observability_v4/contact_events.jsonl)、[核对结果](../../offline/task28_stage_b_contact/evidence/task28_force_observability_v4/analysis.json)。文件包含每步时间、阶段、驱动力命令、接触状态及数据；传感器是模拟地面真值，非现实测量。

## 观测结果

| 指标 | v4 实测 |
| --- | --- |
| 总样本 | 210 个物理步；3.500 s |
| 初始分离 / 退回后成对力最大值 | 0 / 0 N，分别 29 / 25 个样本；仅说明本次仿真无测得噪声 |
| 首次 / 末次有载接触 | 第 97 / 126 步，1.617 / 2.100 s |
| 峰值作用于 Cube 的力 | `[1.54e-9, 0.056391, -9.45e-12] N`，世界坐标 |
| LOAD 24 步平均力 | `[-1.42e-8, 0.027965, -1.84e-8] N`；24/24 步 +Y 有载 |
| Cup 最大前进 / 最终位置相对初始 | +10.893 / −4.981 mm（Y） |
| Cube 最终位移 | +9.770 mm（Y） |
| 首次至末次接触的力积分 / Cube 动量增量 | `0.014631003 / 0.014631008 N·s`，差约 `4.79e-9 N·s` |
| 接触报告与成对矩阵逐步冲量最大差 | `4.36e-11 N·s`，按碰撞体顺序翻转受力符号 |

第 126 步被标记 `RETRACT`，该行记录的是**刚完成的**物理步的接触力及**下一步**的退回命令；第 127 步成对力为零。末次接触后 Cube 的速度在第 143 步从约 0.0183 m/s 突然归零；当步未记录其他 Cube 接触，原因尚未确认，因此动量核对只覆盖到末次有载接触，不把后续运动当作已解释的动力学证据。

首轮 [v1](../../offline/task28_stage_b_contact/evidence/task28_force_observability_v1/summary.json) 固定工具目标，Cube 脱离后惯性前行，LOAD 60 步均无接触，故不算持续加载成功。第二轮 [v2](../../offline/task28_stage_b_contact/evidence/task28_force_observability_v2/summary.json) 改为短时相对跟随并获得 24/24 有载步；v3 使用相同物理命令，增加对所有 Cube 碰撞报告的记录，结果复现，未发现其他 Cube 碰撞。v1–v3 的工具施力实际按工具局部坐标施加，但日志字段误标世界坐标且未记录姿态，因此这些轮次的命令方向不能据此独立确认；成对力与动量读数仍是世界坐标物理观测。v4 改用显式世界坐标中心施力并记录双方姿态，所得结论以 v4 为准。各轮数据独立保存，未覆盖既有 B0/Stage A 证据。

## 复现与下一步

```bash
DISPLAY=:1 OMNI_KIT_ACCEPT_EULA=YES ROS_DOMAIN_ID=153 \
  /home/ubuntu2004/isaacsim-4.5.0/python.sh \
  isaac/scripts/task28_contact_force_probe.py --run-id NEW_UNIQUE_ID
python3 offline/task28_stage_b_contact/analyze_force_probe.py \
  offline/task28_stage_b_contact/evidence/NEW_UNIQUE_ID
PYTHONPATH=. python3 -m pytest -q offline/task28_stage_b_contact/test_force_probe.py
```

分析器检查分离/加载/退回、力方向、碰撞对、步号关联以及末次接触前动量；测试包含载荷丢失、反向力、无关碰撞和乱序的负对照。该探针未评估多杯载荷分配、旋转、桌面摩擦、墙接触、FR3 驱动或实际 B0 在线时延；不能据此给出经过标定的测量误差范围。

下一步应在有桌面支撑且保留完整工具碰撞体的场景中验证单次有界 CONTACT → FORCE_RAMP → EXIT，包括成对力反馈、接触丢失与过力退出，再逐步接入 FR3；这需要进一步验证，不代表已经实现混合力/位置控制。
