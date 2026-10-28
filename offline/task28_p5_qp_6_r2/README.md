# QP-6-R2：原场景保留审计及禁用的 torque 接口原型

总状态 `QP6_R2_SCENE_VERIFIED_CONTROL_BLOCKED`。完整中文报告见
`docs/tasks/TASK28_G2_P5_QP_6_R2_SCENE_PRESERVING_L_SUCTION_TORQUE.md`。

- `scene_identity.py`：CPU 合同、原始 USD 身份和来源绑定；运行 `python3 -m offline.task28_p5_qp_6_r2.scene_identity --output <new-path>`。
- `runtime_probe.py`：在新进程、新匿名 stage、停止时间线下调用冻结 scene builder，仅读取原始几何和属性。无 physics step、机器人命令、桥接、gripper 初始化或闭合。仅用于重新执行获授权的独立只读场景核对。
- `targets.py`：调用冻结 G1 nominal grasp、closure 逻辑，纯 CPU。
- `torque_adapter.py`：数据结构和力矩消息原型；默认禁用，激活及 effort 计算一律拒绝。无 Isaac actuator sink、无运行中的 HQP，因此不能用于控制机器人。
- `suction_interface.py`：保留原合同的 CPU 状态接口设计，拒绝闭合，未连接 Surface Gripper。
- `evidence/runtime_scene_v2.json`：停止的实际新 stage 中读取的身份和变换；`initial_usd_closure.json`：独立使用冻结 G1 算出的 authored 初始几何。
- `evidence/regressions/`：历史 959 项的重跑记录及首次缺 OSQP 的失败日志。

停止的 USD authored 变换不能充当物理 q/dq、TCP、Cube 接触状态或动力学样本。60 Hz 是实际构建的场景属性，未测量已经推进的物理 dt。工具质量/惯量是否进入 Isaac articulation 的数值动力学仍为 UNKNOWN。R1 的裸臂 source 拒绝是必要背景，不是原场景自己的 source 实测。
