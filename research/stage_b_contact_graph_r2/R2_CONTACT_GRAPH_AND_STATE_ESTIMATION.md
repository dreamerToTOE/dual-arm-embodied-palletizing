# R2 接触图与状态估计设计

## 图模型

每个物理帧建立一个带时间戳的图 `G_t=(V,E_t)`。节点包括 `Cube`、`Table`、`DeepWall`、`SideWall`、`LeftTool`、`RightTool`；边包括 `LEFT_TOOL_CUBE`、`RIGHT_TOOL_CUBE`、`CUBE_TABLE`、`CUBE_DEEP_WALL`、`CUBE_SIDE_WALL`。边状态不是布尔值，而是：

- `OBSERVED_ACTIVE`：本帧有对应传感/接触报告；
- `OBSERVED_INACTIVE`：本帧对应的已接入参考源没有报告；
- `UNKNOWN`：该边没有被当前监视器覆盖（本运行的侧墙边）；
- `STALE`：时间戳超过允许新鲜度，不能当作当前状态。

拓扑状态与任务状态分开。任务状态为 `FREE_SPACE_APPROACH`、`TOOL_CONTACT`、`NORMAL_ADVANCEMENT`、`DEEP_WALL_ARRIVAL`、`SUSPECTED_PREMATURE_BLOCKAGE`、`TRACKING_FAILURE`、`OBSERVATION_UNAVAILABLE` 等。未知边不会被当成无接触，也不会直接触发恢复。

## 两个最小估计器

`estimator/r2_contact_estimator.py` 实现两个可审计基线：

- **E0 Geometric/Motion Rules**：单帧因果规则。工具 TCP 到 Cube 中心的距离、Cube x 到名义深墙的有符号距离、Cube 底部 gap、Cube 速度和可选 tracking error 驱动状态；同时输出左右工具、桌面和深墙的几何边假设。它们是几何假设，不是物理接触真值；不读取未来帧或 PhysX oracle。
- **E1 Temporal Hysteresis**：E0 外包一层激活/释放驻留（默认 3 帧），对 `UNKNOWN/STALE`、跟踪失败和有命令但低速度的可疑阻塞立即传播。它只使用当前和历史已到达的帧，没有未来窗口。

`estimator/live_adapter.py` 是最小桥接扩展：把 PhysicalTrace 形状的运行行转换为 `FrameObservation`，并显式忽略 `oracle_reference`。它目前没有接入 Isaac 主循环，因为运行时还缺少同步命令、吸盘状态和侧墙观察接口。

## 离线与在线边界

PhysX contact report 被放在 `oracle_reference` 中，只作为回放参照标签；估计器输入只含 q/dq、TCP、Cube pose/velocity、几何特征和显式 UNKNOWN。这样可以检查“几何/运动基线是否复现名义事件”，但不能把参考标签 agreement 说成独立传感器真值，也不能由它证明物理卡滞恢复。

当前物理数据只覆盖 1 个名义 episode，未覆盖实际 premature jam、侧墙边、命令停滞或失吸。复杂 GNN/大规模学习估计器没有必要性证据，也没有训练或实现；本 R2 不把“接触图”命名为论文创新。
