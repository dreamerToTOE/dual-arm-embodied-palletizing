# Task28 G1-2A 离线接触模式验证

本目录实现 `G1_2A_PASS` 范围内的离线、单状态接触模式和支持转换门禁。输入绑定
冻结 fixture `fixtures/task28_g1_0_new_single_cube_v0`，不修改 fixture、G1-1、
Task24/26/27 或现有 SRDF/ACM。

实现包括四个显式模式：

- `DETACHED_SUPPORTED`：Cube 在冻结 supply 支撑区，左右 attachment 均未观测到；
- `BILATERAL_SUPPORTED`：双侧 attachment 独立合成观测为真，Cube 仍在 supply 或 target 支撑区；
- `BILATERAL_AIRBORNE`：双侧 attachment 独立合成观测为真，没有支撑区，Cube 底部离桌面严格超过数值零带；
- `RELEASED_SUPPORTED`：attachment 已释放，Cube 在 target 支撑区。

支持接触只对一条已经由未修改 G1-1 FCL 返回的 `Cube_01`/`task28_table` 记录做局部
重分类。所有 Cube/工具、Cube/机械臂、机械臂/机械臂、自碰和 robot/table 碰撞仍然
是拒绝条件；没有写入或扩大 ACM。支撑检查使用冻结 Cube 八个顶点、底部四点、桌面
平面、指定 supply/target footprint、法向和接触深度，`1e-6 m` 是数值容差，不能解释
为实际力、摩擦、吸附或稳定性测量。`INDEPENDENT_SYNTHETIC` 反馈明确标注为合成测试
输入，命令回显不能当作 attachment 证据。

## 离线复现

在仓库根目录执行：

```bash
PYTHONPATH=offline/task28_g1_1:offline/task28_g1_2a \
  python3 -m pytest offline/task28_g1_2a/tests -q -p no:cacheprovider
PYTHONPATH=offline/task28_g1_1:offline/task28_g1_2a \
  python3 -u offline/task28_g1_2a/run_validation.py
```

脚本也回归运行 G1-1 测试，生成 [validation_results.json](evidence/validation_results.json)。
独立 FCL 支撑边界诊断在 [independent_fcl_support_audit.json](evidence/independent_fcl_support_audit.json)。
这些命令不启动 Isaac/ROS，不发机器人或吸附命令，不执行路径、扫掠、NR、QP、力控制或 RL。

转换验证只检查两个端点和声明的事件：

`DETACHED_SUPPORTED -> BILATERAL_SUPPORTED -> BILATERAL_AIRBORNE -> BILATERAL_SUPPORTED -> RELEASED_SUPPORTED`。

因此它不提供连续轨迹安全、边级碰撞安全或真实 attachment 成功证明；这些属于后续工单。

## G1-2A-R1：几何间隙与时序确认

R1 增加 `BILATERAL_TRANSITIONING` 单帧模式，以及独立的
`TemporalClearanceConfirmer`。0.5/1/2 mm 的 Cube 都是几何分离；只有三帧高阈值观测
才确认 airborne。接近桌面的三帧低阈值只确认 near-table，目标的三帧严格几何支撑和
显式释放事件仍是释放的前置条件。没有力传感器证据，输出
`physical_support_observed` 始终为 false。

默认 `d_low=0.001 m`、`d_high=0.003 m`，是可配置研究参数，独立于原有 `1e-6 m`
接触/穿透容差。参数在会话内固定；无效观测清零计数并撤销授权。Cube pose 与 attachment
feedback 均需要独立 evidence ID，六类时间戳必须严格递增；输入来源不得在会话内混合。

```bash
PYTHONPATH=offline/task28_g1_1:offline/task28_g1_2a \
  python3 -u offline/task28_g1_2a/run_r1_validation.py
```

这会分别运行原有 188 项回归及 R1 测试，保存
[evidence/r1_validation_results.json](evidence/r1_validation_results.json)，包括完整端点序列、
负例、15 组确定性阈值/噪声敏感性序列和 Cube-only 观测噪声控制。单状态关节配置与独立
USD/URDF FK 核验在 [examples/r1_configurations.json](examples/r1_configurations.json)；
可选离线重生成脚本为 `examples/build_r1_configurations.py`，不运行规划器。
原 G1-2A evidence 文件仍保留其历史内容。

详细语义、数学条件与边界见
[TASK28_G1_2A_R1_CLEARANCE_HYSTERESIS.md](../../docs/tasks/TASK28_G1_2A_R1_CLEARANCE_HYSTERESIS.md)。
