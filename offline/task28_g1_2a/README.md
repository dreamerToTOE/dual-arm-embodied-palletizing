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
