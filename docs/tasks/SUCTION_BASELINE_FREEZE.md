# 吸盘路线冻结记录（Task04–Task22-D）

**状态：🧊 已冻结**  
**冻结日期：2026-09-15**

## 决策

根据教师要求，项目后续末端执行器主线统一切换为 **FR3 二指夹爪**。因此，顶部 Surface Gripper / 紧凑吸盘路线不再新增功能、调整参数或继续扩展任务。

冻结不代表删除：现有吸盘实现、Isaac 场景、ROS Bridge、MoveIt 配置、Task04–Task22-D 文档和验收日志全部保留，作为后续夹爪方案的对照与可复现实验基线。

## 冻结范围

- 单臂吸盘抓取、放置和 Ground Truth 回写基线（Task04 / Task04-B）。
- B-A-C 顶部插入放置与紧凑吸盘 MoveIt/Isaac 几何对齐（Task05 / Task05-C）。
- 双臂吸盘松协调、紧协调共同物体搬运、放置与连续几何监测（Task06–Task15）。
- 运行时 BoxSpec、Placement、自动路由、连续调度与 Ground Truth 分派（Task16–Task22-D）中的吸盘执行实现。

代表性入口仍保留如下：

```text
isaac/scripts/task20e_runtime_feedback_scene.py
isaac/scripts/task20_runtime_suction_bridge.py
ros_ws/src/fr3_dual_palletize/src/task22_sequential_runtime_execute.cpp
docs/tasks/TASK22_GT_RUNTIME_DISPATCH.md
```

## 已有验收证据

冻结前已完成的代表性物理验收包括：

- 双吸盘共同物体运输：共同抬升 50 mm、共同 X 向运输 100 mm 通过；运输误差 0.311 mm，相对 `link8` 误差 0.123 mm。
- 双吸盘共同放置：放置误差 0.947 mm，姿态误差 0.026°。
- Task22-D 串行运行时闭环：同一 Isaac 场景完成 1 个紧协调对象和 2 个松协调对象的连续处理；紧协调放置误差 0.274 mm，两个松协调对象的 XY 放置误差分别为 1.932 mm 与 1.071 mm，`pending=0`。

这些数据只说明冻结版本在对应吸盘模型、物理参数和场景下可复现，不能作为二指夹爪方案的性能承诺。

## 夹爪主线的边界

后续夹爪工作必须独立定义并验证：

1. 二指夹爪的模型、TCP、碰撞几何和 MoveIt 配置；
2. 夹持宽度、接触/抓取确认、物体附着与释放语义；
3. 夹爪占用空间下的抓取姿态、放置姿态和双臂冲突策略；
4. 单臂、松协调和紧协调（如适用）的独立 Isaac 验收指标。

在用户确定新的夹爪规划策略并建立新的 Task 后，再解除“末端执行器主线”的冻结；历史吸盘基线仍保持只读参考。
