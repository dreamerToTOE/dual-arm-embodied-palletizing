# Task20：Multi-size Palletizing / 多尺寸连续码垛

状态：⚪ 待实现。

## 目标

在 Task17--19 的通用数据接口上，完成至少 6 类不同尺寸箱体的连续码垛，不再依赖“30 mm 小 Cube + 1 个固定大件”二分类 Demo。

## 尺寸集合

第一版至少准备 6 类尺寸，具体数值作为配置而不是写死在控制器中。

建议覆盖：

```text
小 / 中 / 大
细长 / 扁平 / 近方形
单臂明显可搬 / 双臂更合适
```

质量可先与体积按简化规则关联，后续再独立随机化。

## Episode

每个 episode：

```text
随机 Box 类型组合
+ 随机初始位置
+ Placement Planner 自动生成目标
+ Task21 选择松/紧协调模式
+ Task16 鲁棒运动规划
+ Task08/FCL 安全验证
+ 执行
```

## 重点指标

```text
episode success rate
box success rate
planning failure rate
IK failure rate
collision-rejection rate
coordination strategy distribution
mean makespan
mean replans
placement utilization
```

## 验收标准

```text
T20-01  至少 6 类尺寸均可由同一 BoxSpec 数据模型描述
T20-02  核心执行代码不按具体尺寸写 if/else 特例
T20-03  随机初始位置下可连续处理多个 Box
T20-04  目标由 Task19 自动生成
T20-05  每个任务均通过 Task08/FCL 或紧协调几何安全检查
T20-06  批量 episode 输出统计结果，而不是只保存单次成功演示
T20-07  失败可按 seed 复现
```
