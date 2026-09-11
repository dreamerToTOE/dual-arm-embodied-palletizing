# Task18：Randomized Scene + Ground Truth Task Input

状态：⚪ 待实现。

## 目标

把“初始位置已知并写死”升级为“每次仿真随机生成，控制系统只能从运行时输入获得物体状态”。

第一阶段不引入视觉误差，使用 Isaac Ground Truth 代替未来相机感知输出。

## 场景随机化

每个 episode 随机：

```text
x / y
可选 yaw
box type
box count
```

必须满足：

- 不初始重叠；
- 在至少一台机械臂可达区；
- 与桌边保持安全边界；
- 大件若要求双臂紧协调，则两个吸附点均应处于双臂可行工作区。

## 运行时输入接口

建议统一发布：

```text
DetectedObjectArray / BoxStateArray
```

每个对象至少包含：

```text
id
pose
size
mass / class（若仿真已知）
timestamp
```

未来相机模块只需替换该输入源，Task19+ 不关心数据来自 GT 还是 D455。

## Random Seed

支持：

```text
seed = fixed   # 回归复现
seed = random  # 压力测试
```

每次实验记录 seed，失败场景可一键重放。

## 验收标准

```text
T18-01  同一 seed 可复现完全相同的 Box 场景
T18-02  不同 seed 产生不同初始布局
T18-03  控制节点不读取源码中的固定初始坐标
T18-04  Pick pose 由运行时 Box pose 生成
T18-05  失败日志记录 seed + BoxSpec + pose
T18-06  Task16 benchmark 可批量运行随机场景
```

## 边界

- 暂不加入 RealSense / 点云；
- 暂不自动生成 placement pose；
- 本 Task 可继续使用外部给定目标位姿，以单独验证“随机 pick”。
