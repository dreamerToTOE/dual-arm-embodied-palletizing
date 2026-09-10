# Task11：双吸盘共同物体基线

状态：🟡 场景与物理 Bridge 已建立，等待首次 Isaac 运行时验收。

## 目标

在不引入共同搬运轨迹、力分配或柔顺控制前，先验证两个 FR3 紧凑顶部吸盘可对**同一个**动态刚体分别建立 Surface Gripper 连接：

```text
LEFT contact  -> LEFT SUCTION CLOSED
RIGHT contact -> RIGHT SUCTION CLOSED
                         ↓
                 SharedBox 同时具有两个物理约束
```

Task11 只验收双吸附建立和状态/位姿观测；共同抬升、保持相对几何和共同运输属于 Task12。

## 场景参数

| 项目 | 值 |
| --- | --- |
| Prim | `/World/SharedBox` |
| 尺寸 | `0.220 × 0.320 × 0.080 m`（x × y × z） |
| 质量 | `1.000 kg` |
| 初始中心 | `(0.550, 0.000, 0.090) m` |
| 顶面高度 | `z = 0.130 m` |
| 左吸附参考点 | `(0.550, -0.100, 0.131) m` |
| 右吸附参考点 | `(0.550, +0.100, 0.131) m` |
| 两点间距 | `0.200 m` |

箱体底面位于 Task06 桌面顶面 `z=0.050 m`。两个参考点距各自 y 向边缘 60 mm，明显大于 10 mm 吸盘 cup 半径；因此它们是分离、对称的顶部吸附点。

## 复用与约束

- 复用 Task06 双 FR3、桌面、独立 `/left` / `/right` joint command 链路和 MoveIt 描述。
- 复用 Task09/10 已验收的 Surface Gripper 参数：`gripThreshold=0.003 m`、`forceLimit=torqueLimit=1e6`、`retryClose=True`。
- 两个 gripper 均以各自 `fr3_hand` 为 parent，但均可连接 `/World/SharedBox`。
- 不扩大 Allowed Collision Matrix；不关闭重力；不把共同箱体拆为两个虚拟物体。
- Task11 不发布机器人关节命令，也不实现抬升。这样可将“共同物体物理约束是否成立”与后续紧协调轨迹问题隔离。

## 新增接口

```text
isaac/scripts/task11_shared_box_scene.py
isaac/scripts/task11_shared_box_bridge.py

/task11/left/suction_command   std_msgs/Bool
/task11/left/suction_state     std_msgs/Bool
/task11/right/suction_command  std_msgs/Bool
/task11/right/suction_state    std_msgs/Bool
/task11/shared_box_pose        geometry_msgs/PoseStamped
```

## 首次验收流程

在 Isaac Timeline Stop 时：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing-task11-shared-object-baseline/isaac/scripts/task11_shared_box_scene.py").read())
```

在 Play 后：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing-task11-shared-object-baseline/isaac/scripts/task11_shared_box_bridge.py").read())
```

之后由 Task11 控制节点（下一步）将左右吸盘分别移动到表中参考点；再向两个 command topic 发送 `true`，并确认：

```text
/task11/left/suction_state  = true
/task11/right/suction_state = true
```

保持双吸附期间读取 `/task11/shared_box_pose`，确认箱体没有掉落或被重力穿透。释放顺序将作为 Task13 的内容，不在本 Task11 基线中提前实现。

## 后续边界

```text
Task11: 双吸附建立与 SharedBox Ground Truth
Task12: 共同抬升 / 共同运输 / 相对位姿保持
Task13: 共同下降 / 同步释放 / 双臂安全退出
```
