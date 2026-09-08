# Task05-C：MoveIt / Isaac 紧凑吸盘模型对齐

## 背景

Task05 B-A-C 高密度放置验证中：

- 使用官方 `cobot_pump` MoveIt 模型时，`PRE_PLACE -> PLACE` 在 B/C 中间插入失败；
- `load_gripper:=false` 后，同一 Task04 基础码垛事件单元、同一 B-A-C 几何场景完整成功；
- 本次成功结果：BoxA 最终约 `(0.6492, -0.1494, 0.0650)`，`e_xy ≈ 0.98 mm`，`e_z = 0.00 mm`。

因此项目不继续使用官方大尺寸 `cobot_pump` 环境碰撞模型，而建立与 Isaac 实际紧凑吸盘一致的自定义 MoveIt 模型。

## 自定义模型

尺寸严格复用 Task04 Isaac 基线：

- stem: `radius = 0.006 m`, `length = 0.099 m`
- cup: `radius = 0.010 m`, `length = 0.006 m`
- TCP offset: `0.105 m`

URDF 结构：

```text
fr3_link8
  ├─ fixed -> fr3_compact_suction
  │           ├─ stem collision
  │           └─ cup collision
  └─ fixed -> fr3_compact_suction_tcp (z = 0.105 m)
```

SRDF 仅增加结构相邻碰撞放行：

```text
fr3_link8 <-> fr3_compact_suction : Adjacent
```

不对 BoxB / BoxC 等环境障碍扩大 ACM。

## 当前兼容策略

当前 planning tip 继续保持 `fr3_link8`。

因此 Task04 起已验证的基础码垛事件单元仍保留：

```text
suction TCP target
    -> +0.105 m 几何换算
fr3_link8 pose target
```

暂不切换 planning tip 到 `fr3_compact_suction_tcp`，以减少对已验证控制代码的改动。

## 验收

1. 编译 `fr3_compact_suction_description`；
2. 启动 `moveit_compact_suction.launch.py`；
3. RViz 中确认末端为细杆 + 20 mm 直径吸盘，不存在官方 cobot_pump；
4. 重跑 Task05；
5. `PRE_PLACE -> PLACE (B-A-C INSERT)` 应为 `Cartesian fraction = 1.0000`；
6. Isaac 中 BoxA 能落入 B/C 中间并完成 RETREAT。

当前状态：🟡 模型代码已建立，待本机编译 + Isaac/MoveIt 联合验收。
