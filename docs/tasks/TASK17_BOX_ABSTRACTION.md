# Task17：Box / Task Parameterization

状态：🟢 第一版已完成（2026-09-12）。

## 目标

移除 Task15 等代码中对箱体数量、尺寸、质量、初始位置、目标位置、支撑面和抓取点的硬编码，建立通用任务数据模型。

Task17 完成后，新增第 7、第 10 种箱体不应修改核心执行器源码，只修改配置或输入数据。

## 建议数据结构

```text
BoxSpec
  id
  size_x / size_y / size_z
  mass
  pose
  grasp_candidates[]
  allowed_modes[]
  payload_class

PlacementSpec
  target_pose
  support_surface_id
  support_height
  orientation_constraints

PalletizingJob
  boxes[]
  pallet_region
  optional_precomputed_targets[]
```

第一版可使用 YAML / JSON 配置驱动。

## 需要移除的硬编码类型

```text
BOX_SIZE = 0.030
NUM_SMALL_CUBES = 4
固定 target_x / target_y
固定 batch lower / upper
固定 large / small 二分类逻辑
固定 pose_index -> object 绑定
```

## 抓取接口

第一版仍以顶部吸盘为主，但 BoxSpec 应允许多个候选抓取点：

```text
center_top
left_top
right_top
```

为 Task21 松/紧协调模式选择预留。

## 验收标准

```text
T17-01  Box 几何由 BoxSpec 生成 MoveIt CollisionObject
T17-02  Primitive 不再假设固定 30 mm cube
T17-03  场景可从配置读取任意数量 Box
T17-04  目标位置可由外部 PlacementSpec 输入
T17-05  旧 Task15 配置可通过新数据模型复现原场景
T17-06  增加新的尺寸类型无需修改核心执行代码
```

## 实现

新增独立于 Isaac、MoveIt action 和左右臂分配的任务数据模型：

```text
PalletizingJob
  ├── PalletRegion
  ├── BoxSpec[]
  │     ├── id / dimensions / mass / initial_pose
  │     ├── grasp_candidates[]
  │     ├── allowed_modes[]
  │     └── payload_class
  └── PlacementSpec[]
        ├── object_id / target_pose
        ├── support_surface_id / support_height
        └── orientation_tolerance_rad
```

实现文件：

```text
include/fr3_dual_palletize/palletizing_job.hpp
include/fr3_dual_palletize/palletizing_job_loader.hpp
src/palletizing_job_loader.cpp
src/task17_job_validate.cpp
config/task15_legacy_job.yaml
config/task17_seven_box_fixture.yaml
```

YAML loader 会拒绝空/重复 id、非法尺寸或质量、零四元数、缺失抓取候选、未知
placement object、非法托盘边界等输入。`makeCollisionObject(BoxSpec, pose)` 统一
生成 MoveIt `CollisionObject`；运行时 pose 可覆盖 `initial_pose`，为 Task18 的
Ground Truth 输入预留。

同时，`PalletizePrimitive` 与 `TaskTrajectoryCandidate` 增加
`object_dimensions={x,y,z}`。抓取高度、AttachedCollisionObject、释放高度及
Task08/FCL 中的 world/attached Box 都由该几何数据计算，不再把新任务物体默认为
30 mm cube。旧 Task07/08/10 的固定 demo 常量保留为历史验收场景，不是新任务
数据接口。

## Task15 兼容配置

`config/task15_legacy_job.yaml` 逐项复现 Task15 的一个大件和四个小件：

```text
LargeCube: 0.220 x 0.320 x 0.080 m, 1.000 kg
SmallCube1--4: 0.030 x 0.030 x 0.030 m, 0.200 kg
Large target:      (0.650,  0.000, 0.090)
Lower targets:     (0.650, ±0.080, 0.145)
Upper targets:     (0.650, ±0.080, 0.175)
```

它同时记录大件的左右顶部抓取候选和小件的中心顶部抓取候选。Task15 已验收的
Isaac bridge/批次控制不在本 Task 中重写；后续 Task18 将把运行时 Ground Truth
按 Box id 接入同一模型，而非再引入新的固定 `pose_index -> object` 逻辑。

## 验证结果

构建：

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fr3_dual_palletize --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Task15 兼容配置：

```bash
ros2 run fr3_dual_palletize task17_job_validate --ros-args \
  -p job_config:=/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws/src/fr3_dual_palletize/config/task15_legacy_job.yaml
```

实际结果：`boxes=5, placements=5`，5 个 CollisionObject 的尺寸与上述 Task15
场景一致；lower/upper 的名义中心高度分别为 `0.145 m` 和 `0.175 m`。

扩展性 fixture：

```bash
ros2 run fr3_dual_palletize task17_job_validate --ros-args \
  -p job_config:=/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws/src/fr3_dual_palletize/config/task17_seven_box_fixture.yaml
```

实际结果：`boxes=7, placements=7`，覆盖 30 mm cube、扁平件、细长件、
0.140 x 0.100 x 0.060 m 大件和第七个 `0.080 x 0.040 x 0.090 m` 高件；所有
7 个 CollisionObject 均由同一数据模型生成。

Task16 回归：在更新后的携带物 FCL 几何链路上，`soft_preference`、5 candidates、
seed `20260913` 的 1 次 Task15 lower 只读基准得到：左右 primitive 均 PASS、
Task08 SAFE、Task09 `SIMULTANEOUS`、等待 `0.000 s`。该回归不发布 joint/suction
命令，也不驱动 Isaac。

## 验收结论

- T17-01：通过。`BoxSpec -> CollisionObject` 的三维尺寸逐项校验。
- T17-02：通过。Primitive 的接触、抓取、释放和 FCL 携带物几何均由
  `object_dimensions` 计算。
- T17-03：通过。`PalletizingJob.boxes` 是动态数组，七箱 fixture 无需核心分支。
- T17-04：通过。`PlacementSpec` 提供外部 target pose、支撑面和高度。
- T17-05：通过。Task15 的 5 个物体、5 个目标和抓取候选由兼容 YAML 复现。
- T17-06：通过。第七个不同尺寸 Box 仅新增 YAML 条目，解析/几何验证通过。
