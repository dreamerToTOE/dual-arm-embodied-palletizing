# Task17：Box / Task Parameterization

状态：⚪ 待实现。

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
