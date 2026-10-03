#pragma once

#include <cstddef>

namespace fr3_dual_palletize
{
// [ADAPTATION] 用户只批准第四件复用第五件的精准暂放、单臂后吸直推。
// 编号为零基；不得把第四件误归类为中央件或取消其邻件间隙验收。
constexpr bool precisionSingleInsert(std::size_t cube_index)
{
  return cube_index == 3 || cube_index == 4;
}

constexpr bool requiresInnerTrim(std::size_t cube_index)
{
  return cube_index == 2;
}

// 第四件在暂放时使用旧侧压终点已有的 0.5 mm 目标，而非进墙后侧压。
// 其他件的 1.5 mm 通道、场景和最终验收门限完全保留。
constexpr double stagingGap(std::size_t cube_index, double straight_gap, double pressed_gap)
{
  return cube_index == 3 ? pressed_gap : straight_gap;
}
}  // namespace fr3_dual_palletize
