#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace fr3_dual_palletize
{
// [ENGINEERING] 仅约束已释放后的空载局部 IK 延拓，不是接触/论文控制器。
// 2 mm 段复用既有精细 IK；原 4 mm/0.08 rad 局部界限不变。
inline std::size_t emptyRetreatSegmentCount(double span_m, double angle_rad)
{
  if (!std::isfinite(span_m) || !std::isfinite(angle_rad) ||
      span_m < 0.0 || span_m > 0.6 || angle_rad < 0.0 || angle_rad > 3.141593)
  {
    return 0;
  }
  return static_cast<std::size_t>(std::max(
    1.0, std::ceil(std::max(span_m / 0.002, angle_rad / 0.01))));
}

inline bool validMeasuredJointSeed(const std::vector<double>& q, std::size_t count)
{
  return count > 0 && q.size() == count && std::all_of(q.begin(), q.end(),
    [](double value) { return std::isfinite(value); });
}

inline bool validEmptyRetreatSeed(
  bool left_closed, bool right_closed,
  const std::vector<double>& left, const std::vector<double>& right,
  std::size_t left_count, std::size_t right_count)
{
  return !left_closed && !right_closed && validMeasuredJointSeed(left, left_count) &&
    validMeasuredJointSeed(right, right_count);
}
}  // namespace fr3_dual_palletize
