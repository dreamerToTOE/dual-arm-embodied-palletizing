#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fr3_dual_palletize
{
// [ENGINEERING] 未吸附时的有界残差修正，不改变对称性/接触门限。
// 旧实现的 0.65 mm 最小步长大于临近门限的剩余误差，会越过目标。
// 小误差应发小位移，由精细 IK、同步 FCL 和真实 TCP 复测确认执行。
inline double precloseAlignmentDelta(double measured_gap, bool left)
{
  if (!std::isfinite(measured_gap))
  {
    throw std::invalid_argument("Nonfinite pre-close gap");
  }
  constexpr double nominal_gap = 0.001;
  constexpr double maximum_step = 0.001;
  const double residual = measured_gap - nominal_gap;
  return std::clamp(left ? residual : -residual, -maximum_step, maximum_step);
}
}  // namespace fr3_dual_palletize
