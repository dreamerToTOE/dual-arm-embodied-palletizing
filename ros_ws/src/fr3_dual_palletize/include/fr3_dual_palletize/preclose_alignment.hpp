#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace fr3_dual_palletize
{
using PrecloseVector3 = std::array<double, 3>;

// [ENGINEERING] 两杯仍 OPEN 时，一次合并 XYZ 实测残差；不是三个轴串行规划。
// 1 mm 是整个位移向量的上限，不是每轴 1 mm，避免对角线动作扩大旧微调范围。
// 调用方将该残差累加到上一条命令 FK，执行后按原几何门限重新检查。
inline PrecloseVector3 precloseAlignmentDeltaXYZ(
  const PrecloseVector3& desired, const PrecloseVector3& measured)
{
  PrecloseVector3 delta{};
  for (std::size_t axis = 0; axis < delta.size(); ++axis)
  {
    if (!std::isfinite(desired[axis]) || !std::isfinite(measured[axis]))
    {
      throw std::invalid_argument("Nonfinite pre-close XYZ input");
    }
    delta[axis] = desired[axis] - measured[axis];
    if (!std::isfinite(delta[axis]))
    {
      throw std::invalid_argument("Nonfinite pre-close XYZ residual");
    }
  }
  const double norm = std::hypot(std::hypot(delta[0], delta[1]), delta[2]);
  if (!std::isfinite(norm))
  {
    throw std::invalid_argument("Nonfinite pre-close XYZ norm");
  }
  constexpr double maximum_step = 0.001;
  if (norm > maximum_step)
  {
    for (auto& component : delta)
    {
      component *= maximum_step / norm;
    }
  }
  return delta;
}

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
