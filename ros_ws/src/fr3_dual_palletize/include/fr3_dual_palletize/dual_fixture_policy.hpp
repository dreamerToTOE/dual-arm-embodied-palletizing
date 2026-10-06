#pragma once
#include <cstddef>
#include <cmath>
#include <array>
#include <cstdint>

namespace fr3_dual_palletize
{
// [ADAPTATION] 2026-10-05 用户批准：前三件 rear+side 双吸附推压，后两件精准单后推。
// 下标零基，不能将第四件偷偷恢复成双臂侧压，也不能将第三件留在单臂 trim 分支。
constexpr bool dualFixtureRequired(std::size_t index) { return index < 3; }
constexpr bool preciseSingleFixtureRequired(std::size_t index)
{ return index == 3 || index == 4; }
constexpr bool validDualFixtureContactState(std::size_t index, bool rear, bool side)
{ return dualFixtureRequired(index) && rear && side; }
// [ENGINEERING] 有限姿态参数范围，不是放宽物理/碰撞/力矩验收门限。
inline bool validSideFixtureRoll(double degrees)
{ return std::isfinite(degrees) && std::abs(degrees) <= 30.0; }
// [ENGINEERING] 观测格式/新鲜度，不更改原几何与力矩门限。
inline bool validFixturePose(const std::array<double, 7>& pose)
{
  for (double value : pose) if (!std::isfinite(value)) return false;
  const double norm2 = pose[3]*pose[3]+pose[4]*pose[4]+pose[5]*pose[5]+pose[6]*pose[6];
  return std::abs(norm2 - 1.0) <= 1e-6;
}
inline bool validFixtureStamp(std::int64_t stamp, std::int64_t previous)
{ return stamp > 0 && stamp > previous; }
inline bool freshFixtureReceipt(double age_sec)
{ return std::isfinite(age_sec) && age_sec >= 0.0 && age_sec <= 0.25; }
}  // namespace fr3_dual_palletize
