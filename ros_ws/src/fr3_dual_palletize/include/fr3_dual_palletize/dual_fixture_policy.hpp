#pragma once
#include <cstddef>
#include <cmath>

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
}  // namespace fr3_dual_palletize
