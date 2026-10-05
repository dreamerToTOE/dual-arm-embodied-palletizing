#pragma once
#include <cstddef>

namespace fr3_dual_palletize
{
// [ADAPTATION] 2026-10-05 用户批准：前三件 rear+side 双吸附推压，后两件精准单后推。
// 下标零基，不能将第四件偷偷恢复成双臂侧压，也不能将第三件留在单臂 trim 分支。
constexpr bool dualFixtureRequired(std::size_t index) { return index < 3; }
constexpr bool preciseSingleFixtureRequired(std::size_t index)
{ return index == 3 || index == 4; }
constexpr bool validDualFixtureContactState(std::size_t index, bool rear, bool side)
{ return dualFixtureRequired(index) && rear && side; }
}  // namespace fr3_dual_palletize
