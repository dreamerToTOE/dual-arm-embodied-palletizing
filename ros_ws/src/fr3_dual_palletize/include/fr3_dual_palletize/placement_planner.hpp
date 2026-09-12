#pragma once

#include <functional>
#include <string>
#include <vector>

#include "fr3_dual_palletize/palletizing_job.hpp"

namespace fr3_dual_palletize
{

struct PlacedBox
{
  BoxSpec box;
  geometry_msgs::msg::Pose pose;
};

struct PlacementCandidate
{
  PlacementSpec placement;
  double support_ratio{0.0};
  double cost{0.0};
};

struct PlacementPlannerConfig
{
  double grid_step{0.050};
  double clearance{0.002};
  double min_support_ratio{0.98};
  bool allow_quarter_turn{true};
};

struct PlacementPlanResult
{
  std::vector<PlacementCandidate> candidates;
  bool has_solution{false};
  PlacementCandidate chosen;
  std::string error;
  std::vector<std::string> rejected_candidates;
};

// 快速可达性预检由调用者提供。false 表示跳过当前几何候选而非让 MoveIt 对同一
// 不可达目标重复采样；Task19 本身不负责具体机械臂的轨迹规划。
using PlacementReachabilityCheck = std::function<bool(const PlacementSpec&, std::string&)>;

class PlacementPlanner
{
public:
  explicit PlacementPlanner(PlacementPlannerConfig config = PlacementPlannerConfig{});

  PlacementPlanResult plan(
    const BoxSpec& box,
    const PalletRegion& region,
    const std::vector<PlacedBox>& placed,
    const PlacementReachabilityCheck& reachability_check = PlacementReachabilityCheck{}) const;

private:
  PlacementPlannerConfig config_;
};

}  // namespace fr3_dual_palletize
