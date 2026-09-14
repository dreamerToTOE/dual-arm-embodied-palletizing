#pragma once

#include <functional>
#include <memory>
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
  // geometric_cost 由不可绕过的几何层给出；selection_cost 可由未来用户模型覆盖。
  double geometric_cost{0.0};
  double selection_cost{0.0};
  std::string selection_reason;
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
  std::string selection_model_id;
  std::string error;
  std::vector<std::string> rejected_candidates;
};

// 快速可达性预检由调用者提供。false 表示跳过当前几何候选而非让 MoveIt 对同一
// 不可达目标重复采样；Task19 本身不负责具体机械臂的轨迹规划。
using PlacementReachabilityCheck = std::function<bool(const PlacementSpec&, std::string&)>;

// 选择模型仅看已经通过边界、碰撞与完整支撑门禁的候选。未来模型可以是启发式、
// 学习模型或外部服务 adapter；它只能重排候选，不能绕过几何安全层。
struct PlacementSelectionContext
{
  const BoxSpec& box;
  const PalletRegion& region;
  const std::vector<PlacedBox>& placed;
};

class PlacementSelectionModel
{
public:
  virtual ~PlacementSelectionModel() = default;
  virtual std::string id() const = 0;
  virtual bool scoreCandidates(
    const PlacementSelectionContext& context,
    std::vector<PlacementCandidate>& candidates,
    std::string& error) const = 0;
};

// 当前默认模型：复用确定性的几何 cost。保留为独立类，用户模型无需改 planner。
class GeometricPlacementSelectionModel final : public PlacementSelectionModel
{
public:
  std::string id() const override;
  bool scoreCandidates(
    const PlacementSelectionContext& context,
    std::vector<PlacementCandidate>& candidates,
    std::string& error) const override;
};

class PlacementPlanner
{
public:
  explicit PlacementPlanner(
    PlacementPlannerConfig config = PlacementPlannerConfig{},
    std::shared_ptr<const PlacementSelectionModel> selection_model = nullptr);

  PlacementPlanResult plan(
    const BoxSpec& box,
    const PalletRegion& region,
    const std::vector<PlacedBox>& placed,
    const PlacementReachabilityCheck& reachability_check = PlacementReachabilityCheck{}) const;

private:
  PlacementPlannerConfig config_;
  std::shared_ptr<const PlacementSelectionModel> selection_model_;
};

}  // namespace fr3_dual_palletize
