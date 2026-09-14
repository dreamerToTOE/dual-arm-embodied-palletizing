#pragma once

#include <limits>
#include <string>
#include <vector>

#include "fr3_dual_palletize/palletizing_job.hpp"

namespace fr3_dual_palletize
{

// Task21 的高层协调输出。它不直接产生轨迹；下游仍必须分别调用已验收的
// Task16 / Task08-FCL（loose）或 shared-object geometry gate（tight）。
enum class CoordinationMode
{
  LOOSE_LEFT,
  LOOSE_RIGHT,
  LOOSE_DUAL_PARALLEL,
  TIGHT_SHARED_OBJECT,
  NO_FEASIBLE_MODE,
};

const char* coordinationModeName(CoordinationMode mode);

// 调用方的单臂实际评估结果。Router 不把尺寸阈值当作可达性或稳定性的替代品；
// 这些事实应来自 Task16 candidate / FCL、负载模型和吸盘评估器。
struct LooseModeEstimate
{
  bool allowed_by_task{false};
  bool payload_safe{false};
  bool pick_reachable{false};
  bool place_reachable{false};
  bool top_suction_stable{false};
  bool fcl_safe{false};
  double estimated_duration_sec{std::numeric_limits<double>::infinity()};
  double planning_cost{std::numeric_limits<double>::infinity()};
  std::string diagnostics;
};

// 对同一 Box 的双臂共享搬运评估。第一版将 shared-object planner 是否已准备好
// 作为显式输入，因此不存在时 Router 必须返回 NO_FEASIBLE_MODE，而不能借用
// loose primitive 假装完成 tight。
struct TightModeEstimate
{
  bool allowed_by_task{false};
  bool dual_grasp_feasible{false};
  bool payload_safe{false};
  bool shared_transport_planner_ready{false};
  bool geometry_safe{false};
  double estimated_duration_sec{std::numeric_limits<double>::infinity()};
  double planning_cost{std::numeric_limits<double>::infinity()};
  std::string diagnostics;
};

struct CoordinationRouteRequest
{
  const BoxSpec* box{nullptr};
  const PlacementSpec* placement{nullptr};
  LooseModeEstimate left;
  LooseModeEstimate right;
  TightModeEstimate tight;
};

struct CoordinationRouteScore
{
  CoordinationMode mode{CoordinationMode::NO_FEASIBLE_MODE};
  bool feasible{false};
  double score{std::numeric_limits<double>::infinity()};
  std::string reason;
};

struct CoordinationRouteDecision
{
  CoordinationMode mode{CoordinationMode::NO_FEASIBLE_MODE};
  bool feasible{false};
  double score{std::numeric_limits<double>::infinity()};
  std::string reason;
  std::vector<CoordinationRouteScore> scores;
};

struct LooseParallelRequest
{
  // 两个任务必须已经分别被路由到不同单臂 loose mode；两条完整候选的 FCL
  // 和时间协调结果由 Task08 / Task09 提供。
  CoordinationRouteDecision first;
  CoordinationRouteDecision second;
  bool pair_fcl_safe{false};
  bool temporal_schedule_safe{false};
  double estimated_makespan_sec{std::numeric_limits<double>::infinity()};
  std::string diagnostics;
};

struct CoordinationRouterConfig
{
  // 均为可解释排序权重。所有安全条件均是硬门禁；评分只在可行模式间选择。
  double duration_weight{1.0};
  double planning_cost_weight{0.10};
  double tight_coordination_overhead_sec{0.0};
};

class CoordinationRouter
{
public:
  explicit CoordinationRouter(CoordinationRouterConfig config = CoordinationRouterConfig{});

  CoordinationRouteDecision decide(const CoordinationRouteRequest& request) const;

  // 两个已被单独批准的 loose 任务在 Task08/FCL + Task09 均通过后，才可升级为
  // LOOSE_DUAL_PARALLEL。否则显式拒绝并保留它们各自的单臂决策。
  CoordinationRouteDecision decideLooseParallel(const LooseParallelRequest& request) const;

private:
  CoordinationRouterConfig config_;
};

}  // namespace fr3_dual_palletize
