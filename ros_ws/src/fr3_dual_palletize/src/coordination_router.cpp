#include "fr3_dual_palletize/coordination_router.hpp"

#include <cmath>
#include <utility>

namespace fr3_dual_palletize
{
namespace
{

bool finiteNonNegative(double value)
{
  return std::isfinite(value) && value >= 0.0;
}

CoordinationRouteScore evaluateLoose(
  CoordinationMode mode,
  const LooseModeEstimate& estimate,
  const CoordinationRouterConfig& config)
{
  CoordinationRouteScore score;
  score.mode = mode;
  if (!estimate.allowed_by_task)
  {
    score.reason = "任务输入未允许该 loose mode";
    return score;
  }
  if (!estimate.payload_safe)
  {
    score.reason = "单臂负载评估未通过";
    return score;
  }
  if (!estimate.pick_reachable || !estimate.place_reachable)
  {
    score.reason = "完整 pick/place 可达性未通过";
    return score;
  }
  if (!estimate.top_suction_stable)
  {
    score.reason = "顶部吸盘稳定性评估未通过";
    return score;
  }
  if (!estimate.fcl_safe)
  {
    score.reason = "Task08/FCL 门禁未通过";
    return score;
  }
  if (!finiteNonNegative(estimate.estimated_duration_sec) ||
      !finiteNonNegative(estimate.planning_cost))
  {
    score.reason = "候选时长或规划代价无效";
    return score;
  }
  score.feasible = true;
  score.score = config.duration_weight * estimate.estimated_duration_sec +
    config.planning_cost_weight * estimate.planning_cost;
  score.reason = estimate.diagnostics.empty() ?
    "单臂完整候选与 Task08/FCL 均通过" : estimate.diagnostics;
  return score;
}

CoordinationRouteScore evaluateTight(
  const TightModeEstimate& estimate,
  const CoordinationRouterConfig& config)
{
  CoordinationRouteScore score;
  score.mode = CoordinationMode::TIGHT_SHARED_OBJECT;
  if (!estimate.allowed_by_task)
  {
    score.reason = "任务输入未允许 tight_shared_object";
    return score;
  }
  if (!estimate.dual_grasp_feasible)
  {
    score.reason = "双吸盘共同抓取几何不可行";
    return score;
  }
  if (!estimate.payload_safe)
  {
    score.reason = "共同搬运负载评估未通过";
    return score;
  }
  if (!estimate.shared_transport_planner_ready)
  {
    score.reason = "通用 shared-object planner 尚不可用";
    return score;
  }
  if (!estimate.geometry_safe)
  {
    score.reason = "共同搬运几何安全门禁未通过";
    return score;
  }
  if (!finiteNonNegative(estimate.estimated_duration_sec) ||
      !finiteNonNegative(estimate.planning_cost))
  {
    score.reason = "共同搬运候选时长或规划代价无效";
    return score;
  }
  score.feasible = true;
  score.score = config.duration_weight *
    (estimate.estimated_duration_sec + config.tight_coordination_overhead_sec) +
    config.planning_cost_weight * estimate.planning_cost;
  score.reason = estimate.diagnostics.empty() ?
    "共同抓取、共享运输和几何安全均通过" : estimate.diagnostics;
  return score;
}

bool isLooseSingleMode(CoordinationMode mode)
{
  return mode == CoordinationMode::LOOSE_LEFT || mode == CoordinationMode::LOOSE_RIGHT;
}

}  // namespace

const char* coordinationModeName(CoordinationMode mode)
{
  switch (mode)
  {
    case CoordinationMode::LOOSE_LEFT: return "LOOSE_LEFT";
    case CoordinationMode::LOOSE_RIGHT: return "LOOSE_RIGHT";
    case CoordinationMode::LOOSE_DUAL_PARALLEL: return "LOOSE_DUAL_PARALLEL";
    case CoordinationMode::TIGHT_SHARED_OBJECT: return "TIGHT_SHARED_OBJECT";
    case CoordinationMode::NO_FEASIBLE_MODE: return "NO_FEASIBLE_MODE";
  }
  return "NO_FEASIBLE_MODE";
}

CoordinationRouter::CoordinationRouter(CoordinationRouterConfig config)
  : config_(std::move(config))
{
}

CoordinationRouteDecision CoordinationRouter::decide(
  const CoordinationRouteRequest& request) const
{
  CoordinationRouteDecision decision;
  if (request.box == nullptr || request.placement == nullptr ||
      request.box->id.empty() || request.placement->object_id != request.box->id)
  {
    decision.reason = "BoxSpec / PlacementSpec 不完整或 object_id 不匹配";
    return decision;
  }
  if (config_.duration_weight < 0.0 || config_.planning_cost_weight < 0.0 ||
      config_.tight_coordination_overhead_sec < 0.0)
  {
    decision.reason = "Router 排序权重无效";
    return decision;
  }

  decision.scores.push_back(evaluateLoose(
    CoordinationMode::LOOSE_LEFT, request.left, config_));
  decision.scores.push_back(evaluateLoose(
    CoordinationMode::LOOSE_RIGHT, request.right, config_));
  decision.scores.push_back(evaluateTight(request.tight, config_));

  for (const auto& score : decision.scores)
  {
    if (!score.feasible || score.score >= decision.score)
    {
      continue;
    }
    decision.mode = score.mode;
    decision.feasible = true;
    decision.score = score.score;
    decision.reason = score.reason;
  }
  if (!decision.feasible)
  {
    decision.mode = CoordinationMode::NO_FEASIBLE_MODE;
    decision.reason = "所有允许模式均未通过负载、可达性、稳定性或安全门禁";
  }
  return decision;
}

CoordinationRouteDecision CoordinationRouter::decideLooseParallel(
  const LooseParallelRequest& request) const
{
  CoordinationRouteDecision decision;
  CoordinationRouteScore score;
  score.mode = CoordinationMode::LOOSE_DUAL_PARALLEL;
  if (!request.first.feasible || !request.second.feasible ||
      !isLooseSingleMode(request.first.mode) ||
      !isLooseSingleMode(request.second.mode))
  {
    score.reason = "两个任务必须先各自通过单臂 loose 路由";
  }
  else if (request.first.mode == request.second.mode)
  {
    score.reason = "两个任务被分配到同一机械臂，不能并行";
  }
  else if (!request.pair_fcl_safe)
  {
    score.reason = "双任务 Task08/FCL 未通过";
  }
  else if (!request.temporal_schedule_safe)
  {
    score.reason = "Task09 时间协调未找到安全调度";
  }
  else if (!finiteNonNegative(request.estimated_makespan_sec))
  {
    score.reason = "并行 makespan 无效";
  }
  else
  {
    score.feasible = true;
    score.score = config_.duration_weight * request.estimated_makespan_sec;
    score.reason = request.diagnostics.empty() ?
      "不同单臂任务均安全，Task08/FCL 与 Task09 均通过" : request.diagnostics;
  }
  decision.scores.push_back(score);
  if (score.feasible)
  {
    decision.mode = score.mode;
    decision.feasible = true;
    decision.score = score.score;
    decision.reason = score.reason;
  }
  else
  {
    decision.mode = CoordinationMode::NO_FEASIBLE_MODE;
    decision.reason = score.reason;
  }
  return decision;
}

}  // namespace fr3_dual_palletize
