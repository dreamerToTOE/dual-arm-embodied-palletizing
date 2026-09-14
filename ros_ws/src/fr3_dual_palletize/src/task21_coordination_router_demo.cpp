// Task21：可解释的松/紧协调 Router 回归演示。
//
// 此 demo 不连接 Isaac、不规划轨迹；它验证 Router 的契约：全部可达性、稳定性与
// FCL 事实由调用者输入，Router 只做安全门禁后的可解释模式选择。Task20-B 将实际
// Task16 / Task08 结果填入同一接口。

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "fr3_dual_palletize/coordination_router.hpp"

namespace
{

fr3_dual_palletize::BoxSpec makeBox(const std::string& id, std::vector<std::string> modes)
{
  fr3_dual_palletize::BoxSpec box;
  box.id = id;
  box.dimensions = {{0.042, 0.071, 0.033}};
  box.mass = 0.37;
  box.allowed_modes = std::move(modes);
  return box;
}

fr3_dual_palletize::PlacementSpec makePlacement(const std::string& id)
{
  fr3_dual_palletize::PlacementSpec placement;
  placement.id = "auto_place_" + id;
  placement.object_id = id;
  placement.target_pose.orientation.w = 1.0;
  placement.support_height = 0.050;
  return placement;
}

fr3_dual_palletize::LooseModeEstimate feasibleLoose(double duration, double cost)
{
  fr3_dual_palletize::LooseModeEstimate estimate;
  estimate.allowed_by_task = true;
  estimate.payload_safe = true;
  estimate.pick_reachable = true;
  estimate.place_reachable = true;
  estimate.top_suction_stable = true;
  estimate.fcl_safe = true;
  estimate.estimated_duration_sec = duration;
  estimate.planning_cost = cost;
  estimate.diagnostics = "runtime Task16 + Task08 facts";
  return estimate;
}

bool expect(
  const rclcpp::Logger& logger,
  const std::string& label,
  const fr3_dual_palletize::CoordinationRouteDecision& decision,
  fr3_dual_palletize::CoordinationMode expected)
{
  const bool ok = decision.mode == expected &&
    (expected == fr3_dual_palletize::CoordinationMode::NO_FEASIBLE_MODE ||
     decision.feasible);
  RCLCPP_INFO(
    logger, "case=%s mode=%s score=%.3f reason=%s", label.c_str(),
    fr3_dual_palletize::coordinationModeName(decision.mode), decision.score,
    decision.reason.c_str());
  if (!ok)
  {
    RCLCPP_ERROR(
      logger, "case=%s expected=%s", label.c_str(),
      fr3_dual_palletize::coordinationModeName(expected));
  }
  return ok;
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("task21_coordination_router_demo");
  fr3_dual_palletize::CoordinationRouter router;
  bool all_passed = true;

  {
    auto box = makeBox("left_only", {"loose_left", "loose_right"});
    auto placement = makePlacement(box.id);
    fr3_dual_palletize::CoordinationRouteRequest request;
    request.box = &box;
    request.placement = &placement;
    request.left = feasibleLoose(8.0, 2.0);
    request.right = feasibleLoose(11.0, 1.0);
    all_passed &= expect(
      node->get_logger(), "lowest_full_candidate_cost", router.decide(request),
      fr3_dual_palletize::CoordinationMode::LOOSE_LEFT);
  }
  {
    auto box = makeBox("right_reachable", {"loose_left", "loose_right"});
    auto placement = makePlacement(box.id);
    fr3_dual_palletize::CoordinationRouteRequest request;
    request.box = &box;
    request.placement = &placement;
    request.left.allowed_by_task = true;
    request.left.payload_safe = true;
    request.left.top_suction_stable = true;
    request.left.fcl_safe = true;
    request.left.diagnostics = "pick IK failed";
    request.right = feasibleLoose(9.0, 1.5);
    all_passed &= expect(
      node->get_logger(), "reachability_gate", router.decide(request),
      fr3_dual_palletize::CoordinationMode::LOOSE_RIGHT);
  }
  {
    auto box = makeBox("shared_only", {"tight_shared_object"});
    auto placement = makePlacement(box.id);
    fr3_dual_palletize::CoordinationRouteRequest request;
    request.box = &box;
    request.placement = &placement;
    request.tight.allowed_by_task = true;
    request.tight.dual_grasp_feasible = true;
    request.tight.payload_safe = true;
    request.tight.shared_transport_planner_ready = true;
    request.tight.geometry_safe = true;
    request.tight.estimated_duration_sec = 14.0;
    request.tight.planning_cost = 3.0;
    all_passed &= expect(
      node->get_logger(), "tight_when_loose_disallowed", router.decide(request),
      fr3_dual_palletize::CoordinationMode::TIGHT_SHARED_OBJECT);
  }
  {
    auto box = makeBox("shared_unavailable", {"tight_shared_object"});
    auto placement = makePlacement(box.id);
    fr3_dual_palletize::CoordinationRouteRequest request;
    request.box = &box;
    request.placement = &placement;
    request.tight.allowed_by_task = true;
    request.tight.dual_grasp_feasible = true;
    request.tight.payload_safe = true;
    request.tight.geometry_safe = true;
    // shared_transport_planner_ready 保持 false：必须拒绝，不能降级为 loose。
    all_passed &= expect(
      node->get_logger(), "tight_safe_reject", router.decide(request),
      fr3_dual_palletize::CoordinationMode::NO_FEASIBLE_MODE);
  }
  {
    fr3_dual_palletize::LooseParallelRequest request;
    request.first.mode = fr3_dual_palletize::CoordinationMode::LOOSE_LEFT;
    request.first.feasible = true;
    request.second.mode = fr3_dual_palletize::CoordinationMode::LOOSE_RIGHT;
    request.second.feasible = true;
    request.pair_fcl_safe = true;
    request.temporal_schedule_safe = true;
    request.estimated_makespan_sec = 12.0;
    request.diagnostics = "Task08=SAFE; Task09=NO_WAIT";
    all_passed &= expect(
      node->get_logger(), "parallel_after_pair_gates", router.decideLooseParallel(request),
      fr3_dual_palletize::CoordinationMode::LOOSE_DUAL_PARALLEL);
  }

  if (all_passed)
  {
    RCLCPP_INFO(
      node->get_logger(),
      "Task21 PASS：Router 根据负载/完整可达性/吸盘稳定性/FCL/共享 planner 事实选择模式；"
      "未按尺寸阈值分流，也未执行机器人命令。");
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Task21 FAIL：Router contract regression failed.");
  }
  rclcpp::shutdown();
  return all_passed ? 0 : 1;
}
