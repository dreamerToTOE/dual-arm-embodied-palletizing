#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "fr3_dual_palletize/palletizing_job_loader.hpp"
#include "fr3_dual_palletize/placement_planner.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("task19_placement_planner_demo");
  const auto job_path = node->declare_parameter<std::string>("job_config", "");
  if (job_path.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "Task19 需要 job_config:=/absolute/path/to/job.yaml。");
    rclcpp::shutdown(); return 1;
  }
  fr3_dual_palletize::PalletizingJob job;
  std::string error;
  if (!fr3_dual_palletize::loadPalletizingJobYaml(job_path, job, error))
  {
    RCLCPP_ERROR(node->get_logger(), "Task19 读取 job 失败：%s", error.c_str());
    rclcpp::shutdown(); return 1;
  }
  // Task15 的大件尺寸恰好填满其遗留 pallet_region，因此回归 fixture 使用零
  // 边界余量；新任务可保留 PlacementPlannerConfig 默认的 2 mm 安全余量。
  fr3_dual_palletize::PlacementPlannerConfig config;
  config.clearance = 0.0;
  fr3_dual_palletize::PlacementPlanner planner(config);
  std::vector<fr3_dual_palletize::PlacedBox> placed;
  RCLCPP_INFO(node->get_logger(), "========== Task19 PLACEMENT PLANNER: job=%s ==========" , job.id.c_str());
  for (const auto& box : job.boxes)
  {
    const auto plan = planner.plan(box, job.pallet_region, placed);
    if (!plan.has_solution)
    {
      RCLCPP_ERROR(node->get_logger(), "Task19 FAIL: %s -> %s", box.id.c_str(), plan.error.c_str());
      rclcpp::shutdown(); return 1;
    }
    const auto& pose = plan.chosen.placement.target_pose;
    RCLCPP_INFO(node->get_logger(), "Box=%s candidates=%zu chosen=%s support=%s pose=(%.3f, %.3f, %.3f) cost=%.3f",
      box.id.c_str(), plan.candidates.size(), plan.chosen.placement.id.c_str(),
      plan.chosen.placement.support_surface_id.c_str(), pose.position.x, pose.position.y,
      pose.position.z, plan.chosen.cost);
    placed.push_back({box, pose});
  }
  // 以拒绝 table 候选的可达性 callback 验证“不可达时回退到下一 candidate”。
  const auto* large = job.findBox("task15_large_cube");
  const auto* small = job.findBox("task15_small_cube_1");
  if (large != nullptr && small != nullptr)
  {
    std::vector<fr3_dual_palletize::PlacedBox> support{{*large, large->initial_pose}};
    const auto fallback = planner.plan(*small, job.pallet_region, support,
      [](const auto& placement, std::string& reason)
      {
        if (placement.support_surface_id == "table") { reason = "fixture: table target unreachable"; return false; }
        return true;
      });
    if (!fallback.has_solution || fallback.rejected_candidates.empty() ||
        fallback.chosen.placement.support_surface_id != large->id)
    {
      RCLCPP_ERROR(node->get_logger(), "Task19 FAIL: reachability fallback 未进入大件顶面候选。");
      rclcpp::shutdown(); return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Fallback PASS: rejected=%zu, support=%s", fallback.rejected_candidates.size(),
      fallback.chosen.placement.support_surface_id.c_str());
  }
  RCLCPP_INFO(node->get_logger(), "Task19 PASS：自动生成 PlacementSpec；无固定目标坐标，且可达性失败会回退到下一候选。");
  rclcpp::shutdown(); return 0;
}
