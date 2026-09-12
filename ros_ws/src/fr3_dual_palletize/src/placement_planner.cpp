#include "fr3_dual_palletize/placement_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>

namespace fr3_dual_palletize
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double yawFromPose(const geometry_msgs::msg::Pose& pose)
{
  const auto& q = pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Pose makePose(double x, double y, double z, double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.z = std::sin(0.5 * yaw);
  pose.orientation.w = std::cos(0.5 * yaw);
  return pose;
}

std::array<std::array<double, 2>, 4> corners(
  const geometry_msgs::msg::Pose& pose,
  const std::array<double, 3>& dimensions)
{
  const double yaw = yawFromPose(pose);
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  const double half_x = 0.5 * dimensions[0];
  const double half_y = 0.5 * dimensions[1];
  std::array<std::array<double, 2>, 4> result{};
  const std::array<std::array<double, 2>, 4> local{{
    {{-half_x, -half_y}}, {{-half_x, half_y}}, {{half_x, half_y}}, {{half_x, -half_y}},
  }};
  for (std::size_t index = 0; index < local.size(); ++index)
  {
    result[index][0] = pose.position.x + cosine * local[index][0] - sine * local[index][1];
    result[index][1] = pose.position.y + sine * local[index][0] + cosine * local[index][1];
  }
  return result;
}

bool rectanglesOverlap(
  const geometry_msgs::msg::Pose& first_pose, const std::array<double, 3>& first_size,
  const geometry_msgs::msg::Pose& second_pose, const std::array<double, 3>& second_size,
  double clearance)
{
  const auto first = corners(first_pose, first_size);
  const auto second = corners(second_pose, second_size);
  for (const auto& polygon : {first, second})
  {
    for (std::size_t index = 0; index < polygon.size(); ++index)
    {
      const auto& start = polygon[index];
      const auto& end = polygon[(index + 1) % polygon.size()];
      const double axis_x = -(end[1] - start[1]);
      const double axis_y = end[0] - start[0];
      const double length = std::hypot(axis_x, axis_y);
      if (length < 1e-9) continue;
      const double nx = axis_x / length;
      const double ny = axis_y / length;
      auto project = [nx, ny](const auto& rectangle, double& minimum, double& maximum)
      {
        minimum = maximum = nx * rectangle[0][0] + ny * rectangle[0][1];
        for (const auto& point : rectangle)
        {
          const double value = nx * point[0] + ny * point[1];
          minimum = std::min(minimum, value);
          maximum = std::max(maximum, value);
        }
      };
      double first_min{}, first_max{}, second_min{}, second_max{};
      project(first, first_min, first_max);
      project(second, second_min, second_max);
      if (first_max + clearance <= second_min || second_max + clearance <= first_min)
      {
        return false;
      }
    }
  }
  return true;
}

bool insideOrientedSupport(
  const geometry_msgs::msg::Pose& candidate_pose, const std::array<double, 3>& candidate_size,
  const geometry_msgs::msg::Pose& support_pose, const std::array<double, 3>& support_size,
  double clearance)
{
  const double yaw = yawFromPose(support_pose);
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  for (const auto& point : corners(candidate_pose, candidate_size))
  {
    const double dx = point[0] - support_pose.position.x;
    const double dy = point[1] - support_pose.position.y;
    const double local_x = cosine * dx + sine * dy;
    const double local_y = -sine * dx + cosine * dy;
    if (std::abs(local_x) > 0.5 * support_size[0] - clearance ||
        std::abs(local_y) > 0.5 * support_size[1] - clearance)
    {
      return false;
    }
  }
  return true;
}

bool insidePallet(
  const geometry_msgs::msg::Pose& pose, const std::array<double, 3>& size,
  const PalletRegion& region, double clearance)
{
  for (const auto& point : corners(pose, size))
  {
    if (point[0] < region.min_x + clearance || point[0] > region.max_x - clearance ||
        point[1] < region.min_y + clearance || point[1] > region.max_y - clearance)
    {
      return false;
    }
  }
  return true;
}

bool collidesPlaced(
  const geometry_msgs::msg::Pose& pose, const BoxSpec& box,
  const std::vector<PlacedBox>& placed, double clearance)
{
  const double bottom = pose.position.z - 0.5 * box.dimensions[2];
  const double top = pose.position.z + 0.5 * box.dimensions[2];
  for (const auto& existing : placed)
  {
    const double existing_bottom = existing.pose.position.z - 0.5 * existing.box.dimensions[2];
    const double existing_top = existing.pose.position.z + 0.5 * existing.box.dimensions[2];
    if (top <= existing_bottom + clearance || bottom >= existing_top - clearance)
    {
      continue;
    }
    if (rectanglesOverlap(pose, box.dimensions, existing.pose, existing.box.dimensions, clearance))
    {
      return true;
    }
  }
  return false;
}

}  // namespace

PlacementPlanner::PlacementPlanner(PlacementPlannerConfig config) : config_(std::move(config)) {}

PlacementPlanResult PlacementPlanner::plan(
  const BoxSpec& box, const PalletRegion& region, const std::vector<PlacedBox>& placed,
  const PlacementReachabilityCheck& reachability_check) const
{
  PlacementPlanResult result;
  if (config_.grid_step <= 0.0 || config_.min_support_ratio <= 0.0 ||
      config_.min_support_ratio > 1.0)
  {
    result.error = "PlacementPlannerConfig 无效。";
    return result;
  }
  const std::array<double, 2> yaws = {0.0, kPi / 2.0};
  const std::size_t yaw_count = config_.allow_quarter_turn ? yaws.size() : 1U;
  auto add = [&](const geometry_msgs::msg::Pose& pose, const std::string& support_id,
                 double support_height, bool require_support)
  {
    if (!insidePallet(pose, box.dimensions, region, config_.clearance) ||
        collidesPlaced(pose, box, placed, config_.clearance)) return;
    if (require_support)
    {
      const auto iterator = std::find_if(placed.begin(), placed.end(), [&support_id](const PlacedBox& item)
      { return item.box.id == support_id; });
      if (iterator == placed.end() || !insideOrientedSupport(
          pose, box.dimensions, iterator->pose, iterator->box.dimensions, config_.clearance)) return;
    }
    PlacementCandidate candidate;
    candidate.placement.id = "auto_place_" + box.id + "_" + std::to_string(result.candidates.size());
    candidate.placement.object_id = box.id;
    candidate.placement.target_pose = pose;
    candidate.placement.support_surface_id = support_id;
    candidate.placement.support_height = support_height;
    candidate.placement.orientation_tolerance_rad = 0.02;
    candidate.support_ratio = 1.0;
    const double center_distance = std::hypot(
      pose.position.x - 0.5 * (region.min_x + region.max_x),
      pose.position.y - 0.5 * (region.min_y + region.max_y));
    candidate.cost = 10.0 * pose.position.z + 0.10 * center_distance +
      (support_id == "table" ? 0.0 : 0.02);
    result.candidates.push_back(std::move(candidate));
  };

  // Height-map 的 table 基层：有限网格，而不是固定的 Task15 目标坐标。
  std::vector<double> x_samples;
  std::vector<double> y_samples;
  for (double x = region.min_x; x <= region.max_x + 1e-9; x += config_.grid_step) x_samples.push_back(x);
  for (double y = region.min_y; y <= region.max_y + 1e-9; y += config_.grid_step) y_samples.push_back(y);
  // 窄托盘或“托盘尺寸恰好等于大件尺寸”时，规则网格可能跨过唯一的合法中心。
  // 补充几何中心仍是通用 height-map 采样，不是写入某个 Task 的目标坐标。
  x_samples.push_back(0.5 * (region.min_x + region.max_x));
  y_samples.push_back(0.5 * (region.min_y + region.max_y));
  for (std::size_t yaw_index = 0; yaw_index < yaw_count; ++yaw_index)
  {
    const double yaw = yaws[yaw_index];
    for (const double x : x_samples)
    {
      for (const double y : y_samples)
      {
        add(makePose(x, y, region.support_height + 0.5 * box.dimensions[2], yaw),
            "table", region.support_height, false);
      }
    }
  }
  // 已放物体的顶面是下一层 support map。第一版要求候选底面完全位于单一支撑面内，
  // 所以 support ratio=1，避免桥接/悬空造成不稳定放置。
  for (const auto& support : placed)
  {
    const double support_top = support.pose.position.z + 0.5 * support.box.dimensions[2];
    const double support_yaw = yawFromPose(support.pose);
    const double cosine = std::cos(support_yaw);
    const double sine = std::sin(support_yaw);
    for (std::size_t yaw_index = 0; yaw_index < yaw_count; ++yaw_index)
    {
      // 在每个已放顶面建立局部 height-map，而不只尝试几何中心。这样同一个大件
      // 顶面可容纳多个小件；insideOrientedSupport() 仍保证完整支撑而非悬空。
      for (double local_x = -0.5 * support.box.dimensions[0];
           local_x <= 0.5 * support.box.dimensions[0] + 1e-9;
           local_x += config_.grid_step)
      {
        for (double local_y = -0.5 * support.box.dimensions[1];
             local_y <= 0.5 * support.box.dimensions[1] + 1e-9;
             local_y += config_.grid_step)
        {
          add(makePose(
              support.pose.position.x + cosine * local_x - sine * local_y,
              support.pose.position.y + sine * local_x + cosine * local_y,
              support_top + 0.5 * box.dimensions[2], yaws[yaw_index]),
              support.box.id, support_top, true);
        }
      }
      add(makePose(
          support.pose.position.x, support.pose.position.y,
          support_top + 0.5 * box.dimensions[2], yaws[yaw_index]),
          support.box.id, support_top, true);
    }
  }
  std::sort(result.candidates.begin(), result.candidates.end(),
    [](const PlacementCandidate& first, const PlacementCandidate& second) { return first.cost < second.cost; });
  for (const auto& candidate : result.candidates)
  {
    std::string reason;
    if (reachability_check && !reachability_check(candidate.placement, reason))
    {
      result.rejected_candidates.push_back(candidate.placement.id + ": " + reason);
      continue;
    }
    result.chosen = candidate;
    result.has_solution = true;
    return result;
  }
  std::ostringstream stream;
  stream << "没有满足边界、无重叠、完整支撑";
  if (reachability_check) stream << "与可达性预检";
  stream << "的 placement candidate。";
  result.error = stream.str();
  return result;
}

}  // namespace fr3_dual_palletize
