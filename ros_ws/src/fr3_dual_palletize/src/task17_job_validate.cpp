// Task17：只读 Job 数据验证器。
// 不连接 MoveIt action、不发布关节/吸盘命令；用于在执行前验证 YAML 能否产生准确的
// BoxSpec、PlacementSpec 和 MoveIt CollisionObject。

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "fr3_dual_palletize/palletizing_job_loader.hpp"

namespace
{

bool approximatelyEqual(double first, double second)
{
  return std::abs(first - second) <= 1.0e-9;
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("task17_job_validate");
  const std::string job_config = node->declare_parameter<std::string>("job_config", "");
  if (job_config.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "Task17 必须传入 job_config:=/absolute/path/to/job.yaml。");
    rclcpp::shutdown();
    return 1;
  }

  fr3_dual_palletize::PalletizingJob job;
  std::string error;
  if (!fr3_dual_palletize::loadPalletizingJobYaml(job_config, job, error))
  {
    RCLCPP_ERROR(node->get_logger(), "Task17 YAML 无效：%s", error.c_str());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "========== Task17 JOB VALIDATION: id=%s, boxes=%zu, placements=%zu ==========" ,
    job.id.c_str(), job.boxes.size(), job.placements.size());

  for (const auto& box : job.boxes)
  {
    const auto collision = fr3_dual_palletize::makeCollisionObject(
      box, box.initial_pose, moveit_msgs::msg::CollisionObject::ADD,
      job.pallet_region.frame_id);
    if (collision.primitives.size() != 1 ||
        collision.primitives.front().dimensions.size() != 3 ||
        !approximatelyEqual(collision.primitives.front().dimensions[0], box.dimensions[0]) ||
        !approximatelyEqual(collision.primitives.front().dimensions[1], box.dimensions[1]) ||
        !approximatelyEqual(collision.primitives.front().dimensions[2], box.dimensions[2]))
    {
      RCLCPP_ERROR(node->get_logger(), "Task17 CollisionObject 几何转换失败：%s", box.id.c_str());
      rclcpp::shutdown();
      return 1;
    }
    RCLCPP_INFO(
      node->get_logger(),
      "Box id=%s size=(%.3f, %.3f, %.3f) m mass=%.3f kg grasps=%zu modes=%zu",
      box.id.c_str(), box.dimensions[0], box.dimensions[1], box.dimensions[2], box.mass,
      box.grasp_candidates.size(), box.allowed_modes.size());
  }

  for (const auto& placement : job.placements)
  {
    const auto* box = job.findBox(placement.object_id);
    if (box == nullptr)
    {
      RCLCPP_ERROR(node->get_logger(), "Task17 Placement 引用了未知 Box：%s", placement.object_id.c_str());
      rclcpp::shutdown();
      return 1;
    }
    const double expected_center_z = placement.support_height + 0.5 * box->dimensions[2];
    RCLCPP_INFO(
      node->get_logger(),
      "Placement id=%s object=%s support=%s z=%.3f (nominal center z=%.3f)",
      placement.id.c_str(), placement.object_id.c_str(), placement.support_surface_id.c_str(),
      placement.support_height, expected_center_z);
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Task17 PASS：YAML 已生成 %zu 个独立 CollisionObject；新增 Box 仅需改配置，不改核心执行代码。",
    job.boxes.size());
  rclcpp::shutdown();
  return 0;
}
