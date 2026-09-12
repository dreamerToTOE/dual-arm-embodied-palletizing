#pragma once

#include <array>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

namespace fr3_dual_palletize
{

// Task17 的物体描述只表达任务事实，不绑定某一个 Isaac Prim、某个 ROS topic
// 或左右臂。运行时感知输入、场景随机化和 Task21 路由均可复用这一模型。
struct GraspCandidate
{
  std::string id;
  // 相对 Box 中心的局部抓取位姿。第一版顶部吸盘通常使用 (0, 0, size_z / 2)。
  geometry_msgs::msg::Pose local_pose;
};

struct BoxSpec
{
  std::string id;
  std::array<double, 3> dimensions{};  // x / y / z, m
  double mass{0.0};                    // kg
  geometry_msgs::msg::Pose initial_pose;
  std::vector<GraspCandidate> grasp_candidates;
  std::vector<std::string> allowed_modes;
  std::string payload_class;
};

struct PlacementSpec
{
  std::string id;
  std::string object_id;
  geometry_msgs::msg::Pose target_pose;
  std::string support_surface_id;
  // 支撑面的 world Z，高度不必从 Box 推断，因而也可表达桌面、托盘或外部工装。
  double support_height{0.0};
  double orientation_tolerance_rad{0.0};
};

struct PalletRegion
{
  std::string frame_id{"world"};
  double min_x{0.0};
  double max_x{0.0};
  double min_y{0.0};
  double max_y{0.0};
  double support_height{0.0};
};

struct PalletizingJob
{
  std::string id;
  PalletRegion pallet_region;
  std::vector<BoxSpec> boxes;
  std::vector<PlacementSpec> placements;

  const BoxSpec* findBox(const std::string& id) const;
  const PlacementSpec* findPlacement(const std::string& id) const;
};

// 将同一 BoxSpec 转换为 MoveIt 碰撞物。传入 pose 允许 Task18 用运行时 Ground
// Truth 覆盖 initial_pose，而不用复制/修改物体几何。
moveit_msgs::msg::CollisionObject makeCollisionObject(
  const BoxSpec& box,
  const geometry_msgs::msg::Pose& pose,
  int operation = moveit_msgs::msg::CollisionObject::ADD,
  const std::string& frame_id = "world");

}  // namespace fr3_dual_palletize
