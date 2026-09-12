#include "fr3_dual_palletize/palletizing_job_loader.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <set>
#include <sstream>
#include <utility>

#include <shape_msgs/msg/solid_primitive.hpp>
#include <yaml-cpp/yaml.h>

namespace fr3_dual_palletize
{
namespace
{

bool readVector3(
  const YAML::Node& parent,
  const char* field,
  std::array<double, 3>& value,
  std::string& error)
{
  const YAML::Node node = parent[field];
  if (!node || !node.IsSequence() || node.size() != 3)
  {
    error = std::string("字段 '") + field + "' 必须是含 3 个数值的数组。";
    return false;
  }
  for (std::size_t index = 0; index < value.size(); ++index)
  {
    value[index] = node[index].as<double>();
  }
  return true;
}

bool readPose(
  const YAML::Node& parent,
  const char* field,
  geometry_msgs::msg::Pose& pose,
  std::string& error)
{
  const YAML::Node node = parent[field];
  if (!node || !node.IsMap())
  {
    error = std::string("字段 '") + field + "' 必须是 pose map。";
    return false;
  }

  std::array<double, 3> position{};
  if (!readVector3(node, "position", position, error))
  {
    error = std::string("pose.") + error;
    return false;
  }
  pose.position.x = position[0];
  pose.position.y = position[1];
  pose.position.z = position[2];

  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  const YAML::Node orientation = node["orientation"];
  if (orientation)
  {
    if (!orientation.IsSequence() || orientation.size() != 4)
    {
      error = "pose.orientation 必须是 [x, y, z, w]。";
      return false;
    }
    pose.orientation.x = orientation[0].as<double>();
    pose.orientation.y = orientation[1].as<double>();
    pose.orientation.z = orientation[2].as<double>();
    pose.orientation.w = orientation[3].as<double>();
  }
  const double norm = std::sqrt(
    pose.orientation.x * pose.orientation.x +
    pose.orientation.y * pose.orientation.y +
    pose.orientation.z * pose.orientation.z +
    pose.orientation.w * pose.orientation.w);
  if (norm < 1.0e-8)
  {
    error = "pose.orientation 不能是零四元数。";
    return false;
  }
  pose.orientation.x /= norm;
  pose.orientation.y /= norm;
  pose.orientation.z /= norm;
  pose.orientation.w /= norm;
  return true;
}

bool readStringSequence(
  const YAML::Node& parent,
  const char* field,
  std::vector<std::string>& values,
  std::string& error)
{
  values.clear();
  const YAML::Node node = parent[field];
  if (!node)
  {
    return true;
  }
  if (!node.IsSequence())
  {
    error = std::string("字段 '") + field + "' 必须是字符串数组。";
    return false;
  }
  for (const auto& value : node)
  {
    values.push_back(value.as<std::string>());
  }
  return true;
}

bool readBox(const YAML::Node& node, BoxSpec& box, std::string& error)
{
  if (!node.IsMap() || !node["id"])
  {
    error = "boxes 中每一项必须有 id。";
    return false;
  }
  box.id = node["id"].as<std::string>();
  if (!readVector3(node, "dimensions", box.dimensions, error) ||
      !readPose(node, "initial_pose", box.initial_pose, error))
  {
    error = "Box '" + box.id + "': " + error;
    return false;
  }
  if (!node["mass"])
  {
    error = "Box '" + box.id + "' 缺少 mass。";
    return false;
  }
  box.mass = node["mass"].as<double>();
  box.payload_class = node["payload_class"] ? node["payload_class"].as<std::string>() : "unspecified";
  if (!readStringSequence(node, "allowed_modes", box.allowed_modes, error))
  {
    error = "Box '" + box.id + "': " + error;
    return false;
  }

  box.grasp_candidates.clear();
  const YAML::Node grasps = node["grasp_candidates"];
  if (!grasps || !grasps.IsSequence() || grasps.size() == 0)
  {
    error = "Box '" + box.id + "' 至少需要一个 grasp_candidates。";
    return false;
  }
  for (const auto& grasp_node : grasps)
  {
    if (!grasp_node.IsMap() || !grasp_node["id"])
    {
      error = "Box '" + box.id + "' 的 grasp candidate 缺少 id。";
      return false;
    }
    GraspCandidate grasp;
    grasp.id = grasp_node["id"].as<std::string>();
    if (!readPose(grasp_node, "local_pose", grasp.local_pose, error))
    {
      error = "Box '" + box.id + "', grasp '" + grasp.id + "': " + error;
      return false;
    }
    box.grasp_candidates.push_back(std::move(grasp));
  }
  return true;
}

bool readPlacement(const YAML::Node& node, PlacementSpec& placement, std::string& error)
{
  if (!node.IsMap() || !node["id"] || !node["object_id"] || !node["support_surface_id"] ||
      !node["support_height"])
  {
    error = "placements 中每一项必须有 id/object_id/support_surface_id/support_height。";
    return false;
  }
  placement.id = node["id"].as<std::string>();
  placement.object_id = node["object_id"].as<std::string>();
  placement.support_surface_id = node["support_surface_id"].as<std::string>();
  placement.support_height = node["support_height"].as<double>();
  placement.orientation_tolerance_rad = node["orientation_tolerance_rad"] ?
    node["orientation_tolerance_rad"].as<double>() : 0.0;
  if (!readPose(node, "target_pose", placement.target_pose, error))
  {
    error = "Placement '" + placement.id + "': " + error;
    return false;
  }
  return true;
}

}  // namespace

const BoxSpec* PalletizingJob::findBox(const std::string& requested_id) const
{
  const auto iterator = std::find_if(
    boxes.begin(), boxes.end(), [&requested_id](const BoxSpec& box)
    {
      return box.id == requested_id;
    });
  return iterator == boxes.end() ? nullptr : &(*iterator);
}

const PlacementSpec* PalletizingJob::findPlacement(const std::string& requested_id) const
{
  const auto iterator = std::find_if(
    placements.begin(), placements.end(), [&requested_id](const PlacementSpec& placement)
    {
      return placement.id == requested_id;
    });
  return iterator == placements.end() ? nullptr : &(*iterator);
}

moveit_msgs::msg::CollisionObject makeCollisionObject(
  const BoxSpec& box,
  const geometry_msgs::msg::Pose& pose,
  int operation,
  const std::string& frame_id)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = frame_id;
  object.id = box.id;
  shape_msgs::msg::SolidPrimitive shape;
  shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  shape.dimensions = {box.dimensions[0], box.dimensions[1], box.dimensions[2]};
  object.primitives.push_back(shape);
  object.primitive_poses.push_back(pose);
  object.operation = operation;
  return object;
}

bool validatePalletizingJob(const PalletizingJob& job, std::string& error)
{
  if (job.id.empty() || job.boxes.empty())
  {
    error = "job_id 和 boxes 均不能为空。";
    return false;
  }
  if (job.pallet_region.frame_id.empty() ||
      job.pallet_region.min_x >= job.pallet_region.max_x ||
      job.pallet_region.min_y >= job.pallet_region.max_y)
  {
    error = "pallet_region 边界或 frame_id 无效。";
    return false;
  }

  std::set<std::string> ids;
  for (const auto& box : job.boxes)
  {
    if (box.id.empty() || !ids.insert(box.id).second)
    {
      error = "Box id 为空或重复：'" + box.id + "'。";
      return false;
    }
    if (box.mass <= 0.0 ||
        std::any_of(box.dimensions.begin(), box.dimensions.end(),
          [](double dimension) { return dimension <= 0.0 || !std::isfinite(dimension); }))
    {
      error = "Box '" + box.id + "' 的 mass/dimensions 必须为有限正数。";
      return false;
    }
    if (box.grasp_candidates.empty())
    {
      error = "Box '" + box.id + "' 缺少抓取候选。";
      return false;
    }
    std::set<std::string> grasp_ids;
    for (const auto& grasp : box.grasp_candidates)
    {
      if (grasp.id.empty() || !grasp_ids.insert(grasp.id).second)
      {
        error = "Box '" + box.id + "' 的 grasp id 为空或重复。";
        return false;
      }
    }
  }

  std::set<std::string> placement_ids;
  for (const auto& placement : job.placements)
  {
    if (placement.id.empty() || !placement_ids.insert(placement.id).second ||
        job.findBox(placement.object_id) == nullptr ||
        placement.support_surface_id.empty() || !std::isfinite(placement.support_height) ||
        placement.orientation_tolerance_rad < 0.0)
    {
      error = "Placement '" + placement.id + "' 无效或引用了未知 Box。";
      return false;
    }
  }
  return true;
}

bool loadPalletizingJobYaml(
  const std::string& path,
  PalletizingJob& job,
  std::string& error)
{
  try
  {
    const YAML::Node root = YAML::LoadFile(path);
    if (!root.IsMap() || !root["job_id"] || !root["pallet_region"] || !root["boxes"])
    {
      error = "YAML 必须包含 job_id、pallet_region 和 boxes。";
      return false;
    }
    job = PalletizingJob{};
    job.id = root["job_id"].as<std::string>();

    const YAML::Node region = root["pallet_region"];
    if (!region.IsMap())
    {
      error = "pallet_region 必须是 map。";
      return false;
    }
    job.pallet_region.frame_id = region["frame_id"] ? region["frame_id"].as<std::string>() : "world";
    for (const auto* field : {"min_x", "max_x", "min_y", "max_y", "support_height"})
    {
      if (!region[field])
      {
        error = std::string("pallet_region 缺少 ") + field;
        return false;
      }
    }
    job.pallet_region.min_x = region["min_x"].as<double>();
    job.pallet_region.max_x = region["max_x"].as<double>();
    job.pallet_region.min_y = region["min_y"].as<double>();
    job.pallet_region.max_y = region["max_y"].as<double>();
    job.pallet_region.support_height = region["support_height"].as<double>();

    const YAML::Node boxes = root["boxes"];
    if (!boxes.IsSequence())
    {
      error = "boxes 必须是数组。";
      return false;
    }
    for (const auto& node : boxes)
    {
      BoxSpec box;
      if (!readBox(node, box, error))
      {
        return false;
      }
      job.boxes.push_back(std::move(box));
    }

    const YAML::Node placements = root["placements"];
    if (placements)
    {
      if (!placements.IsSequence())
      {
        error = "placements 必须是数组。";
        return false;
      }
      for (const auto& node : placements)
      {
        PlacementSpec placement;
        if (!readPlacement(node, placement, error))
        {
          return false;
        }
        job.placements.push_back(std::move(placement));
      }
    }
    return validatePalletizingJob(job, error);
  }
  catch (const YAML::Exception& exception)
  {
    error = "读取 YAML 失败：" + std::string(exception.what());
    return false;
  }
  catch (const std::exception& exception)
  {
    error = "读取 Job 失败：" + std::string(exception.what());
    return false;
  }
}

}  // namespace fr3_dual_palletize
