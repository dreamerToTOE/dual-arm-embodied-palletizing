#include "fr3_dual_palletize/runtime_box_state.hpp"

#include <cmath>

#include <geometry_msgs/msg/vector3.hpp>

namespace fr3_dual_palletize
{
namespace
{

bool normalizeQuaternion(geometry_msgs::msg::Quaternion& quaternion)
{
  const double norm = std::sqrt(
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  if (!std::isfinite(norm) || norm < 1.0e-9)
  {
    return false;
  }
  quaternion.x /= norm;
  quaternion.y /= norm;
  quaternion.z /= norm;
  quaternion.w /= norm;
  return true;
}

geometry_msgs::msg::Quaternion multiplyQuaternion(
  const geometry_msgs::msg::Quaternion& first,
  const geometry_msgs::msg::Quaternion& second)
{
  geometry_msgs::msg::Quaternion result;
  result.x = first.w * second.x + first.x * second.w + first.y * second.z - first.z * second.y;
  result.y = first.w * second.y - first.x * second.z + first.y * second.w + first.z * second.x;
  result.z = first.w * second.z + first.x * second.y - first.y * second.x + first.z * second.w;
  result.w = first.w * second.w - first.x * second.x - first.y * second.y - first.z * second.z;
  return result;
}

geometry_msgs::msg::Vector3 rotateVector(
  const geometry_msgs::msg::Quaternion& quaternion,
  const geometry_msgs::msg::Vector3& vector)
{
  // q * (v, 0) * q^-1 的展开形式。输入 q 已由调用者单位化。
  const double xx = quaternion.x * quaternion.x;
  const double yy = quaternion.y * quaternion.y;
  const double zz = quaternion.z * quaternion.z;
  const double xy = quaternion.x * quaternion.y;
  const double xz = quaternion.x * quaternion.z;
  const double yz = quaternion.y * quaternion.z;
  const double wx = quaternion.w * quaternion.x;
  const double wy = quaternion.w * quaternion.y;
  const double wz = quaternion.w * quaternion.z;

  geometry_msgs::msg::Vector3 result;
  result.x = (1.0 - 2.0 * (yy + zz)) * vector.x + 2.0 * (xy - wz) * vector.y +
    2.0 * (xz + wy) * vector.z;
  result.y = 2.0 * (xy + wz) * vector.x + (1.0 - 2.0 * (xx + zz)) * vector.y +
    2.0 * (yz - wx) * vector.z;
  result.z = 2.0 * (xz - wy) * vector.x + 2.0 * (yz + wx) * vector.y +
    (1.0 - 2.0 * (xx + yy)) * vector.z;
  return result;
}

}  // namespace

bool boxStateToBoxSpec(const msg::BoxState& state, BoxSpec& box, std::string& error)
{
  if (state.id.empty())
  {
    error = "BoxState.id 不能为空。";
    return false;
  }
  for (const double dimension : state.size)
  {
    if (!std::isfinite(dimension) || dimension <= 0.0)
    {
      error = "BoxState '" + state.id + "' 的 size 必须为有限正数。";
      return false;
    }
  }
  if (!std::isfinite(state.mass) || state.mass <= 0.0)
  {
    error = "BoxState '" + state.id + "' 的 mass 必须为有限正数。";
    return false;
  }
  if (state.grasp_candidate_ids.empty() ||
      state.grasp_candidate_ids.size() != state.grasp_candidates.size())
  {
    error = "BoxState '" + state.id + "' 的 grasp id/pose 数量不一致或为空。";
    return false;
  }

  box.id = state.id;
  box.dimensions = {state.size[0], state.size[1], state.size[2]};
  box.mass = state.mass;
  box.initial_pose = state.pose;
  if (!normalizeQuaternion(box.initial_pose.orientation))
  {
    error = "BoxState '" + state.id + "' 的世界四元数无效。";
    return false;
  }
  box.payload_class = state.payload_class;
  box.allowed_modes.assign(state.allowed_modes.begin(), state.allowed_modes.end());
  box.grasp_candidates.clear();
  box.grasp_candidates.reserve(state.grasp_candidates.size());
  for (std::size_t index = 0; index < state.grasp_candidates.size(); ++index)
  {
    GraspCandidate grasp;
    grasp.id = state.grasp_candidate_ids[index];
    grasp.local_pose = state.grasp_candidates[index];
    if (grasp.id.empty() || !normalizeQuaternion(grasp.local_pose.orientation))
    {
      error = "BoxState '" + state.id + "' 的 grasp candidate 无效。";
      return false;
    }
    box.grasp_candidates.push_back(std::move(grasp));
  }
  return true;
}

geometry_msgs::msg::Pose composePose(
  const geometry_msgs::msg::Pose& parent,
  const geometry_msgs::msg::Pose& local)
{
  geometry_msgs::msg::Quaternion parent_rotation = parent.orientation;
  geometry_msgs::msg::Quaternion local_rotation = local.orientation;
  // 运行时输入已验证；这里仍做安全退化，避免意外零四元数传播 NaN 到 MoveIt。
  if (!normalizeQuaternion(parent_rotation))
  {
    parent_rotation.w = 1.0;
  }
  if (!normalizeQuaternion(local_rotation))
  {
    local_rotation.w = 1.0;
  }

  geometry_msgs::msg::Vector3 offset;
  offset.x = local.position.x;
  offset.y = local.position.y;
  offset.z = local.position.z;
  const auto rotated_offset = rotateVector(parent_rotation, offset);

  geometry_msgs::msg::Pose result;
  result.position.x = parent.position.x + rotated_offset.x;
  result.position.y = parent.position.y + rotated_offset.y;
  result.position.z = parent.position.z + rotated_offset.z;
  result.orientation = multiplyQuaternion(parent_rotation, local_rotation);
  normalizeQuaternion(result.orientation);
  return result;
}

}  // namespace fr3_dual_palletize
