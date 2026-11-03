#pragma once

#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace task28_b0
{
constexpr std::int64_t kMaxAgeNs = 50000000;
constexpr std::int64_t kMaxCaptureNs = 50000000;

struct Snapshot
{
  std::uint64_t step{0};
  double sim_s{0};
  std::int64_t started_ns{0}, finished_ns{0};
  std::array<double, 3> cube{};
  std::array<double, 4> quat_wxyz{};
  std::array<double, 7> left{}, right{};
};

template <std::size_t N>
bool finiteArray(const YAML::Node& node, std::array<double, N>& out)
{
  if (!node.IsSequence() || node.size() != N) return false;
  for (std::size_t i = 0; i < N; ++i)
  {
    out[i] = node[i].as<double>();
    if (!std::isfinite(out[i])) return false;
  }
  return true;
}

inline bool decode(const std::string& data, Snapshot* out)
{
  if (!out) return false;
  try
  {
    const auto n = YAML::Load(data);
    if (n["version"].as<int>() != 1 || n["frame"].as<std::string>() != "world" ||
        n["source"].as<std::string>() != "post_step_physx_tensor" || !n["handle_valid"].as<bool>())
      return false;
    Snapshot s;
    s.step = n["step"].as<std::uint64_t>();
    s.sim_s = n["simulation_time_s"].as<double>();
    s.started_ns = n["sample_started_ns"].as<std::int64_t>();
    s.finished_ns = n["sample_finished_ns"].as<std::int64_t>();
    if (!s.step || !std::isfinite(s.sim_s) || s.sim_s <= 0 || s.started_ns <= 0 ||
        s.finished_ns < s.started_ns || s.finished_ns - s.started_ns > kMaxCaptureNs ||
        !finiteArray(n["cube_xyz_m"], s.cube) || !finiteArray(n["cube_quaternion_wxyz"], s.quat_wxyz) ||
        !finiteArray(n["left_q_rad"], s.left) || !finiteArray(n["right_q_rad"], s.right))
      return false;
    double qnorm = 0;
    for (double q : s.quat_wxyz) qnorm += q * q;
    if (qnorm < .998001 || qnorm > 1.002001) return false;
    *out = s;
    return true;
  }
  catch (const YAML::Exception&) { return false; }
}

inline bool initialPositions(const Snapshot& s, const std::vector<std::string>& left_names,
                             const std::vector<std::string>& right_names,
                             std::vector<double>* left, std::vector<double>* right)
{
  if (!left || !right || left_names.size() != 7 || right_names.size() != 7) return false;
  for (std::size_t i = 0; i < 7; ++i)
    if (left_names[i] != "left_fr3_joint" + std::to_string(i + 1) ||
        right_names[i] != "right_fr3_joint" + std::to_string(i + 1)) return false;
  left->assign(s.left.begin(), s.left.end());
  right->assign(s.right.begin(), s.right.end());
  return true;
}

inline bool fresh(const Snapshot& s, std::int64_t now_ns, std::int64_t after_ns,
                  const Snapshot* previous = nullptr)
{
  return s.started_ns >= after_ns && s.finished_ns <= now_ns &&
    now_ns - s.finished_ns <= kMaxAgeNs &&
    (!previous || (s.step > previous->step && s.sim_s > previous->sim_s &&
                   s.started_ns > previous->finished_ns));
}
}  // namespace task28_b0
