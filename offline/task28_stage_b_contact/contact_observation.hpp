#pragma once
#include <array>
#include <cmath>
#include <cstdint>

// An observation gate only; no actuator command or force controller is implemented.
enum class ForceSource
{
  Missing,
  MeasuredContact,
  IndependentlyValidatedEstimate,
  JointTorque,
  ModelOnly
};
struct ContactObservation
{
  bool state_valid{false};
  bool force_observed{false};
  bool force_source_validated{false};
  bool world_frame{false};
  ForceSource force_source{ForceSource::Missing};
  std::uint64_t state_step{0}, force_step{0};
  std::array<double, 3> normal_world{}, force_world{};
};

inline bool contactObservationUsable(const ContactObservation &s)
{
  if (!s.state_valid || !s.force_observed || !s.force_source_validated || !s.world_frame ||
      s.state_step == 0 || s.state_step != s.force_step ||
      (s.force_source != ForceSource::MeasuredContact &&
       s.force_source != ForceSource::IndependentlyValidatedEstimate))
    return false;
  double n2 = 0.0;
  for (std::size_t i = 0; i < 3; ++i)
  {
    if (!std::isfinite(s.normal_world[i]) || !std::isfinite(s.force_world[i]))
      return false;
    n2 += s.normal_world[i] * s.normal_world[i];
  }
  return n2 >= .998001 && n2 <= 1.002001;
}
