#include <cmath>
#include <fcl/fcl.h>
#include <iostream>
#include <memory>

// One of the original 10 mm-radius, 10 mm-long cup cylinders and the 120 mm
// Cube. The left helper's cup normal is +Y; the +Y wall face is at y=0.303 m.
int main() {
  auto cube_geometry = std::make_shared<fcl::Boxd>(.12, .12, .12);
  auto cup_geometry = std::make_shared<fcl::Cylinderd>(.01, .01);
  auto check = [&](double cube_y, double tcp_y) {
    fcl::Transform3d cube_transform = fcl::Transform3d::Identity();
    cube_transform.translation() = Eigen::Vector3d(0, cube_y, .26);
    fcl::Transform3d cup_transform = fcl::Transform3d::Identity();
    cup_transform.translation() = Eigen::Vector3d(.024, tcp_y - .005, .284);
    cup_transform.linear() =
        Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitX())
            .toRotationMatrix();
    fcl::CollisionObjectd cube(cube_geometry, cube_transform),
        cup(cup_geometry, cup_transform);
    fcl::CollisionRequestd request(10, true);
    fcl::CollisionResultd result;
    fcl::collide(&cup, &cube, request, result);
    double depth = 0;
    std::vector<fcl::Contactd> contacts;
    result.getContacts(contacts);
    for (const auto &contact : contacts)
      depth = std::max(depth, contact.penetration_depth);
    return std::make_pair(result.isCollision(), depth);
  };
  constexpr double cube_half = .060, command_gap = .001, overtravel = .004;
  const auto entry = check(.060, .060 - cube_half - command_gap);
  const auto wall = check(.243, .243 + overtravel - cube_half - command_gap);
  std::cout << "entry_collision=" << entry.first
            << " endpoint_collision=" << wall.first
            << " endpoint_depth_mm=" << wall.second * 1000 << '\n';
  if (entry.first || !wall.first || std::abs(wall.second - .003) > .00005)
    return 1;
  return 0;
}
