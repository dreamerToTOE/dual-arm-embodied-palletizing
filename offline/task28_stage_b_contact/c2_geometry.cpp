#include <fcl/fcl.h>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

using Object = fcl::CollisionObjectd;
using Geometry = std::shared_ptr<fcl::CollisionGeometryd>;

static bool hit(const Geometry& a, const fcl::Transform3d& ta,
                const Geometry& b, const fcl::Transform3d& tb) {
  Object x(a, ta), y(b, tb);
  fcl::CollisionRequestd request;
  fcl::CollisionResultd result;
  return fcl::collide(&x, &y, request, result) != 0;
}

int main() {
  const Geometry cube = std::make_shared<fcl::Boxd>(.12, .12, .12);
  const Geometry table = std::make_shared<fcl::Boxd>(1.5, .8, .2);
  const Geometry cup = std::make_shared<fcl::Cylinderd>(.01, .01);
  struct Shape { Geometry geom; Eigen::Vector3d local; bool cylindrical; };
  std::vector<Shape> shapes = {
    {std::make_shared<fcl::Boxd>(.028, .028, .080), {0, 0, .040}, false},
    {std::make_shared<fcl::Boxd>(.018, .130, .018), {0, -.065, .080}, false},
    {std::make_shared<fcl::Boxd>(.070, .014, .070), {0, -.138, .080}, false},
  };
  for (double x : {-.024, .024}) for (double z : {.056, .104})
    shapes.push_back({cup, {x, -.150, z}, true});
  fcl::Transform3d cube_pose = fcl::Transform3d::Identity(), table_pose = cube_pose;
  cube_pose.translation() = Eigen::Vector3d(.5, 0., .26);
  table_pose.translation() = Eigen::Vector3d(.55, 0., .10);
  const Eigen::Matrix3d tool_rotation = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const Eigen::Matrix3d cylinder_rotation = Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitX()).toRotationMatrix();
  auto collides = [&](double dy) {
    for (const auto& shape : shapes) {
      fcl::Transform3d t = fcl::Transform3d::Identity();
      t.translation() = Eigen::Vector3d(.5, -.218 + dy, .34) + tool_rotation * shape.local;
      t.linear() = tool_rotation * (shape.cylindrical ? cylinder_rotation : Eigen::Matrix3d::Identity().eval());
      if (hit(shape.geom, t, cube, cube_pose) || hit(shape.geom, t, table, table_pose)) return true;
    }
    return false;
  };
  for (int i = 0; i <= 200; ++i) {
    const double dy = -.010 + .0129 * i / 200.;
    if (collides(dy)) { std::cerr << "precontact path collision at " << dy << '\n'; return 1; }
  }
  if (!collides(.004)) { std::cerr << "penetrating endpoint was not rejected\n"; return 1; }
  std::cout << "C2 FCL full seven-shape tool, 201 precontact/retreat samples PASS; penetrating endpoint rejected\n";
}
