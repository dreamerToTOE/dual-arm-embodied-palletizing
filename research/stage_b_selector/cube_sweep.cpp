#include <fcl/fcl.h>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using Object = fcl::CollisionObjectd;
std::shared_ptr<Object> box(double x, double y, double z, double sx, double sy, double sz, double yaw = 0)
{
  fcl::Transform3d transform = fcl::Transform3d::Identity();
  transform.translation() = fcl::Vector3d(x, y, z);
  transform.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return std::make_shared<Object>(std::make_shared<fcl::Boxd>(sx, sy, sz), transform);
}
bool collides(const Object& a, const Object& b)
{
  fcl::CollisionRequestd request;
  fcl::CollisionResultd result;
  fcl::collide(&a, &b, request, result);
  return result.isCollision();
}
int main()
{
  // Task28 pre-contact scene translated from the planning frame to world (+0.100 m X).
  const std::vector<std::shared_ptr<Object>> world = {
    box(.55, 0, .1, 1.5, .8, .2), box(1.17, 0, .275, .02, .646, .15),
    box(1.035, -.313, .275, .29, .02, .15), box(1.035, .313, .275, .29, .02, .15)};
  if (collides(*box(1.08, 0, .26007622480392456, .12, .12, .12), *world[1]) ||
      !collides(*box(1.097, 0, .26007622480392456, .12, .12, .12, 6 * M_PI / 180), *world[1]))
    return 2;
  std::cout << "case,initial_world_collision,first_collision_sample,first_collision_object,terminal_world_collision\n";
  std::string name;
  double x, y, z, yaw;
  const char* names[] = {"table", "deep_wall", "minus_y_wall", "plus_y_wall"};
  while (std::cin >> name >> x >> y >> z >> yaw)
  {
    int first = -1, object = -1;
    bool terminal = false;
    for (int i = 0; i <= 100; ++i)
    {
      auto cube = box(x + (1.097 - x) * i / 100., y, z, .12, .12, .12, yaw);
      bool hit = false;
      for (int j = 0; j < 4; ++j)
        if (collides(*cube, *world[j]))
        {
          hit = true;
          if (first < 0) { first = i; object = j; }
        }
      if (i == 100) terminal = hit;
    }
    std::cout << name << ',' << (first == 0) << ',' << first << ','
              << (object < 0 ? "none" : names[object]) << ',' << terminal << '\n';
  }
  return std::cin.eof() ? 0 : 1;
}
