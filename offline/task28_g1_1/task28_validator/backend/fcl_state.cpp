// G1-1: bounded, single-state collision queries; never a ROS/Isaac executable.
#include <fcl/fcl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <map>
#include <array>
#include <stdexcept>
#include <string>
#include <vector>

using Geometry = fcl::CollisionGeometryd;

// Frozen conservative hulls are binary STL in metres. No external mesh loader.
std::shared_ptr<Geometry> mesh(const std::string& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("mesh missing: " + path);
  char header[80]; input.read(header, 80);
  std::uint32_t count = 0; input.read(reinterpret_cast<char*>(&count), 4);
  if (!input || count == 0 || count > 1000000) throw std::runtime_error("invalid binary STL");
  auto vertices = std::make_shared<std::vector<fcl::Vector3d>>();
  auto faces = std::make_shared<std::vector<int>>();
  std::map<std::array<float,3>,int> ids;
  vertices->reserve(count * 3); faces->reserve(count * 4);
  for (std::uint32_t i = 0; i < count; ++i) {
    float record[12]; std::uint16_t attribute;
    input.read(reinterpret_cast<char*>(record), 48);
    input.read(reinterpret_cast<char*>(&attribute), 2);
    if (!input) throw std::runtime_error("truncated binary STL");
    faces->push_back(3);
    for (int j = 0; j < 3; ++j) {
      std::array<float,3> key{{record[3+j*3],record[4+j*3],record[5+j*3]}};
      auto it = ids.find(key);
      if (it == ids.end()) {
        int index = vertices->size(); ids[key] = index;
        vertices->emplace_back(key[0],key[1],key[2]); faces->push_back(index);
      } else faces->push_back(it->second);
    }
  }
  // The unchanged fixture STLs are proven closed convex hulls. Treating them as
  // solids avoids triangle-surface-only containment misses for an enclosed Cube.
  return std::make_shared<fcl::Convexd>(vertices, count, faces, true);
}

int main() {
  try {
    std::size_t count, pair_count;
    if (!(std::cin >> count >> pair_count) || count > 1000 || pair_count > 1000000)
      throw std::runtime_error("invalid scene header");
    std::vector<std::shared_ptr<Geometry>> geometries;
    std::vector<fcl::CollisionObjectd> objects;
    geometries.reserve(count); objects.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
      char kind; std::cin >> kind;
      std::shared_ptr<Geometry> geometry;
      if (kind == 'B') { double x,y,z; std::cin >> x >> y >> z; geometry = std::make_shared<fcl::Boxd>(x,y,z); }
      else if (kind == 'C') { double r,l; std::cin >> r >> l; geometry = std::make_shared<fcl::Cylinderd>(r,l); }
      else if (kind == 'M') { std::string path; std::cin >> path; geometry = mesh(path); }
      else throw std::runtime_error("unsupported shape");
      fcl::Transform3d pose = fcl::Transform3d::Identity();
      for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 4; ++col) std::cin >> pose.matrix()(row,col);
      if (!std::cin || !pose.matrix().allFinite()) throw std::runtime_error("invalid pose");
      geometries.push_back(geometry); objects.emplace_back(geometry, pose);
    }
    std::cout << std::setprecision(17) << "{\"backend\":\"FCL-0.7.0\",\"pairs\":[";
    bool first = true;
    std::size_t checked_count = 0;
    for (std::size_t i = 0; i < pair_count; ++i) {
      std::size_t a,b; std::cin >> a >> b;
      if (!std::cin || a >= count || b >= count || a == b) throw std::runtime_error("invalid pair");
      fcl::CollisionRequestd request(128, true);
      fcl::CollisionResultd result;
      fcl::collide(&objects[a], &objects[b], request, result);
      ++checked_count;
      if (!result.isCollision()) continue;
      if (!first) std::cout << ','; first = false;
      std::cout << "{\"a\":" << a << ",\"b\":" << b << ",\"contacts\":[";
      std::vector<fcl::Contactd> contacts; result.getContacts(contacts);
      for (std::size_t j = 0; j < contacts.size(); ++j) {
        if (j) std::cout << ',';
        const auto& c = contacts[j];
        std::cout << "{\"depth_m\":" << c.penetration_depth << ",\"normal_world\":["
          << c.normal[0] << ',' << c.normal[1] << ',' << c.normal[2] << "],\"position_world_m\":["
          << c.pos[0] << ',' << c.pos[1] << ',' << c.pos[2] << "]}";
      }
      std::cout << "]}";
    }
    // Completion receipts are written only after every requested query ran.
    std::cout << "],\"shape_count\":" << objects.size()
              << ",\"checked_pair_count\":" << checked_count << "}\n";
    return 0;
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 2; }
}
