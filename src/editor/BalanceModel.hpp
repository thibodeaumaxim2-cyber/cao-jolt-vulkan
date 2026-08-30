#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace CaoBalance {

struct Point { float x = 0.0f; float z = 0.0f; };

inline float cross(Point a, Point b, Point c) {
  return (b.x-a.x)*(c.z-a.z) - (b.z-a.z)*(c.x-a.x);
}

inline bool insideTriangle(Point p, std::array<Point,3> t, float margin = 0.0f) {
  const float a = cross(t[0], t[1], p);
  const float b = cross(t[1], t[2], p);
  const float c = cross(t[2], t[0], p);
  const bool positive = a >= margin && b >= margin && c >= margin;
  const bool negative = a <= -margin && b <= -margin && c <= -margin;
  return positive || negative;
}

inline Point centerOfMass(const std::array<Point,4>& positions,
                          const std::array<float,4>& masses) {
  float total = 0.0f, x = 0.0f, z = 0.0f;
  for (size_t i=0; i<4; ++i) {
    total += masses[i]; x += positions[i].x*masses[i]; z += positions[i].z*masses[i];
  }
  return total > 0.0f ? Point{x/total,z/total} : Point{};
}

inline Point correctionToward(Point com, Point polygonCenter, float gain = 0.35f) {
  return {std::clamp((polygonCenter.x-com.x)*gain, -0.12f, 0.12f),
          std::clamp((polygonCenter.z-com.z)*gain, -0.12f, 0.12f)};
}

} // namespace CaoBalance
