#pragma once

#include "renderer/Matrix.hpp"
#include <cstdint>

struct PhysicsTransform {
  uint32_t objectId = 0;
  Mat4 model = identity4();
  bool active = true;
};

inline PhysicsTransform makePhysicsTransform(uint32_t objectId,
                                             const Mat4& model) {
  PhysicsTransform result;
  result.objectId = objectId;
  result.model = model;
  return result;
}
