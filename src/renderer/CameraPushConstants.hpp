#pragma once

#include "Matrix.hpp"
#include <cstdint>

struct CameraPushConstants {
  Mat4 viewProjection = identity4();
};

struct ObjectPushConstants {
  Mat4 model = identity4();
  float tint[4] = {1.0f, 1.0f, 1.0f, 1.0f};
};

inline constexpr uint32_t cameraPushConstantSize() {
  return sizeof(CameraPushConstants);
}

inline constexpr uint32_t objectPushConstantSize() {
  return sizeof(ObjectPushConstants);
}
