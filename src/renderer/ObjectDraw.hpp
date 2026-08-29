#pragma once

#include "Matrix.hpp"
#include <cstdint>

struct ObjectDrawData {
  Mat4 model = Mat4::identity();
  float tint[4] = {1.0f, 1.0f, 1.0f, 1.0f};
  uint32_t firstIndex = 0;
  uint32_t indexCount = 0;
  int32_t vertexOffset = 0;
  uint32_t objectId = 0;
};

inline ObjectDrawData makeObjectDraw(uint32_t objectId,
                                     const Mat4& model,
                                     uint32_t firstIndex,
                                     uint32_t indexCount) {
  ObjectDrawData draw;
  draw.objectId = objectId;
  draw.model = model;
  draw.firstIndex = firstIndex;
  draw.indexCount = indexCount;
  return draw;
}
