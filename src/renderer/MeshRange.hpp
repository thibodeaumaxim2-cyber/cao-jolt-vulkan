#pragma once

#include <cstdint>

struct MeshRange {
  uint32_t firstIndex = 0;
  uint32_t indexCount = 0;
  int32_t vertexOffset = 0;

  bool valid() const { return indexCount != 0; }
};
