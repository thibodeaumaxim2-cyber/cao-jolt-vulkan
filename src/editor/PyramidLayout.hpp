#pragma once

#include "StructuralPrimitive.hpp"
#include <vector>

inline std::vector<StructuralPrimitive> makeCubePyramid(int baseWidth,
                                                        float cubeSize) {
  std::vector<StructuralPrimitive> result;
  uint32_t id = 1;
  for (int layer = 0; layer < baseWidth; ++layer) {
    const int width = baseWidth - layer;
    const float offset = -0.5f * static_cast<float>(width - 1) * cubeSize;
    for (int row = 0; row < width; ++row)
      for (int col = 0; col < width; ++col) {
        StructuralPrimitive cube;
        cube.id = id++;
        cube.type = StructuralPrimitiveType::Cube;
        cube.position = {offset + col * cubeSize,
                         -0.5f + layer * cubeSize,
                         offset + row * cubeSize};
        cube.scale = {cubeSize, cubeSize, cubeSize};
        result.push_back(cube);
      }
  }
  return result;
}
