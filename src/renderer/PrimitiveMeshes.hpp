#pragma once

#include "Vertex.hpp"
#include <cstdint>
#include <vector>

struct PrimitiveMesh {
  std::vector<CadVertex> vertices;
  std::vector<uint32_t> indices;
};

inline PrimitiveMesh makeCubeMesh(float halfExtent = 0.5f) {
  const float h = halfExtent;
  PrimitiveMesh mesh;
  mesh.vertices = {
      {{-h, -h, -h}, {0.15f, 0.65f, 0.95f}},
      {{ h, -h, -h}, {0.20f, 0.75f, 1.00f}},
      {{ h,  h, -h}, {0.35f, 0.85f, 1.00f}},
      {{-h,  h, -h}, {0.10f, 0.55f, 0.90f}},
      {{-h, -h,  h}, {0.20f, 0.80f, 0.75f}},
      {{ h, -h,  h}, {0.25f, 0.90f, 0.85f}},
      {{ h,  h,  h}, {0.45f, 1.00f, 0.95f}},
      {{-h,  h,  h}, {0.15f, 0.70f, 0.80f}},
  };
  mesh.indices = {
      0, 1, 2, 2, 3, 0,
      4, 6, 5, 6, 4, 7,
      0, 4, 5, 5, 1, 0,
      3, 2, 6, 6, 7, 3,
      1, 5, 6, 6, 2, 1,
      0, 3, 7, 7, 4, 0,
  };
  return mesh;
}

inline PrimitiveMesh makePyramidMesh(float halfExtent = 0.5f,
                                     float height = 1.0f) {
  const float h = halfExtent;
  const float y = height * 0.5f;
  PrimitiveMesh mesh;
  mesh.vertices = {
      {{-h, -y, -h}, {0.90f, 0.35f, 0.12f}},
      {{ h, -y, -h}, {1.00f, 0.45f, 0.10f}},
      {{ h, -y,  h}, {1.00f, 0.65f, 0.12f}},
      {{-h, -y,  h}, {0.85f, 0.30f, 0.10f}},
      {{0.0f, y, 0.0f}, {1.00f, 0.90f, 0.25f}},
  };
  mesh.indices = {
      0, 1, 2, 2, 3, 0,
      0, 4, 1,
      1, 4, 2,
      2, 4, 3,
      3, 4, 0,
  };
  return mesh;
}
