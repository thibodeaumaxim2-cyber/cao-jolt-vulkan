#pragma once

#include "Camera.hpp"
#include <cstdint>

enum class StructuralPrimitiveType { Cube, CompressionBeam, DyneemaCable };

struct StructuralPrimitive {
  uint32_t id = 0;
  StructuralPrimitiveType type = StructuralPrimitiveType::Cube;
  Vec3 position{};
  Vec3 rotation{};
  Vec3 scale{1.0f, 1.0f, 1.0f};
  float mass = 1.0f;
  bool selected = false;
};
