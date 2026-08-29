#pragma once

#include "Camera.hpp"
#include <cstdint>

enum class PhysicsBodyKind { Static, Dynamic, Kinematic };

struct PhysicsBodyDescriptor {
  uint32_t objectId = 0;
  PhysicsBodyKind kind = PhysicsBodyKind::Dynamic;
  Vec3 position{};
  Vec3 halfExtent{0.1f, 0.1f, 0.1f};
  float mass = 1.0f;
  float restitution = 0.05f;
  float friction = 0.65f;
};
