#pragma once

#include <Jolt/Physics/Body/BodyID.h>
#include <cstdint>

struct JoltBodyRecord {
  uint32_t sceneObjectId = 0;
  JPH::BodyID bodyId;
  bool dynamic = false;
};
