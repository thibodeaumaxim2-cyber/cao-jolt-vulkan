#pragma once

#include <cstdint>

struct CableConnection {
  uint32_t cableId = 0;
  uint32_t startObjectId = 0;
  uint32_t endObjectId = 0;
  float restLength = 1.0f;
  float stiffness = 1000.0f;
  float breakingTension = 10000.0f;
};
