#pragma once

#include <cstdint>

struct JoltSyncState {
  uint32_t objectId = 0;
  uint32_t bodyId = 0;
  bool dynamic = false;
  bool active = true;
};
