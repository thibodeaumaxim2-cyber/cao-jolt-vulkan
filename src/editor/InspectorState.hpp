#pragma once

#include <cstdint>

struct InspectorState {
  uint32_t inspectedObjectId = 0;
  bool transformExpanded = true;
  bool physicsExpanded = true;
  bool materialExpanded = false;
  bool structuralExpanded = true;
};
