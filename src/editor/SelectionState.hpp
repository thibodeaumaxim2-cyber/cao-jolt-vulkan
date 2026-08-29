#pragma once

#include <cstdint>

struct SelectionState {
  uint32_t selectedObjectId = 0;
  bool hasSelection = false;

  void clear() {
    selectedObjectId = 0;
    hasSelection = false;
  }

  void select(uint32_t objectId) {
    selectedObjectId = objectId;
    hasSelection = objectId != 0;
  }

  bool isSelected(uint32_t objectId) const {
    return hasSelection && selectedObjectId == objectId;
  }
};
