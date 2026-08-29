#pragma once

#include <cstdint>

enum class EditorActionType {
  None,
  AddCube,
  AddPyramid,
  DeleteSelected,
  ResetScene,
  ToggleSimulation,
  SelectObject
};

struct EditorAction {
  EditorActionType type = EditorActionType::None;
  uint32_t objectId = 0;
};
