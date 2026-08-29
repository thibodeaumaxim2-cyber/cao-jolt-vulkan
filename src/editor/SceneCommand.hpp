#pragma once

#include <cstdint>

enum class SceneCommandKind {
  None,
  AddCube,
  AddBeam,
  AddCable,
  DeleteSelected,
  BuildPyramid,
  ResetScene,
  SaveScene,
  LoadScene
};

struct SceneCommand {
  SceneCommandKind kind = SceneCommandKind::None;
  uint32_t target = 0;
};
