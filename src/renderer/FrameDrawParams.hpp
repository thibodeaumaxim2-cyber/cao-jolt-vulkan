#pragma once

#include "FrameCamera.hpp"
#include "DrawList.hpp"

struct FrameDrawParams {
  CameraFrame camera;
  const DrawList* drawList = nullptr;
  uint32_t selectedObjectId = 0;

  bool valid() const {
    return drawList != nullptr && !drawList->empty();
  }
};
