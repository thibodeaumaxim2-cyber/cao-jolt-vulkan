#pragma once

#include <cstdint>

struct RendererConfig {
  uint32_t framesInFlight = 2;
  bool enableDepth = true;
  bool enableVSync = true;
  bool enableValidation = false;
  float clearColor[4] = {0.035f, 0.055f, 0.10f, 1.0f};
};
