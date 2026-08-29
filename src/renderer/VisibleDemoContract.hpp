#pragma once

#include <cstdint>

struct VisibleDemoContract {
  static constexpr uint32_t vertexCount = 3;
  static constexpr const char* vertexShader = "cad.vert.spv";
  static constexpr const char* fragmentShader = "cad.frag.spv";
};
