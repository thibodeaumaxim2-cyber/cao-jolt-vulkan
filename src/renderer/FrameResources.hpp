#pragma once

#include "SwapchainFrame.hpp"
#include <vector>

struct FrameResources {
  std::vector<SwapchainFrame> frames;
  uint32_t currentFrame = 0;

  bool valid() const { return !frames.empty(); }

  SwapchainFrame* current() {
    return valid() ? &frames[currentFrame % frames.size()] : nullptr;
  }
};
