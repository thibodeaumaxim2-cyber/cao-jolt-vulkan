#pragma once

#include <algorithm>
#include <cstdint>
#include <vulkan/vulkan.h>

inline VkExtent2D chooseSwapchainExtent(const VkSurfaceCapabilitiesKHR& caps,
                                        uint32_t windowWidth,
                                        uint32_t windowHeight) {
  if (caps.currentExtent.width != UINT32_MAX)
    return caps.currentExtent;

  VkExtent2D extent{windowWidth, windowHeight};
  extent.width = std::clamp(extent.width, caps.minImageExtent.width,
                            caps.maxImageExtent.width);
  extent.height = std::clamp(extent.height, caps.minImageExtent.height,
                             caps.maxImageExtent.height);
  return extent;
}
