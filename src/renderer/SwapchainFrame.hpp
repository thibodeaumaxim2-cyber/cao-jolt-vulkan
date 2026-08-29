#pragma once

#include <vulkan/vulkan.h>

struct SwapchainFrame {
  VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
  VkSemaphore imageAvailable = VK_NULL_HANDLE;
  VkSemaphore renderFinished = VK_NULL_HANDLE;
  VkFence inFlight = VK_NULL_HANDLE;
  uint32_t imageIndex = 0;
  bool acquired = false;
};
