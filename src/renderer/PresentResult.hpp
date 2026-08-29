#pragma once

#include <vulkan/vulkan.h>

enum class PresentStatus {
  Presented,
  RecreateSwapchain,
  Failed
};

inline PresentStatus classifyPresentResult(VkResult result) {
  if (result == VK_SUCCESS)
    return PresentStatus::Presented;
  if (result == VK_ERROR_OUT_OF_DATE_KHR ||
      result == VK_SUBOPTIMAL_KHR)
    return PresentStatus::RecreateSwapchain;
  return PresentStatus::Failed;
}
