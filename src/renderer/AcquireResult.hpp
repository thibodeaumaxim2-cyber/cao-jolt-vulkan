#pragma once

#include <vulkan/vulkan.h>

enum class AcquireStatus {
  Ready,
  RecreateSwapchain,
  Failed
};

struct AcquireResult {
  AcquireStatus status = AcquireStatus::Failed;
  uint32_t imageIndex = 0;

  bool ready() const { return status == AcquireStatus::Ready; }
  bool recreate() const { return status == AcquireStatus::RecreateSwapchain; }
};

inline AcquireResult classifyAcquireResult(VkResult result,
                                           uint32_t imageIndex) {
  if (result == VK_SUCCESS || result == VK_SUBOPTIMAL_KHR)
    return {result == VK_SUCCESS ? AcquireStatus::Ready
                                 : AcquireStatus::RecreateSwapchain,
            imageIndex};
  if (result == VK_ERROR_OUT_OF_DATE_KHR)
    return {AcquireStatus::RecreateSwapchain, 0};
  return {AcquireStatus::Failed, 0};
}
