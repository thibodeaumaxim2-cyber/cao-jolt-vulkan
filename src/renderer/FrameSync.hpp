#pragma once

#include <cstdint>
#include <vulkan/vulkan.h>

struct FrameSyncPolicy {
  uint32_t framesInFlight = 2;
  VkPipelineStageFlags waitStage =
      VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
};

inline VkSubmitInfo makeGraphicsSubmitInfo(
    VkCommandBuffer commandBuffer,
    VkSemaphore imageAvailable,
    VkSemaphore renderFinished,
    const FrameSyncPolicy& policy = {}) {
  VkSubmitInfo info{VK_STRUCTURE_TYPE_SUBMIT_INFO};
  info.waitSemaphoreCount = 1;
  info.pWaitSemaphores = &imageAvailable;
  info.pWaitDstStageMask = &policy.waitStage;
  info.commandBufferCount = 1;
  info.pCommandBuffers = &commandBuffer;
  info.signalSemaphoreCount = 1;
  info.pSignalSemaphores = &renderFinished;
  return info;
}
