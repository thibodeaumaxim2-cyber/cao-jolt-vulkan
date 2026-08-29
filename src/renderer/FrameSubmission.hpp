#pragma once

#include <vulkan/vulkan.h>

struct FrameSubmission {
  VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
  VkSemaphore waitSemaphore = VK_NULL_HANDLE;
  VkSemaphore signalSemaphore = VK_NULL_HANDLE;
  VkFence fence = VK_NULL_HANDLE;
  VkPipelineStageFlags waitStage =
      VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;

  VkSubmitInfo submitInfo() const {
    VkSubmitInfo info{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    info.waitSemaphoreCount = waitSemaphore ? 1u : 0u;
    info.pWaitSemaphores = waitSemaphore ? &waitSemaphore : nullptr;
    info.pWaitDstStageMask = waitSemaphore ? &waitStage : nullptr;
    info.commandBufferCount = commandBuffer ? 1u : 0u;
    info.pCommandBuffers = commandBuffer ? &commandBuffer : nullptr;
    info.signalSemaphoreCount = signalSemaphore ? 1u : 0u;
    info.pSignalSemaphores = signalSemaphore ? &signalSemaphore : nullptr;
    return info;
  }
};
