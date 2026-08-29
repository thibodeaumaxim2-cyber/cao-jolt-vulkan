#pragma once

#include <vulkan/vulkan.h>

inline void destroyVulkanPipeline(VkDevice device,
                                  VkPipeline& pipeline,
                                  VkPipelineLayout& layout) {
  if (pipeline != VK_NULL_HANDLE) {
    vkDestroyPipeline(device, pipeline, nullptr);
    pipeline = VK_NULL_HANDLE;
  }
  if (layout != VK_NULL_HANDLE) {
    vkDestroyPipelineLayout(device, layout, nullptr);
    layout = VK_NULL_HANDLE;
  }
}

inline void destroyVulkanRenderPass(VkDevice device,
                                    VkRenderPass& renderPass) {
  if (renderPass != VK_NULL_HANDLE) {
    vkDestroyRenderPass(device, renderPass, nullptr);
    renderPass = VK_NULL_HANDLE;
  }
}
