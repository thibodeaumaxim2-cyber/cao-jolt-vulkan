#pragma once

#include <vulkan/vulkan.h>

struct RendererHandles {
  VkInstance instance = VK_NULL_HANDLE;
  VkSurfaceKHR surface = VK_NULL_HANDLE;
  VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
  VkDevice device = VK_NULL_HANDLE;
  VkQueue graphicsQueue = VK_NULL_HANDLE;
  VkQueue presentQueue = VK_NULL_HANDLE;
  VkCommandPool commandPool = VK_NULL_HANDLE;
  VkRenderPass renderPass = VK_NULL_HANDLE;
  VkPipelineLayout pipelineLayout = VK_NULL_HANDLE;
  VkPipeline pipeline = VK_NULL_HANDLE;

  bool deviceReady() const {
    return physicalDevice != VK_NULL_HANDLE &&
           device != VK_NULL_HANDLE;
  }

  bool pipelineReady() const {
    return renderPass != VK_NULL_HANDLE &&
           pipelineLayout != VK_NULL_HANDLE &&
           pipeline != VK_NULL_HANDLE;
  }
};
