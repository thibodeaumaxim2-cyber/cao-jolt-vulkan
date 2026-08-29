#pragma once

#include <stdexcept>
#include <vulkan/vulkan.h>

inline VkPipelineLayout createCadPipelineLayout(
    VkDevice device,
    const VkPushConstantRange* pushConstants,
    uint32_t pushConstantCount = 1) {
  VkPipelineLayoutCreateInfo info{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
  info.pushConstantRangeCount = pushConstantCount;
  info.pPushConstantRanges = pushConstants;
  VkPipelineLayout layout = VK_NULL_HANDLE;
  if (vkCreatePipelineLayout(device, &info, nullptr, &layout) != VK_SUCCESS)
    throw std::runtime_error("Cannot create Vulkan pipeline layout");
  return layout;
}
