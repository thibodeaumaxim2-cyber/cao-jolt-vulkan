#pragma once

#include <vulkan/vulkan.h>

inline VkPipelineDepthStencilStateCreateInfo cadDepthState(
    bool enable = true) {
  VkPipelineDepthStencilStateCreateInfo info{
      VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
  info.depthTestEnable = enable ? VK_TRUE : VK_FALSE;
  info.depthWriteEnable = enable ? VK_TRUE : VK_FALSE;
  info.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
  info.depthBoundsTestEnable = VK_FALSE;
  info.stencilTestEnable = VK_FALSE;
  return info;
}
