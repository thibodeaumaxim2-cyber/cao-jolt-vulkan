#pragma once

#include <array>
#include <vulkan/vulkan.h>

inline std::array<VkDynamicState, 2> cadDynamicStates() {
  return {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
}

inline VkPipelineDynamicStateCreateInfo cadDynamicStateInfo(
    const VkDynamicState* states, uint32_t count = 2) {
  VkPipelineDynamicStateCreateInfo info{
      VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
  info.dynamicStateCount = count;
  info.pDynamicStates = states;
  return info;
}

inline VkViewport makeCadViewport(VkExtent2D extent) {
  return VkViewport{0.0f, 0.0f, static_cast<float>(extent.width),
                    static_cast<float>(extent.height), 0.0f, 1.0f};
}

inline VkRect2D makeCadScissor(VkExtent2D extent) {
  return VkRect2D{{0, 0}, extent};
}
