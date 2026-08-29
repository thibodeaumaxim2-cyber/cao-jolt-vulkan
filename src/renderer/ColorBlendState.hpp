#pragma once

#include <vulkan/vulkan.h>

inline VkPipelineColorBlendAttachmentState cadColorBlendAttachment() {
  VkPipelineColorBlendAttachmentState state{};
  state.colorWriteMask = VK_COLOR_COMPONENT_R_BIT |
                         VK_COLOR_COMPONENT_G_BIT |
                         VK_COLOR_COMPONENT_B_BIT |
                         VK_COLOR_COMPONENT_A_BIT;
  state.blendEnable = VK_FALSE;
  return state;
}

inline VkPipelineColorBlendStateCreateInfo cadColorBlendState(
    VkPipelineColorBlendAttachmentState* attachment) {
  VkPipelineColorBlendStateCreateInfo info{
      VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
  info.attachmentCount = 1;
  info.pAttachments = attachment;
  return info;
}
