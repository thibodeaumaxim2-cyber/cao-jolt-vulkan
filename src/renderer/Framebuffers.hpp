#pragma once

#include <stdexcept>
#include <vulkan/vulkan.h>

inline VkFramebuffer createCadFramebuffer(VkDevice device,
                                          VkRenderPass renderPass,
                                          VkImageView colorView,
                                          VkImageView depthView,
                                          VkExtent2D extent) {
  VkImageView attachments[] = {colorView, depthView};
  VkFramebufferCreateInfo info{VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
  info.renderPass = renderPass;
  info.attachmentCount = depthView != VK_NULL_HANDLE ? 2u : 1u;
  info.pAttachments = attachments;
  info.width = extent.width;
  info.height = extent.height;
  info.layers = 1;

  VkFramebuffer framebuffer = VK_NULL_HANDLE;
  if (vkCreateFramebuffer(device, &info, nullptr, &framebuffer) != VK_SUCCESS)
    throw std::runtime_error("Cannot create Vulkan framebuffer");
  return framebuffer;
}
