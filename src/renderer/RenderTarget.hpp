#pragma once

#include <vulkan/vulkan.h>

struct RenderTarget {
  VkImage colorImage = VK_NULL_HANDLE;
  VkImageView colorView = VK_NULL_HANDLE;
  VkImageView depthView = VK_NULL_HANDLE;
  VkFramebuffer framebuffer = VK_NULL_HANDLE;
  VkExtent2D extent{};

  bool valid() const {
    return colorView != VK_NULL_HANDLE &&
           framebuffer != VK_NULL_HANDLE &&
           extent.width > 0 && extent.height > 0;
  }
};
