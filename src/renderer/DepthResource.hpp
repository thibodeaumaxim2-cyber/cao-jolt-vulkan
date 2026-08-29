#pragma once

#include <vulkan/vulkan.h>

struct DepthResource {
  VkImage image = VK_NULL_HANDLE;
  VkDeviceMemory memory = VK_NULL_HANDLE;
  VkImageView view = VK_NULL_HANDLE;
  VkFormat format = VK_FORMAT_UNDEFINED;

  bool valid() const {
    return image != VK_NULL_HANDLE && view != VK_NULL_HANDLE &&
           format != VK_FORMAT_UNDEFINED;
  }
};
