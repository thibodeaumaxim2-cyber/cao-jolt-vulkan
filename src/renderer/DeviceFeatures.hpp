#pragma once

#include <vulkan/vulkan.h>

inline bool supportsRequiredDeviceFeatures(VkPhysicalDevice device) {
  VkPhysicalDeviceFeatures features{};
  vkGetPhysicalDeviceFeatures(device, &features);
  return features.samplerAnisotropy == VK_TRUE;
}
