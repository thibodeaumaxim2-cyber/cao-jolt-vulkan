#pragma once

#include <vulkan/vulkan.h>

struct DeviceQueues {
  VkQueue graphics = VK_NULL_HANDLE;
  VkQueue present = VK_NULL_HANDLE;

  bool valid() const {
    return graphics != VK_NULL_HANDLE && present != VK_NULL_HANDLE;
  }
};
