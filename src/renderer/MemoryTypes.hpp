#pragma once

#include <cstdint>
#include <stdexcept>
#include <vulkan/vulkan.h>

inline uint32_t findMemoryType(VkPhysicalDevice device,
                               uint32_t typeFilter,
                               VkMemoryPropertyFlags properties) {
  VkPhysicalDeviceMemoryProperties memory{};
  vkGetPhysicalDeviceMemoryProperties(device, &memory);
  for (uint32_t i = 0; i < memory.memoryTypeCount; ++i) {
    if ((typeFilter & (1u << i)) &&
        (memory.memoryTypes[i].propertyFlags & properties) == properties)
      return i;
  }
  throw std::runtime_error("No compatible Vulkan memory type found");
}
