#pragma once

#include <stdexcept>
#include <vulkan/vulkan.h>

inline VkCommandPool createGraphicsCommandPool(VkDevice device,
                                               uint32_t queueFamily) {
  VkCommandPoolCreateInfo info{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
  info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
  info.queueFamilyIndex = queueFamily;

  VkCommandPool pool = VK_NULL_HANDLE;
  if (vkCreateCommandPool(device, &info, nullptr, &pool) != VK_SUCCESS)
    throw std::runtime_error("Cannot create Vulkan command pool");
  return pool;
}
