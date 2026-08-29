#pragma once

#include <stdexcept>
#include <vulkan/vulkan.h>

inline void copyBuffer(VkCommandBuffer commandBuffer,
                       VkBuffer source,
                       VkBuffer destination,
                       VkDeviceSize size) {
  VkBufferCopy region{};
  region.size = size;
  vkCmdCopyBuffer(commandBuffer, source, destination, 1, &region);
}
