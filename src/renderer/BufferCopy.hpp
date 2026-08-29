#pragma once

#include "TransferCommands.hpp"

inline void copyBufferRegion(VkCommandBuffer commandBuffer,
                             VkBuffer source,
                             VkBuffer destination,
                             VkDeviceSize sourceOffset,
                             VkDeviceSize destinationOffset,
                             VkDeviceSize size) {
  VkBufferCopy region{};
  region.srcOffset = sourceOffset;
  region.dstOffset = destinationOffset;
  region.size = size;
  vkCmdCopyBuffer(commandBuffer, source, destination, 1, &region);
}
