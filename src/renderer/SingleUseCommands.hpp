#pragma once

#include <stdexcept>
#include <vulkan/vulkan.h>

inline VkCommandBuffer beginSingleUseCommands(VkDevice device,
                                               VkCommandPool pool) {
  VkCommandBufferAllocateInfo allocation{
      VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
  allocation.commandPool = pool;
  allocation.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
  allocation.commandBufferCount = 1;

  VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
  if (vkAllocateCommandBuffers(device, &allocation, &commandBuffer) != VK_SUCCESS)
    throw std::runtime_error("Cannot allocate Vulkan upload command buffer");

  VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
  begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
  if (vkBeginCommandBuffer(commandBuffer, &begin) != VK_SUCCESS)
    throw std::runtime_error("Cannot begin Vulkan upload command buffer");
  return commandBuffer;
}

inline void endSingleUseCommands(VkDevice device,
                                 VkCommandPool pool,
                                 VkQueue queue,
                                 VkCommandBuffer commandBuffer) {
  if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS)
    throw std::runtime_error("Cannot end Vulkan upload command buffer");

  VkSubmitInfo submit{VK_STRUCTURE_TYPE_SUBMIT_INFO};
  submit.commandBufferCount = 1;
  submit.pCommandBuffers = &commandBuffer;
  if (vkQueueSubmit(queue, 1, &submit, VK_NULL_HANDLE) != VK_SUCCESS ||
      vkQueueWaitIdle(queue) != VK_SUCCESS)
    throw std::runtime_error("Cannot submit Vulkan upload command buffer");

  vkFreeCommandBuffers(device, pool, 1, &commandBuffer);
}
