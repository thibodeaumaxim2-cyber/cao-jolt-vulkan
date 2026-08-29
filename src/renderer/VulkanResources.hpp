#pragma once
#include <vulkan/vulkan.h>
#include <cstddef>
struct VulkanBuffer { VkBuffer buffer=VK_NULL_HANDLE; VkDeviceMemory memory=VK_NULL_HANDLE; };
struct VulkanImage { VkImage image=VK_NULL_HANDLE; VkImageView view=VK_NULL_HANDLE; VkDeviceMemory memory=VK_NULL_HANDLE; };
uint32_t findMemoryType(VkPhysicalDevice,VkMemoryRequirements,VkMemoryPropertyFlags);
VulkanBuffer createBuffer(VkPhysicalDevice,VkDevice,VkDeviceSize,VkBufferUsageFlags,VkMemoryPropertyFlags);
VulkanImage createDepthImage(VkPhysicalDevice,VkDevice,VkExtent2D,VkFormat);
void destroyBuffer(VkDevice,VulkanBuffer&);
void destroyImage(VkDevice,VulkanImage&);