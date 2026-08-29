#pragma once

#include "MemoryTypes.hpp"
#include <stdexcept>
#include <vulkan/vulkan.h>

struct DepthImage {
  VkImage image = VK_NULL_HANDLE;
  VkDeviceMemory memory = VK_NULL_HANDLE;
  VkImageView view = VK_NULL_HANDLE;
  VkFormat format = VK_FORMAT_UNDEFINED;
};

inline DepthImage createCadDepthImage(VkPhysicalDevice physicalDevice,
                                      VkDevice device,
                                      VkExtent2D extent,
                                      VkFormat format) {
  DepthImage result;
  result.format = format;

  VkImageCreateInfo image{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
  image.imageType = VK_IMAGE_TYPE_2D;
  image.format = format;
  image.extent = {extent.width, extent.height, 1};
  image.mipLevels = 1;
  image.arrayLayers = 1;
  image.samples = VK_SAMPLE_COUNT_1_BIT;
  image.tiling = VK_IMAGE_TILING_OPTIMAL;
  image.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
  image.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

  if (vkCreateImage(device, &image, nullptr, &result.image) != VK_SUCCESS)
    throw std::runtime_error("Cannot create CAO Vulkan depth image");

  VkMemoryRequirements requirements{};
  vkGetImageMemoryRequirements(device, result.image, &requirements);
  VkMemoryAllocateInfo allocation{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
  allocation.allocationSize = requirements.size;
  allocation.memoryTypeIndex =
      findMemoryType(physicalDevice, requirements.memoryTypeBits,
                     VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

  if (vkAllocateMemory(device, &allocation, nullptr, &result.memory) !=
          VK_SUCCESS ||
      vkBindImageMemory(device, result.image, result.memory, 0) != VK_SUCCESS)
    throw std::runtime_error("Cannot allocate CAO Vulkan depth image");

  VkImageViewCreateInfo view{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
  view.image = result.image;
  view.viewType = VK_IMAGE_VIEW_TYPE_2D;
  view.format = format;
  view.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
  view.subresourceRange.levelCount = 1;
  view.subresourceRange.layerCount = 1;

  if (vkCreateImageView(device, &view, nullptr, &result.view) != VK_SUCCESS)
    throw std::runtime_error("Cannot create CAO Vulkan depth view");
  return result;
}
