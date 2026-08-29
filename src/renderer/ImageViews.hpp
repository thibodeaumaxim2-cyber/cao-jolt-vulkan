#pragma once

#include <stdexcept>
#include <vulkan/vulkan.h>

inline VkImageView createColorImageView(VkDevice device,
                                        VkImage image,
                                        VkFormat format) {
  VkImageViewCreateInfo info{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
  info.image = image;
  info.viewType = VK_IMAGE_VIEW_TYPE_2D;
  info.format = format;
  info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  info.subresourceRange.baseMipLevel = 0;
  info.subresourceRange.levelCount = 1;
  info.subresourceRange.baseArrayLayer = 0;
  info.subresourceRange.layerCount = 1;

  VkImageView view = VK_NULL_HANDLE;
  if (vkCreateImageView(device, &info, nullptr, &view) != VK_SUCCESS)
    throw std::runtime_error("Cannot create Vulkan color image view");
  return view;
}

inline VkImageView createDepthImageView(VkDevice device,
                                        VkImage image,
                                        VkFormat format,
                                        VkImageAspectFlags aspectMask) {
  VkImageViewCreateInfo info{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
  info.image = image;
  info.viewType = VK_IMAGE_VIEW_TYPE_2D;
  info.format = format;
  info.subresourceRange.aspectMask = aspectMask;
  info.subresourceRange.baseMipLevel = 0;
  info.subresourceRange.levelCount = 1;
  info.subresourceRange.baseArrayLayer = 0;
  info.subresourceRange.layerCount = 1;

  VkImageView view = VK_NULL_HANDLE;
  if (vkCreateImageView(device, &info, nullptr, &view) != VK_SUCCESS)
    throw std::runtime_error("Cannot create Vulkan depth image view");
  return view;
}
