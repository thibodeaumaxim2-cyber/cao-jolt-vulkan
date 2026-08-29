#pragma once

#include <array>
#include <vulkan/vulkan.h>

struct RenderPassConfig {
  VkFormat colorFormat = VK_FORMAT_B8G8R8A8_SRGB;
  VkFormat depthFormat = VK_FORMAT_D32_SFLOAT;
  VkSampleCountFlagBits samples = VK_SAMPLE_COUNT_1_BIT;
  VkAttachmentLoadOp colorLoad = VK_ATTACHMENT_LOAD_OP_CLEAR;
  VkAttachmentStoreOp colorStore = VK_ATTACHMENT_STORE_OP_STORE;
  VkAttachmentLoadOp depthLoad = VK_ATTACHMENT_LOAD_OP_CLEAR;
  VkAttachmentStoreOp depthStore = VK_ATTACHMENT_STORE_OP_DONT_CARE;
};

inline bool isDepthFormatSupported(VkPhysicalDevice gpu, VkFormat format) {
  VkFormatProperties properties{};
  vkGetPhysicalDeviceFormatProperties(gpu, format, &properties);
  return (properties.optimalTilingFeatures &
          VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT) != 0;
}

inline VkFormat chooseDepthFormat(VkPhysicalDevice gpu) {
  constexpr std::array<VkFormat, 4> candidates{
      VK_FORMAT_D32_SFLOAT,
      VK_FORMAT_D32_SFLOAT_S8_UINT,
      VK_FORMAT_D24_UNORM_S8_UINT,
      VK_FORMAT_D16_UNORM};
  for (VkFormat format : candidates)
    if (isDepthFormatSupported(gpu, format))
      return format;
  return VK_FORMAT_UNDEFINED;
}

inline bool hasStencilComponent(VkFormat format) {
  return format == VK_FORMAT_D32_SFLOAT_S8_UINT ||
         format == VK_FORMAT_D24_UNORM_S8_UINT;
}

inline VkImageAspectFlags depthAspectMask(VkFormat format) {
  return VK_IMAGE_ASPECT_DEPTH_BIT |
         (hasStencilComponent(format) ? VK_IMAGE_ASPECT_STENCIL_BIT : 0);
}

inline VkSurfaceFormatKHR chooseSurfaceFormat(
    const VkSurfaceFormatKHR* formats, uint32_t count) {
  for (uint32_t i = 0; i < count; ++i) {
    if (formats[i].format == VK_FORMAT_B8G8R8A8_SRGB &&
        formats[i].colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR)
      return formats[i];
  }
  return formats[0];
}

inline VkPresentModeKHR choosePresentMode(
    const VkPresentModeKHR* modes, uint32_t count) {
  for (uint32_t i = 0; i < count; ++i)
    if (modes[i] == VK_PRESENT_MODE_MAILBOX_KHR)
      return modes[i];
  return VK_PRESENT_MODE_FIFO_KHR;
}
