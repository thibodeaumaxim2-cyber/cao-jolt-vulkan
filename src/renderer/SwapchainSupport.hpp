#pragma once

#include <vector>
#include <vulkan/vulkan.h>

struct SwapchainSupportDetails {
  VkSurfaceCapabilitiesKHR capabilities{};
  std::vector<VkSurfaceFormatKHR> formats;
  std::vector<VkPresentModeKHR> presentModes;
};

inline SwapchainSupportDetails querySwapchainSupport(VkPhysicalDevice device,
                                                      VkSurfaceKHR surface) {
  SwapchainSupportDetails details;
  vkGetPhysicalDeviceSurfaceCapabilitiesKHR(device, surface,
                                            &details.capabilities);

  uint32_t formatCount = 0;
  vkGetPhysicalDeviceSurfaceFormatsKHR(device, surface, &formatCount, nullptr);
  details.formats.resize(formatCount);
  if (formatCount)
    vkGetPhysicalDeviceSurfaceFormatsKHR(device, surface, &formatCount,
                                         details.formats.data());

  uint32_t modeCount = 0;
  vkGetPhysicalDeviceSurfacePresentModesKHR(device, surface, &modeCount,
                                            nullptr);
  details.presentModes.resize(modeCount);
  if (modeCount)
    vkGetPhysicalDeviceSurfacePresentModesKHR(device, surface, &modeCount,
                                              details.presentModes.data());
  return details;
}
