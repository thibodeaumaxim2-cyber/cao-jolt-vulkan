#pragma once

#include <cstdint>
#include <optional>
#include <vulkan/vulkan.h>

struct QueueFamilySelection {
  std::optional<uint32_t> graphics;
  std::optional<uint32_t> present;

  bool complete() const {
    return graphics.has_value() && present.has_value();
  }
};

inline QueueFamilySelection findQueueFamilies(VkPhysicalDevice device,
                                              VkSurfaceKHR surface) {
  QueueFamilySelection result;
  uint32_t count = 0;
  vkGetPhysicalDeviceQueueFamilyProperties(device, &count, nullptr);
  if (count == 0)
    return result;

  std::vector<VkQueueFamilyProperties> families(count);
  vkGetPhysicalDeviceQueueFamilyProperties(device, &count, families.data());
  for (uint32_t i = 0; i < count; ++i) {
    if (families[i].queueCount > 0 &&
        (families[i].queueFlags & VK_QUEUE_GRAPHICS_BIT))
      result.graphics = i;

    VkBool32 present = VK_FALSE;
    vkGetPhysicalDeviceSurfaceSupportKHR(device, i, surface, &present);
    if (families[i].queueCount > 0 && present)
      result.present = i;

    if (result.complete())
      break;
  }
  return result;
}
