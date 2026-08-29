#pragma once

#include <vulkan/vulkan.h>

inline VkDebugUtilsMessengerCreateInfoEXT makeValidationMessengerInfo(
    PFN_vkDebugUtilsMessengerCallbackEXT callback,
    VkDebugUtilsMessageSeverityFlagsEXT severity =
        VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
        VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT,
    VkDebugUtilsMessageTypeFlagsEXT types =
        VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
        VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
        VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT) {
  VkDebugUtilsMessengerCreateInfoEXT info{
      VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT};
  info.messageSeverity = severity;
  info.messageType = types;
  info.pfnUserCallback = callback;
  return info;
}
