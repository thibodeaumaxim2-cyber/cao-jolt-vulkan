#pragma once

#include <filesystem>
#include <vulkan/vulkan.h>

struct PipelineBuildInfo {
  VkRenderPass renderPass = VK_NULL_HANDLE;
  VkFormat colorFormat = VK_FORMAT_UNDEFINED;
  VkExtent2D extent{};
  std::filesystem::path vertexShader;
  std::filesystem::path fragmentShader;

  bool valid() const {
    return renderPass != VK_NULL_HANDLE &&
           colorFormat != VK_FORMAT_UNDEFINED &&
           extent.width > 0 && extent.height > 0 &&
           !vertexShader.empty() && !fragmentShader.empty();
  }
};
