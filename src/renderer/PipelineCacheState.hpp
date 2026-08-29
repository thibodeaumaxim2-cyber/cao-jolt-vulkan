#pragma once

#include <vulkan/vulkan.h>

struct PipelineCacheState {
  VkPipelineCache cache = VK_NULL_HANDLE;

  bool valid() const { return cache != VK_NULL_HANDLE; }
};
