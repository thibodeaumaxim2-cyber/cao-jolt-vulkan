#pragma once

#include <vulkan/vulkan.h>

inline VkPipelineMultisampleStateCreateInfo cadMultisampling(
    VkSampleCountFlagBits samples = VK_SAMPLE_COUNT_1_BIT) {
  VkPipelineMultisampleStateCreateInfo info{
      VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
  info.rasterizationSamples = samples;
  info.sampleShadingEnable = VK_FALSE;
  return info;
}
