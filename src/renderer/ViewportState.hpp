#pragma once

#include "DynamicState.hpp"

inline VkPipelineViewportStateCreateInfo cadViewportState() {
  VkPipelineViewportStateCreateInfo info{
      VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
  info.viewportCount = 1;
  info.scissorCount = 1;
  return info;
}
