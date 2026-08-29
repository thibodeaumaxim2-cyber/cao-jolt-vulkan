#pragma once

#include "Vertex.hpp"
#include <array>
#include <vulkan/vulkan.h>

inline std::array<VkVertexInputAttributeDescription, 2>
cadVertexAttributes() {
  return CadVertex::attributes();
}

inline VkVertexInputBindingDescription cadVertexBinding() {
  return CadVertex::binding();
}

inline VkPipelineVertexInputStateCreateInfo cadVertexInputState(
    VkVertexInputBindingDescription* binding,
    VkVertexInputAttributeDescription* attributes) {
  VkPipelineVertexInputStateCreateInfo info{
      VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
  info.vertexBindingDescriptionCount = 1;
  info.pVertexBindingDescriptions = binding;
  info.vertexAttributeDescriptionCount = 2;
  info.pVertexAttributeDescriptions = attributes;
  return info;
}
