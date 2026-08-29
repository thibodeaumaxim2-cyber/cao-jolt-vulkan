#pragma once

#include <array>
#include <vulkan/vulkan.h>

inline std::array<VkPipelineShaderStageCreateInfo, 2> cadShaderStages(
    VkShaderModule vertexModule, VkShaderModule fragmentModule) {
  VkPipelineShaderStageCreateInfo vertex{
      VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
  vertex.stage = VK_SHADER_STAGE_VERTEX_BIT;
  vertex.module = vertexModule;
  vertex.pName = "main";

  VkPipelineShaderStageCreateInfo fragment{
      VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
  fragment.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
  fragment.module = fragmentModule;
  fragment.pName = "main";

  return {vertex, fragment};
}
