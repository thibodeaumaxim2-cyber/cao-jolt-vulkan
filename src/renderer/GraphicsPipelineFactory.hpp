#pragma once

#include "PipelineConfig.hpp"
#include "ShaderLoader.hpp"
#include <array>
#include <stdexcept>
#include <vulkan/vulkan.h>

inline VkPipeline createCadGraphicsPipeline(
    VkDevice device,
    VkRenderPass renderPass,
    VkExtent2D extent,
    VkShaderModule vertexShader,
    VkShaderModule fragmentShader,
    VkPipelineLayout layout) {
  auto stages = std::array<VkPipelineShaderStageCreateInfo, 2>{
      VkPipelineShaderStageCreateInfo{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO},
      VkPipelineShaderStageCreateInfo{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO}};
  stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
  stages[0].module = vertexShader;
  stages[0].pName = "main";
  stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
  stages[1].module = fragmentShader;
  stages[1].pName = "main";

  auto binding = cadVertexBinding();
  auto attributes = cadVertexAttributes();
  auto input = cadVertexInputState(&binding, attributes.data());
  auto assembly = cadInputAssembly();
  auto viewport = makeCadViewport(extent);
  auto scissor = makeCadScissor(extent);
  VkPipelineViewportStateCreateInfo viewportState{
      VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
  viewportState.viewportCount = 1;
  viewportState.pViewports = &viewport;
  viewportState.scissorCount = 1;
  viewportState.pScissors = &scissor;
  auto rasterizer = cadRasterizer();
  auto multisampling = cadMultisampling();
  auto depth = cadDepthState();
  auto blendAttachment = cadColorBlendAttachment();
  auto blend = cadColorBlendState(&blendAttachment);

  VkGraphicsPipelineCreateInfo info{
      VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
  info.stageCount = static_cast<uint32_t>(stages.size());
  info.pStages = stages.data();
  info.pVertexInputState = &input;
  info.pInputAssemblyState = &assembly;
  info.pViewportState = &viewportState;
  info.pRasterizationState = &rasterizer;
  info.pMultisampleState = &multisampling;
  info.pDepthStencilState = &depth;
  info.pColorBlendState = &blend;
  info.layout = layout;
  info.renderPass = renderPass;
  info.subpass = 0;

  VkPipeline pipeline = VK_NULL_HANDLE;
  if (vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &info, nullptr,
                                &pipeline) != VK_SUCCESS)
    throw std::runtime_error("Cannot create CAO Vulkan graphics pipeline");
  return pipeline;
}
