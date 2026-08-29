#pragma once
#include <array>
#include <vulkan/vulkan.h>
#include "BufferLayout.hpp"
#include "VulkanTypes.hpp"
inline VkPipelineVertexInputStateCreateInfo cadVertexInput(){
 static auto binding=cadVertexBinding();
 static auto attributes=cadVertexAttributes();
 static VkPipelineVertexInputStateCreateInfo state{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO,nullptr,0,1,&binding,static_cast<uint32_t>(attributes.size()),attributes.data()};
 return state;
}
inline VkPipelineInputAssemblyStateCreateInfo cadInputAssembly(){
 return {VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO,nullptr,0,VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST,VK_FALSE};
}
inline VkPipelineRasterizationStateCreateInfo cadRasterizer(){
 return {VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO,nullptr,0,VK_FALSE,VK_FALSE,VK_POLYGON_MODE_FILL,VK_CULL_MODE_BACK_BIT,VK_FRONT_FACE_COUNTER_CLOCKWISE,VK_FALSE,0,0,0,1.0f};
}
inline VkPipelineDepthStencilStateCreateInfo cadDepthStencil(){
 return {VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO,nullptr,0,VK_TRUE,VK_TRUE,VK_COMPARE_OP_LESS_OR_EQUAL,VK_FALSE,VK_FALSE,{},{},0,0};
}
inline VkPushConstantRange cadPushConstants(){
 return {VK_SHADER_STAGE_VERTEX_BIT,0,sizeof(ObjectPushConstant)};
}