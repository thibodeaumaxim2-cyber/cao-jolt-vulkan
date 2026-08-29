#pragma once
#include <vulkan/vulkan.h>
#include "VulkanRenderer.hpp"
inline VkVertexInputBindingDescription cadVertexBinding(){
 return {0,sizeof(CadVertex),VK_VERTEX_INPUT_RATE_VERTEX};
}
inline std::array<VkVertexInputAttributeDescription,2> cadVertexAttributes(){
 return {{{0,0,VK_FORMAT_R32G32B32_SFLOAT,0},{1,0,VK_FORMAT_R32G32B32_SFLOAT,12}}};
}