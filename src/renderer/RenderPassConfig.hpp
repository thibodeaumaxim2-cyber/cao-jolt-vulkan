#pragma once
#include <vulkan/vulkan.h>
struct RenderPassConfig {
 VkFormat colorFormat=VK_FORMAT_B8G8R8A8_SRGB;
 VkFormat depthFormat=VK_FORMAT_D32_SFLOAT;
 VkSampleCountFlagBits samples=VK_SAMPLE_COUNT_1_BIT;
 VkAttachmentLoadOp colorLoad=VK_ATTACHMENT_LOAD_OP_CLEAR;
 VkAttachmentStoreOp colorStore=VK_ATTACHMENT_STORE_OP_STORE;
 VkAttachmentLoadOp depthLoad=VK_ATTACHMENT_LOAD_OP_CLEAR;
 VkAttachmentStoreOp depthStore=VK_ATTACHMENT_STORE_OP_DONT_CARE;
};
inline bool isDepthFormatSupported(VkPhysicalDevice gpu,VkFormat f){
 VkFormatProperties p{}; vkGetPhysicalDeviceFormatProperties(gpu,f,&p);
 return (p.optimalTilingFeatures&VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT)!=0;
}
inline VkFormat chooseDepthFormat(VkPhysicalDevice gpu){
 for(VkFormat f:{VK_FORMAT_D32_SFLOAT,VK_FORMAT_D32_SFLOAT_S8_UINT,VK_FORMAT_D24_UNORM_S8_UINT})
  if(isDepthFormatSupported(gpu,f)) return f;
 return VK_FORMAT_D16_UNORM;
}