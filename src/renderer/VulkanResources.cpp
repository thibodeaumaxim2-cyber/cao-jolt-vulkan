#include "VulkanResources.hpp"
#include <stdexcept>
#include "RenderPassConfig.hpp"
uint32_t findMemoryType(VkPhysicalDevice gpu,VkMemoryRequirements req,VkMemoryPropertyFlags flags){VkPhysicalDeviceMemoryProperties p{};vkGetPhysicalDeviceMemoryProperties(gpu,&p);for(uint32_t i=0;i<p.memoryTypeCount;i++)if((req.memoryTypeBits&(1u<<i))&&(p.memoryTypes[i].propertyFlags&flags)==flags)return i;throw std::runtime_error("No compatible Vulkan memory type");}
VulkanBuffer createBuffer(VkPhysicalDevice gpu,VkDevice d,VkDeviceSize size,VkBufferUsageFlags usage,VkMemoryPropertyFlags flags){VulkanBuffer out;VkBufferCreateInfo b{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO,nullptr,0,size,usage,VK_SHARING_MODE_EXCLUSIVE,0,nullptr};if(vkCreateBuffer(d,&b,nullptr,&out.buffer)!=VK_SUCCESS)throw std::runtime_error("Cannot create Vulkan buffer");VkMemoryRequirements r{};vkGetBufferMemoryRequirements(d,out.buffer,&r);VkMemoryAllocateInfo a{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,nullptr,r.size,findMemoryType(gpu,r,flags)};if(vkAllocateMemory(d,&a,nullptr,&out.memory)!=VK_SUCCESS)throw std::runtime_error("Cannot allocate Vulkan buffer memory");vkBindBufferMemory(d,out.buffer,out.memory,0);return out;}
void destroyBuffer(VkDevice d,VulkanBuffer& b){if(b.buffer)vkDestroyBuffer(d,b.buffer,nullptr);if(b.memory)vkFreeMemory(d,b.memory,nullptr);b={};}
VulkanImage createDepthImage(VkPhysicalDevice gpu,VkDevice d,VkExtent2D extent,VkFormat format){
 VulkanImage out;
 VkImageCreateInfo ci{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,nullptr,0,VK_IMAGE_TYPE_2D,format,{extent.width,extent.height,1},1,1,VK_SAMPLE_COUNT_1_BIT,VK_IMAGE_TILING_OPTIMAL,VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,VK_SHARING_MODE_EXCLUSIVE,0,nullptr,VK_IMAGE_LAYOUT_UNDEFINED};
 if(vkCreateImage(d,&ci,nullptr,&out.image)!=VK_SUCCESS) throw std::runtime_error("Cannot create depth image");
 VkMemoryRequirements req{}; vkGetImageMemoryRequirements(d,out.image,&req);
 VkMemoryAllocateInfo ai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,nullptr,req.size,findMemoryType(gpu,req,VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT)};
 if(vkAllocateMemory(d,&ai,nullptr,&out.memory)!=VK_SUCCESS) throw std::runtime_error("Cannot allocate depth image memory");
 vkBindImageMemory(d,out.image,out.memory,0);
 VkImageViewCreateInfo vi{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO,nullptr,0,out.image,VK_IMAGE_VIEW_TYPE_2D,format,{VK_COMPONENT_SWIZZLE_IDENTITY,VK_COMPONENT_SWIZZLE_IDENTITY,VK_COMPONENT_SWIZZLE_IDENTITY,VK_COMPONENT_SWIZZLE_IDENTITY},{VK_IMAGE_ASPECT_DEPTH_BIT,0,1,0,1}};
 if(vkCreateImageView(d,&vi,nullptr,&out.view)!=VK_SUCCESS) throw std::runtime_error("Cannot create depth image view");
 return out;
}
void destroyImage(VkDevice d,VulkanImage&i){if(i.view)vkDestroyImageView(d,i.view,nullptr);if(i.image)vkDestroyImage(d,i.image,nullptr);if(i.memory)vkFreeMemory(d,i.memory,nullptr);i={};}