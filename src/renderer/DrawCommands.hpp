#pragma once

#include "ObjectDraw.hpp"
#include "VulkanTypes.hpp"
#include <vulkan/vulkan.h>

inline void recordIndexedObject(VkCommandBuffer commandBuffer,
                                VkPipeline pipeline,
                                VkPipelineLayout pipelineLayout,
                                VkBuffer vertexBuffer,
                                VkBuffer indexBuffer,
                                const ObjectPushConstant& pushConstant,
                                const ObjectDrawData& draw) {
  VkDeviceSize offset = 0;
  vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
  vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vertexBuffer, &offset);
  vkCmdBindIndexBuffer(commandBuffer, indexBuffer, 0, VK_INDEX_TYPE_UINT32);
  vkCmdPushConstants(commandBuffer, pipelineLayout, VK_SHADER_STAGE_VERTEX_BIT,
                     0, sizeof(ObjectPushConstant), &pushConstant);
  vkCmdDrawIndexed(commandBuffer, draw.indexCount, 1, draw.firstIndex,
                   draw.vertexOffset, 0);
}
