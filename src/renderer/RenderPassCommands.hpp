#pragma once

#include <array>
#include <vulkan/vulkan.h>

inline void beginCadRenderPass(VkCommandBuffer commandBuffer,
                               VkRenderPass renderPass,
                               VkFramebuffer framebuffer,
                               VkExtent2D extent,
                               VkClearColorValue clearColor = {{0.035f, 0.055f, 0.10f, 1.0f}},
                               float clearDepth = 1.0f) {
  std::array<VkClearValue, 2> clears{};
  clears[0].color = clearColor;
  clears[1].depthStencil = {clearDepth, 0};

  VkRenderPassBeginInfo begin{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
  begin.renderPass = renderPass;
  begin.framebuffer = framebuffer;
  begin.renderArea.offset = {0, 0};
  begin.renderArea.extent = extent;
  begin.clearValueCount = static_cast<uint32_t>(clears.size());
  begin.pClearValues = clears.data();
  vkCmdBeginRenderPass(commandBuffer, &begin, VK_SUBPASS_CONTENTS_INLINE);
}

inline void endCadRenderPass(VkCommandBuffer commandBuffer) {
  vkCmdEndRenderPass(commandBuffer);
}
