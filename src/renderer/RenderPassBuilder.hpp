#pragma once

#include <vulkan/vulkan.h>

namespace cao::vk {

struct RenderPassBundle {
  VkRenderPass pass = VK_NULL_HANDLE;
  VkFormat colorFormat = VK_FORMAT_UNDEFINED;
  VkFormat depthFormat = VK_FORMAT_UNDEFINED;
};

inline VkAttachmentDescription colorAttachment(VkFormat format) {
  VkAttachmentDescription a{};
  a.format = format;
  a.samples = VK_SAMPLE_COUNT_1_BIT;
  a.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  a.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
  a.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
  a.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
  a.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  a.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
  return a;
}

inline VkAttachmentDescription depthAttachment(VkFormat format) {
  VkAttachmentDescription a{};
  a.format = format;
  a.samples = VK_SAMPLE_COUNT_1_BIT;
  a.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  a.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
  a.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
  a.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
  a.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  a.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
  return a;
}

inline VkRenderPassCreateInfo makeRenderPassInfo(
    VkAttachmentDescription* attachments,
    uint32_t attachmentCount,
    VkSubpassDescription* subpass,
    VkSubpassDependency* dependency) {
  VkRenderPassCreateInfo info{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
  info.attachmentCount = attachmentCount;
  info.pAttachments = attachments;
  info.subpassCount = 1;
  info.pSubpasses = subpass;
  info.dependencyCount = dependency ? 1u : 0u;
  info.pDependencies = dependency;
  return info;
}

inline VkSubpassDescription graphicsSubpass(
    VkAttachmentReference* color,
    VkAttachmentReference* depth) {
  VkSubpassDescription sub{};
  sub.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
  sub.colorAttachmentCount = 1;
  sub.pColorAttachments = color;
  sub.pDepthStencilAttachment = depth;
  return sub;
}

inline VkSubpassDependency externalDependency() {
  VkSubpassDependency d{};
  d.srcSubpass = VK_SUBPASS_EXTERNAL;
  d.dstSubpass = 0;
  d.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
                   VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
  d.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
                   VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
  d.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT |
                    VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
  return d;
}

} // namespace cao::vk
