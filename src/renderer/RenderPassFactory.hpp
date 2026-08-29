#pragma once

#include "RenderPassConfig.hpp"
#include <array>
#include <stdexcept>
#include <vulkan/vulkan.h>

inline VkRenderPass createCadRenderPass(VkDevice device,
                                        VkFormat colorFormat,
                                        VkFormat depthFormat) {
  std::array<VkAttachmentDescription, 2> attachments{};
  attachments[0].format = colorFormat;
  attachments[0].samples = VK_SAMPLE_COUNT_1_BIT;
  attachments[0].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  attachments[0].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
  attachments[0].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
  attachments[0].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
  attachments[0].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  attachments[0].finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

  attachments[1].format = depthFormat;
  attachments[1].samples = VK_SAMPLE_COUNT_1_BIT;
  attachments[1].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  attachments[1].storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
  attachments[1].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
  attachments[1].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
  attachments[1].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  attachments[1].finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

  VkAttachmentReference color{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
  VkAttachmentReference depth{1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};
  VkSubpassDescription subpass{};
  subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
  subpass.colorAttachmentCount = 1;
  subpass.pColorAttachments = &color;
  subpass.pDepthStencilAttachment = &depth;

  VkSubpassDependency dependency{};
  dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
  dependency.dstSubpass = 0;
  dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
                            VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
  dependency.dstStageMask = dependency.srcStageMask;
  dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT |
                             VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

  VkRenderPassCreateInfo info{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
  info.attachmentCount = static_cast<uint32_t>(attachments.size());
  info.pAttachments = attachments.data();
  info.subpassCount = 1;
  info.pSubpasses = &subpass;
  info.dependencyCount = 1;
  info.pDependencies = &dependency;

  VkRenderPass pass = VK_NULL_HANDLE;
  if (vkCreateRenderPass(device, &info, nullptr, &pass) != VK_SUCCESS)
    throw std::runtime_error("Cannot create CAO Vulkan render pass");
  return pass;
}
