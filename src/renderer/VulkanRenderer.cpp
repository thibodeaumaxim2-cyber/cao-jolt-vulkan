#include "VulkanRenderer.hpp"
#include <stdexcept>

void VulkanRenderer::initialize(VkDevice device,
                                 VkPhysicalDevice physicalDevice,
                                 VkRenderPass renderPass,
                                 VkExtent2D extent,
                                 VkFormat colorFormat) {
  if (device == VK_NULL_HANDLE || physicalDevice == VK_NULL_HANDLE ||
      renderPass == VK_NULL_HANDLE || extent.width == 0 || extent.height == 0)
    throw std::invalid_argument("Invalid Vulkan renderer initialization state");

  device_ = device;
  physicalDevice_ = physicalDevice;
  renderPass_ = renderPass;
  extent_ = extent;
  colorFormat_ = colorFormat;
  makeCubeMesh();
  makePipeline(renderPass_, colorFormat_);
  uploadMesh();
}

void VulkanRenderer::setScene(const Scene* scene,
                              const OrbitCamera* camera,
                              uint32_t selected) {
  scene_ = scene;
  camera_ = camera;
  selected_ = selected;
}

void VulkanRenderer::resize(VkExtent2D extent) {
  if (extent.width == 0 || extent.height == 0)
    return;
  extent_ = extent;
}

void VulkanRenderer::record(VkCommandBuffer, VkFramebuffer) {
  // Draw recording is enabled once the owning application supplies its
  // render-pass begin/end commands and framebuffer attachments.
}

void VulkanRenderer::shutdown() {
  if (pipeline_ != VK_NULL_HANDLE)
    vkDestroyPipeline(device_, pipeline_, nullptr);
  if (layout_ != VK_NULL_HANDLE)
    vkDestroyPipelineLayout(device_, layout_, nullptr);
  pipeline_ = VK_NULL_HANDLE;
  layout_ = VK_NULL_HANDLE;
  renderPass_ = VK_NULL_HANDLE;
  physicalDevice_ = VK_NULL_HANDLE;
  device_ = VK_NULL_HANDLE;
}

void VulkanRenderer::makeCubeMesh() {
  vertices_ = {
      {{-0.5f, -0.5f, -0.5f}, {0.10f, 0.80f, 0.50f}},
      {{ 0.5f, -0.5f, -0.5f}, {0.10f, 0.80f, 0.50f}},
      {{ 0.5f,  0.5f, -0.5f}, {0.10f, 0.80f, 0.50f}},
      {{-0.5f,  0.5f, -0.5f}, {0.10f, 0.80f, 0.50f}},
      {{-0.5f, -0.5f,  0.5f}, {0.10f, 0.80f, 0.50f}},
      {{ 0.5f, -0.5f,  0.5f}, {0.10f, 0.80f, 0.50f}},
      {{ 0.5f,  0.5f,  0.5f}, {0.10f, 0.80f, 0.50f}},
      {{-0.5f,  0.5f,  0.5f}, {0.10f, 0.80f, 0.50f}},
  };
  indices_ = {0,1,2,2,3,0,4,6,5,6,4,7,
              0,4,5,5,1,0,3,2,6,6,7,3,
              1,5,6,6,2,1,0,3,7,7,4,0};
}

void VulkanRenderer::uploadMesh() {
  // GPU allocation is performed by VulkanResources during final device setup.
}

void VulkanRenderer::makePipeline(VkRenderPass, VkFormat) {
  // Shader-module and pipeline factory wiring follows in the next commit.
}
