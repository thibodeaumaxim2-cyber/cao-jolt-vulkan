#pragma once

#include "editor/Scene.hpp"
#include "editor/Camera.hpp"
#include "Vertex.hpp"
#include <vulkan/vulkan.h>
#include <vector>

class VulkanRenderer {
 public:
  void initialize(VkDevice, VkPhysicalDevice, VkRenderPass, VkExtent2D, VkFormat);
  void setScene(const Scene*, const OrbitCamera*, uint32_t selected);
  void record(VkCommandBuffer, VkFramebuffer);
  void resize(VkExtent2D);
  void shutdown();

 private:
  VkDevice device_ = VK_NULL_HANDLE;
  VkPipeline pipeline_ = VK_NULL_HANDLE;
  VkPipelineLayout layout_ = VK_NULL_HANDLE;
  VkBuffer vertexBuffer_ = VK_NULL_HANDLE;
  VkBuffer indexBuffer_ = VK_NULL_HANDLE;
  VkDeviceMemory vertexMemory_ = VK_NULL_HANDLE;
  VkDeviceMemory indexMemory_ = VK_NULL_HANDLE;
  VkExtent2D extent_{};
  const Scene* scene_ = nullptr;
  const OrbitCamera* camera_ = nullptr;
  uint32_t selected_ = 0;
  std::vector<CadVertex> vertices_;
  std::vector<uint32_t> indices_;

  void makeCubeMesh();
  void uploadMesh();
  void makePipeline(VkRenderPass, VkFormat);
};
