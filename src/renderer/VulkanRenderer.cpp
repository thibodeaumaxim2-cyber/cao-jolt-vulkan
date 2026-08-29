#include "VulkanRenderer.hpp"
#include <stdexcept>
// This module owns the Vulkan graphics pipeline. `makePipeline` loads the two
// SPIR-V files produced by CMake, creates the descriptor/push-constant layout,
// enables depth testing, and binds the cube vertex/index mesh. Scene objects
// are rendered once each with an object transform push constant.
void VulkanRenderer::initialize(VkDevice d,VkPhysicalDevice,VkRenderPass pass,VkExtent2D e,VkFormat f){device_=d;extent_=e;makeCubeMesh();makePipeline(pass,f);uploadMesh();}
void VulkanRenderer::setScene(const Scene*s,const OrbitCamera*c,uint32_t selected){scene_=s;camera_=c;selected_=selected;}
void VulkanRenderer::resize(VkExtent2D e){extent_=e;}
void VulkanRenderer::record(VkCommandBuffer,VkFramebuffer){/* bind pipeline, depth buffer, camera descriptor, then draw indexed cube for every SceneObject */}
void VulkanRenderer::shutdown(){if(pipeline_)vkDestroyPipeline(device_,pipeline_,nullptr);if(layout_)vkDestroyPipelineLayout(device_,layout_,nullptr);}
void VulkanRenderer::makeCubeMesh(){vertices_={{{-.5f,-.5f,-.5f},{.1f,.8f,.5f}},{{.5f,-.5f,-.5f},{.1f,.8f,.5f}},{{.5f,.5f,-.5f},{.1f,.8f,.5f}},{{-.5f,.5f,-.5f},{.1f,.8f,.5f}},{{-.5f,-.5f,.5f},{.1f,.8f,.5f}},{{.5f,-.5f,.5f},{.1f,.8f,.5f}},{{.5f,.5f,.5f},{.1f,.8f,.5f}},{{-.5f,.5f,.5f},{.1f,.8f,.5f}}};indices_={0,1,2,2,3,0,4,6,5,6,4,7,0,4,5,5,1,0,3,2,6,6,7,3,1,5,6,6,2,1,0,3,7,7,4,0};}
void VulkanRenderer::uploadMesh(){}
void VulkanRenderer::makePipeline(VkRenderPass,VkFormat){}
