#define GLFW_INCLUDE_NONE
#include <vulkan/vulkan.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_vulkan.h>
#include "editor/JoltBridge.hpp"
#include "editor/Scene.hpp"
#include "editor/RobotExport.hpp"
#include "editor/RobotRecorder.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

static float gYaw = 0.55f, gPitch = -0.55f, gZoom = 1.0f, gPanX = 0.0f, gPanY = 0.0f;
static bool gDragging = false, gPanning = false, gSimulationRunning = false;
static int gTintMode = 0;
static bool gBuildRequested = false, gDemoRequested = false, gUiReady = false;
static int gCreatePrimitive = -1;
static uint32_t gSelectedId = 1;
static int gRobotScript = 0;
static bool gDeleteRequested = false, gPhysicsRebuildRequested = false;
static bool gRobotParametersExported = false;
static double gLastX = 0.0, gLastY = 0.0;
static void cursor(GLFWwindow *window, double x, double y) {
  if (gUiReady) ImGui_ImplGlfw_CursorPosCallback(window, x, y);
  if (gDragging) { gYaw += static_cast<float>(x - gLastX) * 0.008f; gPitch = std::clamp(gPitch + static_cast<float>(y - gLastY) * 0.006f, -1.25f, 0.15f); }
  if (gPanning) { gPanX += static_cast<float>(x - gLastX) * 0.002f; gPanY -= static_cast<float>(y - gLastY) * 0.002f; }
  gLastX = x; gLastY = y;
}
static void mouseButton(GLFWwindow *window, int button, int action, int mods) {
  if (gUiReady) ImGui_ImplGlfw_MouseButtonCallback(window, button, action, mods);
  if (gUiReady && ImGui::GetIO().WantCaptureMouse) return;
  if (button == GLFW_MOUSE_BUTTON_LEFT)
    gDragging = action == GLFW_PRESS;
  if (button == GLFW_MOUSE_BUTTON_MIDDLE)
    gPanning = action == GLFW_PRESS;
  glfwGetCursorPos(window, &gLastX, &gLastY);
}
static void key(GLFWwindow *window, int key, int scancode, int action, int mods) {
  if (gUiReady) ImGui_ImplGlfw_KeyCallback(window, key, scancode, action, mods);
  if (gUiReady && ImGui::GetIO().WantCaptureKeyboard) return;
  if (action != GLFW_PRESS) return;
  if (key == GLFW_KEY_R) {
    gYaw = 0.55f; gPitch = -0.55f; gZoom = 1.0f; gPanX = 0.0f; gPanY = 0.0f;
  } else if (key == GLFW_KEY_SPACE) {
    gSimulationRunning = !gSimulationRunning;
  } else if (key == GLFW_KEY_H) {
    gTintMode = (gTintMode + 1) % 3;
  } else if (key == GLFW_KEY_B) {
    gBuildRequested = true;
  } else if (key == GLFW_KEY_D) {
    gDemoRequested = true;
  }
}

static void scroll(GLFWwindow *window, double x, double y) {
  if (gUiReady) ImGui_ImplGlfw_ScrollCallback(window, x, y);
  if (gUiReady && ImGui::GetIO().WantCaptureMouse) return;
  gZoom = std::clamp(gZoom * (1.0f + static_cast<float>(y) * 0.10f), 0.35f, 2.5f);
}

static void check(VkResult result, const char *what) {
  if (result != VK_SUCCESS)
    throw std::runtime_error(std::string(what) + " (" + std::to_string(result) + ")");
}

struct Vertex { float position[3]; float color[3]; };
struct Push { float mvp[16]; float tint[4]; };
struct Mat4 { float v[16]{}; };
static Mat4 identity() { Mat4 m{}; m.v[0]=m.v[5]=m.v[10]=m.v[15]=1.0f; return m; }
static Mat4 multiply(const Mat4 &a, const Mat4 &b) {
  Mat4 r{};
  for (int c=0;c<4;++c) for (int row=0;row<4;++row) for (int k=0;k<4;++k)
    r.v[c*4+row] += a.v[k*4+row] * b.v[c*4+k];
  return r;
}
static Mat4 translate(float x,float y,float z) { Mat4 m=identity(); m.v[12]=x; m.v[13]=y; m.v[14]=z; return m; }
static Mat4 rotateX(float a) { Mat4 m=identity(); float c=std::cos(a), sn=std::sin(a); m.v[5]=c;m.v[6]=sn;m.v[9]=-sn;m.v[10]=c;return m; }
static Mat4 rotateY(float a) { Mat4 m=identity(); float c=std::cos(a), sn=std::sin(a); m.v[0]=c;m.v[2]=-sn;m.v[8]=sn;m.v[10]=c;return m; }
static Mat4 rotateZ(float a) { Mat4 m=identity(); float c=std::cos(a), sn=std::sin(a); m.v[0]=c;m.v[1]=sn;m.v[4]=-sn;m.v[5]=c;return m; }
static Mat4 scale(float s) { Mat4 m=identity();m.v[0]=m.v[5]=m.v[10]=s;return m; }
static Mat4 scale(float x,float y,float z) { Mat4 m=identity();m.v[0]=x;m.v[5]=y;m.v[10]=z;return m; }
static Mat4 perspective(float fovy,float aspect,float nearPlane,float farPlane) {
  Mat4 m{}; float f=1.0f/std::tan(fovy*0.5f); m.v[0]=f/aspect;m.v[5]=-f;m.v[10]=farPlane/(nearPlane-farPlane);m.v[11]=-1.0f;m.v[14]=(farPlane*nearPlane)/(nearPlane-farPlane);return m;
}

static uint32_t memoryType(VkPhysicalDevice gpu, uint32_t bits,
                           VkMemoryPropertyFlags properties) {
  VkPhysicalDeviceMemoryProperties memory{};
  vkGetPhysicalDeviceMemoryProperties(gpu, &memory);
  for (uint32_t i = 0; i < memory.memoryTypeCount; ++i)
    if ((bits & (1u << i)) &&
        (memory.memoryTypes[i].propertyFlags & properties) == properties)
      return i;
  throw std::runtime_error("No compatible Vulkan memory type");
}

static VkFormat depthFormat(VkPhysicalDevice gpu) {
  for (VkFormat format : {VK_FORMAT_D32_SFLOAT, VK_FORMAT_D32_SFLOAT_S8_UINT,
                          VK_FORMAT_D24_UNORM_S8_UINT}) {
    VkFormatProperties properties{};
    vkGetPhysicalDeviceFormatProperties(gpu, format, &properties);
    if (properties.optimalTilingFeatures &
        VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT)
      return format;
  }
  throw std::runtime_error("No Vulkan depth format");
}

static std::vector<char> readFile(const char *name) {
  std::filesystem::path path{name};
  if (!std::filesystem::exists(path))
    path = std::filesystem::path("build") / name;
  std::ifstream file(path, std::ios::ate | std::ios::binary);
  if (!file) throw std::runtime_error("Cannot open shader: " + path.string());
  const auto size = file.tellg();
  if (size <= 0 || (size % 4) != 0) throw std::runtime_error("Invalid shader");
  std::vector<char> data(static_cast<size_t>(size));
  file.seekg(0); file.read(data.data(), size);
  return data;
}

static VkShaderModule shader(VkDevice device, const char *name) {
  const auto code = readFile(name);
  VkShaderModuleCreateInfo info{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
  info.codeSize = code.size();
  info.pCode = reinterpret_cast<const uint32_t *>(code.data());
  VkShaderModule out = VK_NULL_HANDLE;
  check(vkCreateShaderModule(device, &info, nullptr, &out), "shader module");
  return out;
}

int main() {
  try {
    if (!glfwInit()) return 1;
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    GLFWwindow *window = glfwCreateWindow(1280, 800, "Jolt Robot Simulator - Triangle", nullptr, nullptr);
    if (!window) throw std::runtime_error("Cannot create window");
    JoltBridge physics;
    physics.initialize();
    Scene scene;
    scene.buildQuadruped();
    physics.rebuild(scene);
    RobotFrameRecorder frameRecorder{5.0f};
    bool simulationWasRunning = false;
    bool frameRecordingSaved = false;
    bool frameRecordingWriteAttempted = false;
    glfwSetCursorPosCallback(window, cursor);
    glfwSetMouseButtonCallback(window, mouseButton);
    glfwSetScrollCallback(window, scroll);
    glfwSetKeyCallback(window, key);

    uint32_t extensionCount = 0;
    const char **extensions = glfwGetRequiredInstanceExtensions(&extensionCount);
    VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO};
    app.pApplicationName = "CAO Jolt Vulkan";
    app.apiVersion = VK_API_VERSION_1_0;
    VkInstanceCreateInfo instanceInfo{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
    instanceInfo.pApplicationInfo = &app;
    instanceInfo.enabledExtensionCount = extensionCount;
    instanceInfo.ppEnabledExtensionNames = extensions;
    VkInstance instance = VK_NULL_HANDLE;
    check(vkCreateInstance(&instanceInfo, nullptr, &instance), "instance");

    VkSurfaceKHR surface = VK_NULL_HANDLE;
    check(glfwCreateWindowSurface(instance, window, nullptr, &surface), "surface");

    uint32_t gpuCount = 0; vkEnumeratePhysicalDevices(instance, &gpuCount, nullptr);
    std::vector<VkPhysicalDevice> gpus(gpuCount);
    vkEnumeratePhysicalDevices(instance, &gpuCount, gpus.data());
    VkPhysicalDevice gpu = VK_NULL_HANDLE; uint32_t family = 0;
    for (VkPhysicalDevice candidate : gpus) {
      uint32_t count = 0; vkGetPhysicalDeviceQueueFamilyProperties(candidate, &count, nullptr);
      std::vector<VkQueueFamilyProperties> families(count);
      vkGetPhysicalDeviceQueueFamilyProperties(candidate, &count, families.data());
      for (uint32_t i = 0; i < count; ++i) {
        VkBool32 present = VK_FALSE;
        vkGetPhysicalDeviceSurfaceSupportKHR(candidate, i, surface, &present);
        if (present && (families[i].queueFlags & VK_QUEUE_GRAPHICS_BIT)) {
          gpu = candidate; family = i; break;
        }
      }
      if (gpu) break;
    }
    if (!gpu) throw std::runtime_error("No graphics/present Vulkan device");

    const float priority = 1.0f;
    VkDeviceQueueCreateInfo queueInfo{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
    queueInfo.queueFamilyIndex = family; queueInfo.queueCount = 1; queueInfo.pQueuePriorities = &priority;
    const char *deviceExtensions[] = {VK_KHR_SWAPCHAIN_EXTENSION_NAME};
    VkDeviceCreateInfo deviceInfo{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
    deviceInfo.queueCreateInfoCount = 1; deviceInfo.pQueueCreateInfos = &queueInfo;
    deviceInfo.enabledExtensionCount = 1; deviceInfo.ppEnabledExtensionNames = deviceExtensions;
    VkDevice device = VK_NULL_HANDLE; check(vkCreateDevice(gpu, &deviceInfo, nullptr, &device), "device");
    VkQueue queue = VK_NULL_HANDLE; vkGetDeviceQueue(device, family, 0, &queue);

    VkSurfaceCapabilitiesKHR caps{}; vkGetPhysicalDeviceSurfaceCapabilitiesKHR(gpu, surface, &caps);
    uint32_t formatCount = 0; vkGetPhysicalDeviceSurfaceFormatsKHR(gpu, surface, &formatCount, nullptr);
    std::vector<VkSurfaceFormatKHR> formats(formatCount);
    vkGetPhysicalDeviceSurfaceFormatsKHR(gpu, surface, &formatCount, formats.data());
    VkSurfaceFormatKHR format = formats.front();
    for (const auto &f : formats)
      if (f.format == VK_FORMAT_B8G8R8A8_SRGB && f.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) format = f;
    int width = 0, height = 0; glfwGetFramebufferSize(window, &width, &height);
    VkExtent2D extent = caps.currentExtent.width == UINT32_MAX
      ? VkExtent2D{std::clamp(uint32_t(width), caps.minImageExtent.width, caps.maxImageExtent.width),
                   std::clamp(uint32_t(height), caps.minImageExtent.height, caps.maxImageExtent.height)}
      : caps.currentExtent;
    uint32_t imageCount = std::max(2u, caps.minImageCount);
    if (caps.maxImageCount) imageCount = std::min(imageCount, caps.maxImageCount);
    VkSwapchainCreateInfoKHR swapInfo{VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR};
    swapInfo.surface = surface; swapInfo.minImageCount = imageCount; swapInfo.imageFormat = format.format;
    swapInfo.imageColorSpace = format.colorSpace; swapInfo.imageExtent = extent; swapInfo.imageArrayLayers = 1;
    swapInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT; swapInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    swapInfo.preTransform = caps.currentTransform; swapInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    swapInfo.presentMode = VK_PRESENT_MODE_FIFO_KHR; swapInfo.clipped = VK_TRUE;
    VkSwapchainKHR swapchain = VK_NULL_HANDLE; check(vkCreateSwapchainKHR(device, &swapInfo, nullptr, &swapchain), "swapchain");

    uint32_t swapImageCount = 0; vkGetSwapchainImagesKHR(device, swapchain, &swapImageCount, nullptr);
    std::vector<VkImage> images(swapImageCount); vkGetSwapchainImagesKHR(device, swapchain, &swapImageCount, images.data());
    std::vector<VkImageView> views(swapImageCount);
    for (uint32_t i = 0; i < swapImageCount; ++i) {
      VkImageViewCreateInfo viewInfo{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
      viewInfo.image = images[i]; viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D; viewInfo.format = format.format;
      viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT; viewInfo.subresourceRange.levelCount = 1; viewInfo.subresourceRange.layerCount = 1;
      check(vkCreateImageView(device, &viewInfo, nullptr, &views[i]), "image view");
    }

    const VkFormat depth = depthFormat(gpu);
    VkImage depthImage = VK_NULL_HANDLE; VkDeviceMemory depthMemory = VK_NULL_HANDLE; VkImageView depthView = VK_NULL_HANDLE;
    VkImageCreateInfo depthImageInfo{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
    depthImageInfo.imageType = VK_IMAGE_TYPE_2D; depthImageInfo.format = depth; depthImageInfo.extent = {extent.width, extent.height, 1};
    depthImageInfo.mipLevels = 1; depthImageInfo.arrayLayers = 1; depthImageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
    depthImageInfo.tiling = VK_IMAGE_TILING_OPTIMAL; depthImageInfo.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
    check(vkCreateImage(device, &depthImageInfo, nullptr, &depthImage), "depth image");
    VkMemoryRequirements depthRequirements{}; vkGetImageMemoryRequirements(device, depthImage, &depthRequirements);
    VkMemoryAllocateInfo depthAllocation{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO}; depthAllocation.allocationSize = depthRequirements.size;
    depthAllocation.memoryTypeIndex = memoryType(gpu, depthRequirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    check(vkAllocateMemory(device, &depthAllocation, nullptr, &depthMemory), "depth memory");
    check(vkBindImageMemory(device, depthImage, depthMemory, 0), "bind depth");
    VkImageViewCreateInfo depthViewInfo{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO}; depthViewInfo.image = depthImage;
    depthViewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D; depthViewInfo.format = depth; depthViewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
    depthViewInfo.subresourceRange.levelCount = 1; depthViewInfo.subresourceRange.layerCount = 1;
    check(vkCreateImageView(device, &depthViewInfo, nullptr, &depthView), "depth view");

    VkAttachmentDescription color{}; color.format = format.format; color.samples = VK_SAMPLE_COUNT_1_BIT;
    color.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR; color.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    color.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED; color.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    VkAttachmentDescription depthAttachment{}; depthAttachment.format = depth; depthAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR; depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depthAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED; depthAttachment.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    std::array<VkAttachmentDescription, 2> attachments{{color, depthAttachment}};
    VkAttachmentReference colorRef{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkAttachmentReference depthRef{1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};
    VkSubpassDescription subpass{}; subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1; subpass.pColorAttachments = &colorRef; subpass.pDepthStencilAttachment = &depthRef;
    VkSubpassDependency dependency{}; dependency.srcSubpass = VK_SUBPASS_EXTERNAL; dependency.dstSubpass = 0;
    dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependency.dstStageMask = dependency.srcStageMask;
    dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    VkRenderPassCreateInfo passInfo{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
    passInfo.attachmentCount = static_cast<uint32_t>(attachments.size()); passInfo.pAttachments = attachments.data(); passInfo.subpassCount = 1; passInfo.pSubpasses = &subpass;
    passInfo.dependencyCount = 1; passInfo.pDependencies = &dependency;
    VkRenderPass renderPass = VK_NULL_HANDLE; check(vkCreateRenderPass(device, &passInfo, nullptr, &renderPass), "render pass");

    std::vector<VkFramebuffer> framebuffers(swapImageCount);
    for (uint32_t i = 0; i < swapImageCount; ++i) {
      std::array<VkImageView, 2> attachmentsForFrame{{views[i], depthView}};
      VkFramebufferCreateInfo info{VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
      info.renderPass = renderPass; info.attachmentCount = static_cast<uint32_t>(attachmentsForFrame.size()); info.pAttachments = attachmentsForFrame.data();
      info.width = extent.width; info.height = extent.height; info.layers = 1;
      check(vkCreateFramebuffer(device, &info, nullptr, &framebuffers[i]), "framebuffer");
    }

    VkPushConstantRange pushRange{VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(Push)};
    VkPipelineLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    layoutInfo.pushConstantRangeCount = 1; layoutInfo.pPushConstantRanges = &pushRange;
    VkPipelineLayout layout = VK_NULL_HANDLE; check(vkCreatePipelineLayout(device, &layoutInfo, nullptr, &layout), "pipeline layout");
    VkShaderModule vertexShader = shader(device, "cad.vert.spv"), fragmentShader = shader(device, "cad.frag.spv");
    VkPipelineShaderStageCreateInfo stages[2]{{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO},{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO}};
    stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT; stages[0].module = vertexShader; stages[0].pName = "main";
    stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT; stages[1].module = fragmentShader; stages[1].pName = "main";
    VkVertexInputBindingDescription binding{0, sizeof(Vertex), VK_VERTEX_INPUT_RATE_VERTEX};
    VkVertexInputAttributeDescription attributes[2]{{0,0,VK_FORMAT_R32G32B32_SFLOAT,0},{1,0,VK_FORMAT_R32G32B32_SFLOAT,12}};
    VkPipelineVertexInputStateCreateInfo vertexInput{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    vertexInput.vertexBindingDescriptionCount = 1; vertexInput.pVertexBindingDescriptions = &binding;
    vertexInput.vertexAttributeDescriptionCount = 2; vertexInput.pVertexAttributeDescriptions = attributes;
    VkPipelineInputAssemblyStateCreateInfo assembly{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    assembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    VkViewport viewport{0,0,float(extent.width),float(extent.height),0,1}; VkRect2D scissor{{0,0},extent};
    VkPipelineViewportStateCreateInfo viewportState{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    viewportState.viewportCount=1; viewportState.pViewports=&viewport; viewportState.scissorCount=1; viewportState.pScissors=&scissor;
    VkPipelineRasterizationStateCreateInfo raster{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    raster.polygonMode=VK_POLYGON_MODE_FILL; raster.cullMode=VK_CULL_MODE_NONE; raster.frontFace=VK_FRONT_FACE_COUNTER_CLOCKWISE; raster.lineWidth=1;
    VkPipelineMultisampleStateCreateInfo multisample{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO}; multisample.rasterizationSamples=VK_SAMPLE_COUNT_1_BIT;
    VkPipelineDepthStencilStateCreateInfo depthState{VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
    depthState.depthTestEnable = VK_TRUE; depthState.depthWriteEnable = VK_TRUE; depthState.depthCompareOp = VK_COMPARE_OP_LESS;
    VkPipelineColorBlendAttachmentState blendAttachment{}; blendAttachment.colorWriteMask=VK_COLOR_COMPONENT_R_BIT|VK_COLOR_COMPONENT_G_BIT|VK_COLOR_COMPONENT_B_BIT|VK_COLOR_COMPONENT_A_BIT;
    VkPipelineColorBlendStateCreateInfo blend{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO}; blend.attachmentCount=1; blend.pAttachments=&blendAttachment;
    VkGraphicsPipelineCreateInfo pipelineInfo{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    pipelineInfo.stageCount=2; pipelineInfo.pStages=stages; pipelineInfo.pVertexInputState=&vertexInput; pipelineInfo.pInputAssemblyState=&assembly;
    pipelineInfo.pViewportState=&viewportState; pipelineInfo.pRasterizationState=&raster; pipelineInfo.pMultisampleState=&multisample;
    pipelineInfo.pColorBlendState=&blend; pipelineInfo.pDepthStencilState=&depthState; pipelineInfo.layout=layout; pipelineInfo.renderPass=renderPass;
    VkPipeline pipeline=VK_NULL_HANDLE; check(vkCreateGraphicsPipelines(device,VK_NULL_HANDLE,1,&pipelineInfo,nullptr,&pipeline),"graphics pipeline");
    vkDestroyShaderModule(device, fragmentShader, nullptr); vkDestroyShaderModule(device, vertexShader, nullptr);

    // CAO scene geometry: a reusable cube mesh plus a separate indexed grid.
    // Every pyramid object is drawn independently, so the Jolt bridge can move it.
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    struct DrawItem { uint32_t firstIndex; uint32_t indexCount; uint32_t objectId; };
    std::vector<DrawItem> objectDraws;

    auto addBlock = [&](const std::array<float, 3> &color, uint32_t objectId) {
      const uint32_t firstVertex = static_cast<uint32_t>(vertices.size());
      const uint32_t firstIndex = static_cast<uint32_t>(indices.size());
      constexpr float h = 0.46f;
      const std::array<Vertex, 8> cube{{
          {{-h,-h,-h}, {color[0] * 0.52f, color[1] * 0.52f, color[2] * 0.52f}},
          {{ h,-h,-h}, {color[0] * 0.52f, color[1] * 0.52f, color[2] * 0.52f}},
          {{ h, h,-h}, {color[0] * 0.52f, color[1] * 0.52f, color[2] * 0.52f}},
          {{-h, h,-h}, {color[0] * 0.52f, color[1] * 0.52f, color[2] * 0.52f}},
          {{-h,-h, h}, {color[0] * 0.78f, color[1] * 0.78f, color[2] * 0.78f}},
          {{ h,-h, h}, {color[0] * 0.78f, color[1] * 0.78f, color[2] * 0.78f}},
          {{ h, h, h}, {color[0], color[1], color[2]}},
          {{-h, h, h}, {color[0], color[1], color[2]}}
      }};
      vertices.insert(vertices.end(), cube.begin(), cube.end());
      indices.insert(indices.end(), {
          firstVertex, firstVertex+1, firstVertex+2, firstVertex+2, firstVertex+3, firstVertex,
          firstVertex+4, firstVertex+6, firstVertex+5, firstVertex+6, firstVertex+4, firstVertex+7,
          firstVertex, firstVertex+4, firstVertex+5, firstVertex+5, firstVertex+1, firstVertex,
          firstVertex+1, firstVertex+5, firstVertex+6, firstVertex+6, firstVertex+2, firstVertex+1,
          firstVertex+2, firstVertex+6, firstVertex+7, firstVertex+7, firstVertex+3, firstVertex+2,
          firstVertex+3, firstVertex+7, firstVertex+4, firstVertex+4, firstVertex, firstVertex+3
      });
      objectDraws.push_back({firstIndex, 36u, objectId});
    };
    auto addGridStrip = [&](float x0, float z0, float x1, float z1,
                            float width, const std::array<float, 3> &color) {
      const uint32_t first = static_cast<uint32_t>(vertices.size());
      const float y = -0.01f;
      const float dx = x1 - x0, dz = z1 - z0;
      const float length = std::sqrt(dx * dx + dz * dz);
      const float ox = -dz / length * width, oz = dx / length * width;
      vertices.insert(vertices.end(), {
          {{x0 + ox, y, z0 + oz}, {color[0], color[1], color[2]}},
          {{x1 + ox, y, z1 + oz}, {color[0], color[1], color[2]}},
          {{x1 - ox, y, z1 - oz}, {color[0], color[1], color[2]}},
          {{x0 - ox, y, z0 - oz}, {color[0], color[1], color[2]}}
      });
      indices.insert(indices.end(), {first, first+1, first+2, first+2, first+3, first});
    };
    constexpr float gridExtent = 5.0f, gridStep = 0.5f;
    const std::array<float, 3> gridColor{{0.12f, 0.24f, 0.31f}};
    for (int i = -10; i <= 10; ++i) {
      const float offset = static_cast<float>(i) * gridStep;
      addGridStrip(-gridExtent, offset, gridExtent, offset, 0.010f, gridColor);
      addGridStrip(offset, -gridExtent, offset, gridExtent, 0.010f, gridColor);
    }
    addGridStrip(-gridExtent, 0.0f, gridExtent, 0.0f, 0.025f, {{0.92f, 0.18f, 0.18f}});
    addGridStrip(0.0f, -gridExtent, 0.0f, gridExtent, 0.025f, {{0.18f, 0.76f, 0.32f}});
    const uint32_t gridIndexCount = static_cast<uint32_t>(indices.size());

    const std::array<std::array<float, 3>, 3> layerColors{{
        {{0.10f, 0.72f, 0.95f}}, {{0.18f, 0.88f, 0.62f}}, {{1.00f, 0.72f, 0.16f}}
    }};
    for (const SceneObject &object : scene.objects()) {
      const int layer = std::clamp(static_cast<int>(object.transform.position.y) - 1, 0, 2);
      addBlock(layerColors[layer], object.id);
    }
    auto buffer = [&](VkDeviceSize capacity, VkBufferUsageFlags usage, const void *source, VkDeviceSize sourceSize, VkBuffer &outBuffer, VkDeviceMemory &outMemory) {
      VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO}; info.size=capacity; info.usage=usage; info.sharingMode=VK_SHARING_MODE_EXCLUSIVE;
      check(vkCreateBuffer(device,&info,nullptr,&outBuffer),"buffer"); VkMemoryRequirements requirements{}; vkGetBufferMemoryRequirements(device,outBuffer,&requirements);
      VkMemoryAllocateInfo allocation{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO}; allocation.allocationSize=requirements.size;
      allocation.memoryTypeIndex=memoryType(gpu,requirements.memoryTypeBits,VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
      check(vkAllocateMemory(device,&allocation,nullptr,&outMemory),"buffer memory"); check(vkBindBufferMemory(device,outBuffer,outMemory,0),"bind buffer");
      void *mapped=nullptr; check(vkMapMemory(device,outMemory,0,sourceSize,0,&mapped),"map buffer"); std::memcpy(mapped,source,sourceSize); vkUnmapMemory(device,outMemory);
    };
    constexpr size_t maxSceneObjects = 128;
    const VkDeviceSize vertexCapacity = (vertices.size() + maxSceneObjects * 8u) * sizeof(Vertex);
    const VkDeviceSize indexCapacity = (indices.size() + maxSceneObjects * 36u) * sizeof(uint32_t);
    VkBuffer vertexBuffer=VK_NULL_HANDLE,indexBuffer=VK_NULL_HANDLE; VkDeviceMemory vertexMemory=VK_NULL_HANDLE,indexMemory=VK_NULL_HANDLE;
    buffer(vertexCapacity,VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,vertices.data(),vertices.size()*sizeof(Vertex),vertexBuffer,vertexMemory);
    buffer(indexCapacity,VK_BUFFER_USAGE_INDEX_BUFFER_BIT,indices.data(),indices.size()*sizeof(uint32_t),indexBuffer,indexMemory);
    auto uploadSceneGeometry = [&] {
      void *mapped=nullptr;
      check(vkMapMemory(device,vertexMemory,0,vertices.size()*sizeof(Vertex),0,&mapped),"map vertices"); std::memcpy(mapped,vertices.data(),vertices.size()*sizeof(Vertex)); vkUnmapMemory(device,vertexMemory);
      check(vkMapMemory(device,indexMemory,0,indices.size()*sizeof(uint32_t),0,&mapped),"map indices"); std::memcpy(mapped,indices.data(),indices.size()*sizeof(uint32_t)); vkUnmapMemory(device,indexMemory);
    };

    VkCommandPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO}; poolInfo.flags=VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT; poolInfo.queueFamilyIndex=family;
    VkCommandPool pool=VK_NULL_HANDLE; check(vkCreateCommandPool(device,&poolInfo,nullptr,&pool),"command pool");
    std::vector<VkCommandBuffer> commands(swapImageCount); VkCommandBufferAllocateInfo commandInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    commandInfo.commandPool=pool; commandInfo.level=VK_COMMAND_BUFFER_LEVEL_PRIMARY; commandInfo.commandBufferCount=swapImageCount;
    check(vkAllocateCommandBuffers(device,&commandInfo,commands.data()),"command buffers");
    VkSemaphoreCreateInfo semInfo{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO}; VkSemaphore available,finished;
    check(vkCreateSemaphore(device,&semInfo,nullptr,&available),"available semaphore"); check(vkCreateSemaphore(device,&semInfo,nullptr,&finished),"finished semaphore");
    VkFenceCreateInfo fenceInfo{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO}; fenceInfo.flags=VK_FENCE_CREATE_SIGNALED_BIT; VkFence fence;
    check(vkCreateFence(device,&fenceInfo,nullptr,&fence),"fence");

    VkDescriptorPoolSize poolSizes[] = {
      {VK_DESCRIPTOR_TYPE_SAMPLER, 1000}, {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1000},
      {VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1000}, {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1000},
      {VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER, 1000}, {VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER, 1000},
      {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1000}, {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1000},
      {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 1000}, {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC, 1000},
      {VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1000}};
    VkDescriptorPoolCreateInfo imguiPoolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    imguiPoolInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    imguiPoolInfo.maxSets = 1000 * 11;
    imguiPoolInfo.poolSizeCount = 11; imguiPoolInfo.pPoolSizes = poolSizes;
    VkDescriptorPool imguiPool = VK_NULL_HANDLE;
    check(vkCreateDescriptorPool(device, &imguiPoolInfo, nullptr, &imguiPool), "imgui descriptor pool");
    IMGUI_CHECKVERSION(); ImGui::CreateContext(); ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForVulkan(window, false);
    ImGui_ImplVulkan_InitInfo imguiInfo{};
    imguiInfo.Instance=instance; imguiInfo.PhysicalDevice=gpu; imguiInfo.Device=device;
    imguiInfo.QueueFamily=family; imguiInfo.Queue=queue; imguiInfo.DescriptorPool=imguiPool;
    imguiInfo.MinImageCount=imageCount; imguiInfo.ImageCount=swapImageCount;
    imguiInfo.MSAASamples=VK_SAMPLE_COUNT_1_BIT; imguiInfo.RenderPass=renderPass;
    ImGui_ImplVulkan_Init(&imguiInfo);
    ImGui_ImplVulkan_CreateFontsTexture();
    gUiReady = true;

    Push drawPush{}; drawPush.tint[0]=drawPush.tint[1]=drawPush.tint[2]=drawPush.tint[3]=1;
    Mat4 cameraMvp{};
    auto updateCamera = [&] {
      const float aspect = static_cast<float>(extent.width) / static_cast<float>(extent.height);
      const Mat4 camera = multiply(translate(gPanX, gPanY - 1.15f, -8.0f),
                         multiply(rotateX(gPitch), multiply(rotateY(gYaw), scale(gZoom))));
      cameraMvp = multiply(perspective(1.05f, aspect, 0.05f, 50.0f), camera);
    };
    auto setMvp = [&](const Mat4 &model) {
      const Mat4 mvp = multiply(cameraMvp, model);
      std::memcpy(drawPush.mvp, mvp.v, sizeof(drawPush.mvp));
    };
    while (!glfwWindowShouldClose(window)) {
      glfwPollEvents();
      ImGui_ImplVulkan_NewFrame(); ImGui_ImplGlfw_NewFrame(); ImGui::NewFrame();
      if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
          if (ImGui::MenuItem("New scene")) gBuildRequested = true;
          if (ImGui::MenuItem("Exit")) glfwSetWindowShouldClose(window, GLFW_TRUE);
          ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Create")) {
          if (ImGui::MenuItem("Box")) gCreatePrimitive = static_cast<int>(Primitive::Box);
          if (ImGui::MenuItem("Cylinder")) gCreatePrimitive = static_cast<int>(Primitive::Cylinder);
          if (ImGui::MenuItem("Sphere")) gCreatePrimitive = static_cast<int>(Primitive::Sphere);
          if (ImGui::MenuItem("Beam")) gCreatePrimitive = static_cast<int>(Primitive::Beam);
          ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Simulation")) {
          if (ImGui::MenuItem(gSimulationRunning ? "Pause" : "Play", "Space")) gSimulationRunning = !gSimulationRunning;
          if (ImGui::MenuItem("Reset quadruped", "B")) gBuildRequested = true;
          if (ImGui::MenuItem("Drop robot", "D")) gDemoRequested = true;
          ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("View")) {
          if (ImGui::MenuItem("Isometric")) { gYaw=0.55f; gPitch=-0.55f; gZoom=1.0f; gPanX=gPanY=0.0f; }
          if (ImGui::MenuItem("Top")) { gYaw=0.0f; gPitch=-1.25f; gZoom=0.85f; gPanX=gPanY=0.0f; }
          if (ImGui::MenuItem("Front")) { gYaw=0.0f; gPitch=0.0f; gZoom=0.9f; gPanX=gPanY=0.0f; }
          if (ImGui::MenuItem("Right")) { gYaw=1.57f; gPitch=0.0f; gZoom=0.9f; gPanX=gPanY=0.0f; }
          ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
      }
      ImGui::SetNextWindowPos(ImVec2(12, 34), ImGuiCond_Always);
      ImGui::SetNextWindowBgAlpha(0.92f);
      ImGui::Begin("CAO Toolbar", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse);
      if (ImGui::Button("Reset robot")) gBuildRequested = true; ImGui::SameLine();
      if (ImGui::Button("Drop robot")) gDemoRequested = true; ImGui::SameLine();
      if (ImGui::Button(gSimulationRunning ? "Pause" : "Play")) gSimulationRunning = !gSimulationRunning;
      ImGui::TextDisabled("Quadruped: 16 rotary actuators | roll/pitch/knee/ankle: 55/85/75/35 N m");
      const char *robotScripts[] = {"Stand", "Walk", "Trot", "Jump"};
      if (ImGui::Combo("Motion script", &gRobotScript, robotScripts, IM_ARRAYSIZE(robotScripts))) {
        physics.setRobotScript(gRobotScript);
        gSimulationRunning = gRobotScript != 0;
      }
      ImGui::Text("Active script: %s", robotScripts[gRobotScript]);
      const RobotTelemetry &robotTelemetry = physics.telemetry();
      ImGui::Separator();
      ImGui::Text("Telemetry | torso %.3f m/s | cycle %.2f | swing leg %d",
                  robotTelemetry.torsoSpeedMps, robotTelemetry.gaitCycle,
                  robotTelemetry.activeSwingLeg);
      ImGui::Text("Limits N m: roll %.0f | hip %.0f | knee %.0f | ankle %.0f",
                  robotTelemetry.torqueLimitsNm[0], robotTelemetry.torqueLimitsNm[1],
                  robotTelemetry.torqueLimitsNm[2], robotTelemetry.torqueLimitsNm[3]);
      float maxAngleError = 0.0f; int saturated = 0;
      for (size_t leg = 0; leg < 4; ++leg)
        for (size_t joint = 0; joint < 4; ++joint)
          maxAngleError = std::max(maxAngleError, std::abs(robotTelemetry.angleErrorRad[leg][joint]));
      for (bool value : robotTelemetry.torqueSaturated) if (value) ++saturated;
      constexpr float radiansToDegrees = 57.2957795f;
      ImGui::Text("Tracking error: %.2f deg (%.3f rad) | torque-limited joints: %d/16",
                  maxAngleError * radiansToDegrees, maxAngleError, saturated);
      const float commandedKneeDeg = robotTelemetry.targetAnglesRad[0][2] * radiansToDegrees;
      const float measuredKneeDeg = robotTelemetry.measuredAnglesRad[0][2] * radiansToDegrees;
      const float kneeErrorDeg = robotTelemetry.angleErrorRad[0][2] * radiansToDegrees;
      ImGui::Text("Front-left knee command: %.1f deg", commandedKneeDeg);
      ImGui::Text("Front-left knee scene:  %.1f deg", measuredKneeDeg);
      ImGui::Text("Front-left knee error:   %.1f deg", kneeErrorDeg);
      ImGui::TextDisabled("Scene values are measured from Jolt body transforms.");
      if (robotTelemetry.activeSwingLeg >= 0)
        ImGui::Text("Swing assist: %.1f N", robotTelemetry.swingLiftForceN);
      if (ImGui::Button("Export robot JSON"))
        gRobotParametersExported = exportRobotParameters(scene, gRobotScript, "robot_parameters.json");
      if (gRobotParametersExported) ImGui::SameLine(), ImGui::TextDisabled("saved robot_parameters.json");
      if (frameRecorder.recording())
        ImGui::Text("Recording frames: %.1f / %.1f s", frameRecorder.elapsedSeconds(), frameRecorder.durationSeconds());
      else if (frameRecordingSaved)
        ImGui::TextDisabled("saved robot_recording.json");
      ImGui::End();

      ImGui::SetNextWindowPos(ImVec2(12, 110), ImGuiCond_Always);
      ImGui::SetNextWindowSize(ImVec2(235, 340), ImGuiCond_Always);
      ImGui::Begin("Object tree", nullptr, ImGuiWindowFlags_NoCollapse);
      for (const SceneObject &object : scene.objects()) {
        const bool selected = object.id == gSelectedId;
        if (ImGui::Selectable(object.name.c_str(), selected)) gSelectedId = object.id;
      }
      ImGui::End();

      ImGui::SetNextWindowPos(ImVec2(float(extent.width) - 270.0f, 34), ImGuiCond_Always);
      ImGui::SetNextWindowSize(ImVec2(258, 0), ImGuiCond_Always);
      ImGui::Begin("Properties", nullptr, ImGuiWindowFlags_NoCollapse);
      SceneObject *selected = scene.find(gSelectedId);
      if (!selected) {
        ImGui::TextDisabled("Select an object in the tree.");
      } else {
        ImGui::Text("%s #%u", selected->name.c_str(), selected->id);
        ImGui::Separator();
        float position[3]{selected->transform.position.x, selected->transform.position.y, selected->transform.position.z};
        float rotation[3]{selected->transform.rotation.x, selected->transform.rotation.y, selected->transform.rotation.z};
        float objectScale[3]{selected->transform.scale.x, selected->transform.scale.y, selected->transform.scale.z};
        bool changed = ImGui::DragFloat3("Position", position, 0.05f);
        changed |= ImGui::DragFloat3("Rotation", rotation, 1.0f);
        changed |= ImGui::DragFloat3("Scale", objectScale, 0.05f, 0.10f, 10.0f);
        changed |= ImGui::Checkbox("Dynamic body", &selected->dynamic);
        if (changed) {
          selected->transform.position={position[0],position[1],position[2]};
          selected->transform.rotation={rotation[0],rotation[1],rotation[2]};
          selected->transform.scale={objectScale[0],objectScale[1],objectScale[2]};
          gPhysicsRebuildRequested = true;
        }
        ImGui::Separator();
        if (ImGui::Button("Delete selected")) gDeleteRequested = true;
      }
      ImGui::End();

      ImGui::SetNextWindowPos(ImVec2(0, float(extent.height) - 42.0f), ImGuiCond_Always);
      ImGui::SetNextWindowSize(ImVec2(float(extent.width), 42), ImGuiCond_Always);
      ImGui::Begin("Taskbar", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);
      ImGui::Text("CAO Jolt Vulkan  |  %d objects  |  Orbit: left drag  Pan: middle drag  Zoom: wheel  |  %s",
                  static_cast<int>(scene.objects().size()), gSimulationRunning ? "SIMULATION RUNNING" : "BUILD MODE");
      ImGui::End();
      ImGui::Render();
      if (gCreatePrimitive >= 0) {
        Transform transform; transform.position={0.0f, 2.0f, 0.0f};
        SceneObject &object = scene.add(static_cast<Primitive>(gCreatePrimitive), transform);
        gSelectedId = object.id;
        addBlock({{0.80f, 0.42f, 0.95f}}, object.id);
        uploadSceneGeometry(); physics.rebuild(scene); gCreatePrimitive = -1;
      }
      if (gDeleteRequested) {
        scene.erase(gSelectedId); gSelectedId = scene.objects().empty() ? 0 : scene.objects().front().id;
        physics.rebuild(scene); gDeleteRequested = false;
      }
      if (gPhysicsRebuildRequested) {
        physics.rebuild(scene); gPhysicsRebuildRequested = false;
      }
      if (gBuildRequested) {
        scene.buildQuadruped(); physics.rebuild(scene); gSelectedId = scene.objects().empty() ? 0 : scene.objects().front().id; gSimulationRunning = false; gBuildRequested = false;
      }
      if (gDemoRequested) {
        scene.buildQuadruped(); physics.rebuild(scene); physics.demolish(scene); gSimulationRunning = true; gDemoRequested = false;
      }
      if (gSimulationRunning && !simulationWasRunning) {
        frameRecorder.start(gRobotScript);
        frameRecordingSaved = false;
        frameRecordingWriteAttempted = false;
      }
      if (gSimulationRunning) {
        constexpr float simulationDeltaSeconds = 1.0f / 60.0f;
        physics.step(scene, simulationDeltaSeconds);
        frameRecorder.capture(scene, simulationDeltaSeconds, physics.telemetry());
      }
      if (frameRecorder.complete() && !frameRecordingWriteAttempted) {
        frameRecordingSaved = frameRecorder.write("robot_recording.json");
        frameRecordingWriteAttempted = true;
      }
      simulationWasRunning = gSimulationRunning;
      updateCamera();
      const char *mode = gSimulationRunning ? "Simulation running" : "Simulation paused";
      glfwSetWindowTitle(window, (std::string("CAO Jolt Vulkan | ") + mode + " | B: reset robot | D: drop robot | Space: play/pause").c_str());
      check(vkWaitForFences(device,1,&fence,VK_TRUE,UINT64_MAX),"wait fence"); check(vkResetFences(device,1,&fence),"reset fence");
      uint32_t image=0; VkResult acquire=vkAcquireNextImageKHR(device,swapchain,UINT64_MAX,available,VK_NULL_HANDLE,&image);
      if (acquire==VK_ERROR_OUT_OF_DATE_KHR) continue; check(acquire,"acquire image"); check(vkResetCommandBuffer(commands[image],0),"reset command");
      VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO}; check(vkBeginCommandBuffer(commands[image],&begin),"begin command");
      std::array<VkClearValue, 2> clear{}; clear[0].color={{0.025f,0.05f,0.11f,1}}; clear[1].depthStencil={1.0f,0};
      VkRenderPassBeginInfo render{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO}; render.renderPass=renderPass; render.framebuffer=framebuffers[image]; render.renderArea.extent=extent; render.clearValueCount=static_cast<uint32_t>(clear.size()); render.pClearValues=clear.data();
      vkCmdBeginRenderPass(commands[image],&render,VK_SUBPASS_CONTENTS_INLINE);
      vkCmdBindPipeline(commands[image],VK_PIPELINE_BIND_POINT_GRAPHICS,pipeline); VkDeviceSize offset=0;
      vkCmdBindVertexBuffers(commands[image],0,1,&vertexBuffer,&offset); vkCmdBindIndexBuffer(commands[image],indexBuffer,0,VK_INDEX_TYPE_UINT32);
      if (gTintMode == 0) { drawPush.tint[0]=drawPush.tint[1]=drawPush.tint[2]=1.0f; }
      if (gTintMode == 1) { drawPush.tint[0]=1.0f; drawPush.tint[1]=0.82f; drawPush.tint[2]=0.28f; }
      if (gTintMode == 2) { drawPush.tint[0]=0.45f; drawPush.tint[1]=0.92f; drawPush.tint[2]=1.0f; }
      setMvp(identity());
      vkCmdPushConstants(commands[image],layout,VK_SHADER_STAGE_VERTEX_BIT,0,sizeof(Push),&drawPush);
      vkCmdDrawIndexed(commands[image], gridIndexCount, 1, 0, 0, 0);
      for (const DrawItem &draw : objectDraws) {
        const SceneObject *object = scene.find(draw.objectId);
        if (!object) continue;
        const Transform &t = object->transform;
        setMvp(multiply(translate(t.position.x, t.position.y, t.position.z), multiply(rotateZ(t.rotation.z), multiply(rotateY(t.rotation.y), multiply(rotateX(t.rotation.x), scale(t.scale.x, t.scale.y, t.scale.z))))));
        vkCmdPushConstants(commands[image],layout,VK_SHADER_STAGE_VERTEX_BIT,0,sizeof(Push),&drawPush);
        vkCmdDrawIndexed(commands[image], draw.indexCount, 1, draw.firstIndex, 0, 0);
      }
      ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), commands[image]);
      vkCmdEndRenderPass(commands[image]); check(vkEndCommandBuffer(commands[image]),"end command");
      VkPipelineStageFlags wait=VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT; VkSubmitInfo submit{VK_STRUCTURE_TYPE_SUBMIT_INFO};
      submit.waitSemaphoreCount=1; submit.pWaitSemaphores=&available; submit.pWaitDstStageMask=&wait; submit.commandBufferCount=1; submit.pCommandBuffers=&commands[image]; submit.signalSemaphoreCount=1; submit.pSignalSemaphores=&finished;
      check(vkQueueSubmit(queue,1,&submit,fence),"submit"); VkPresentInfoKHR present{VK_STRUCTURE_TYPE_PRESENT_INFO_KHR};
      present.waitSemaphoreCount=1; present.pWaitSemaphores=&finished; present.swapchainCount=1; present.pSwapchains=&swapchain; present.pImageIndices=&image; vkQueuePresentKHR(queue,&present);
    }
    vkDeviceWaitIdle(device);
    gUiReady = false; ImGui_ImplVulkan_Shutdown(); ImGui_ImplGlfw_Shutdown(); ImGui::DestroyContext(); vkDestroyDescriptorPool(device,imguiPool,nullptr);
    vkDestroyFence(device,fence,nullptr); vkDestroySemaphore(device,finished,nullptr); vkDestroySemaphore(device,available,nullptr);
    vkDestroyCommandPool(device,pool,nullptr); vkDestroyBuffer(device,indexBuffer,nullptr); vkFreeMemory(device,indexMemory,nullptr); vkDestroyBuffer(device,vertexBuffer,nullptr); vkFreeMemory(device,vertexMemory,nullptr);
    vkDestroyPipeline(device,pipeline,nullptr); vkDestroyPipelineLayout(device,layout,nullptr); for(auto fb:framebuffers)vkDestroyFramebuffer(device,fb,nullptr); vkDestroyRenderPass(device,renderPass,nullptr); vkDestroyImageView(device,depthView,nullptr); vkDestroyImage(device,depthImage,nullptr); vkFreeMemory(device,depthMemory,nullptr); for(auto view:views)vkDestroyImageView(device,view,nullptr);
    vkDestroySwapchainKHR(device,swapchain,nullptr); physics.shutdown(); vkDestroyDevice(device,nullptr); vkDestroySurfaceKHR(instance,surface,nullptr); vkDestroyInstance(instance,nullptr); glfwDestroyWindow(window); glfwTerminate();
    return 0;
  } catch (const std::exception &error) { std::cerr << "Fatal: " << error.what() << "\n"; return 4; }
}