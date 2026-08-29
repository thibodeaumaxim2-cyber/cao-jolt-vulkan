#define GLFW_INCLUDE_NONE
#include <vulkan/vulkan.h>
#include <GLFW/glfw3.h>

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

static float gYaw = 0.0f, gZoom = 1.0f, gPanX = 0.0f, gPanY = 0.0f;
static bool gDragging = false, gPanning = false;
static double gLastX = 0.0, gLastY = 0.0;
static void cursor(GLFWwindow *, double x, double y) {
  if (gDragging) gYaw += static_cast<float>(x - gLastX) * 0.008f;
  if (gPanning) { gPanX += static_cast<float>(x - gLastX) * 0.002f; gPanY -= static_cast<float>(y - gLastY) * 0.002f; }
  gLastX = x; gLastY = y;
}
static void mouseButton(GLFWwindow *window, int button, int action, int) {
  if (button == GLFW_MOUSE_BUTTON_LEFT)
    gDragging = action == GLFW_PRESS;
  if (button == GLFW_MOUSE_BUTTON_MIDDLE)
    gPanning = action == GLFW_PRESS;
  glfwGetCursorPos(window, &gLastX, &gLastY);
}
static void key(GLFWwindow *, int key, int, int action, int) {
  if (key == GLFW_KEY_R && action == GLFW_PRESS) {
    gYaw = 0.0f; gZoom = 1.0f; gPanX = 0.0f; gPanY = 0.0f;
  }
}

static void scroll(GLFWwindow *, double, double y) {
  gZoom = std::clamp(gZoom * (1.0f + static_cast<float>(y) * 0.10f), 0.35f, 2.5f);
}

static void check(VkResult result, const char *what) {
  if (result != VK_SUCCESS)
    throw std::runtime_error(std::string(what) + " (" + std::to_string(result) + ")");
}

struct Vertex { float position[3]; float color[3]; };
struct Push { float mvp[16]; float tint[4]; };

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
    GLFWwindow *window = glfwCreateWindow(1280, 800, "CAO Jolt Vulkan - Triangle", nullptr, nullptr);
    if (!window) throw std::runtime_error("Cannot create window");
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

    // First native CAO scene: a 3-2-1 cube pyramid centred in the work area.
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    const std::array<std::array<float, 3>, 3> layerColors{{
        {{0.10f, 0.72f, 0.95f}}, {{0.18f, 0.88f, 0.62f}}, {{1.00f, 0.72f, 0.16f}}
    }};
    auto addBlock = [&](float x, float y, float size, const std::array<float, 3> &color) {
      // Eight corners and six faces: this is real cube geometry, not a screen tile.
      const uint32_t first = static_cast<uint32_t>(vertices.size());
      const float z0 = -0.10f, z1 = z0 + size;
      const std::array<Vertex, 8> cube{{
          {{x,        y,        z0}, {color[0] * 0.55f, color[1] * 0.55f, color[2] * 0.55f}},
          {{x + size, y,        z0}, {color[0] * 0.55f, color[1] * 0.55f, color[2] * 0.55f}},
          {{x + size, y + size, z0}, {color[0] * 0.55f, color[1] * 0.55f, color[2] * 0.55f}},
          {{x,        y + size, z0}, {color[0] * 0.55f, color[1] * 0.55f, color[2] * 0.55f}},
          {{x,        y,        z1}, {color[0] * 0.78f, color[1] * 0.78f, color[2] * 0.78f}},
          {{x + size, y,        z1}, {color[0] * 0.78f, color[1] * 0.78f, color[2] * 0.78f}},
          {{x + size, y + size, z1}, {color[0], color[1], color[2]}},
          {{x,        y + size, z1}, {color[0], color[1], color[2]}}
      }};
      vertices.insert(vertices.end(), cube.begin(), cube.end());
      indices.insert(indices.end(), {
          first, first+1, first+2, first+2, first+3, first,
          first+4, first+6, first+5, first+6, first+4, first+7,
          first, first+4, first+5, first+5, first+1, first,
          first+1, first+5, first+6, first+6, first+2, first+1,
          first+2, first+6, first+7, first+7, first+3, first+2,
          first+3, first+7, first+4, first+4, first, first+3
      });
    };
    constexpr float block = 0.22f;
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 3; ++col)
        addBlock(-0.33f + col * block, -0.62f + row * block, block * 0.92f, layerColors[0]);
    for (int row = 0; row < 2; ++row)
      for (int col = 0; col < 2; ++col)
        addBlock(-0.22f + col * block, 0.04f + row * block, block * 0.92f, layerColors[1]);
    addBlock(-0.11f, 0.48f, block * 0.92f, layerColors[2]);
    auto buffer = [&](VkDeviceSize size, VkBufferUsageFlags usage, const void *source, VkBuffer &outBuffer, VkDeviceMemory &outMemory) {
      VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO}; info.size=size; info.usage=usage; info.sharingMode=VK_SHARING_MODE_EXCLUSIVE;
      check(vkCreateBuffer(device,&info,nullptr,&outBuffer),"buffer"); VkMemoryRequirements requirements{}; vkGetBufferMemoryRequirements(device,outBuffer,&requirements);
      VkMemoryAllocateInfo allocation{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO}; allocation.allocationSize=requirements.size;
      allocation.memoryTypeIndex=memoryType(gpu,requirements.memoryTypeBits,VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
      check(vkAllocateMemory(device,&allocation,nullptr,&outMemory),"buffer memory"); check(vkBindBufferMemory(device,outBuffer,outMemory,0),"bind buffer");
      void *mapped=nullptr; check(vkMapMemory(device,outMemory,0,size,0,&mapped),"map buffer"); std::memcpy(mapped,source,size); vkUnmapMemory(device,outMemory);
    };
    VkBuffer vertexBuffer=VK_NULL_HANDLE,indexBuffer=VK_NULL_HANDLE; VkDeviceMemory vertexMemory=VK_NULL_HANDLE,indexMemory=VK_NULL_HANDLE;
    buffer(vertices.size() * sizeof(Vertex),VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,vertices.data(),vertexBuffer,vertexMemory);
    buffer(indices.size() * sizeof(uint32_t),VK_BUFFER_USAGE_INDEX_BUFFER_BIT,indices.data(),indexBuffer,indexMemory);

    VkCommandPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO}; poolInfo.flags=VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT; poolInfo.queueFamilyIndex=family;
    VkCommandPool pool=VK_NULL_HANDLE; check(vkCreateCommandPool(device,&poolInfo,nullptr,&pool),"command pool");
    std::vector<VkCommandBuffer> commands(swapImageCount); VkCommandBufferAllocateInfo commandInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    commandInfo.commandPool=pool; commandInfo.level=VK_COMMAND_BUFFER_LEVEL_PRIMARY; commandInfo.commandBufferCount=swapImageCount;
    check(vkAllocateCommandBuffers(device,&commandInfo,commands.data()),"command buffers");
    VkSemaphoreCreateInfo semInfo{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO}; VkSemaphore available,finished;
    check(vkCreateSemaphore(device,&semInfo,nullptr,&available),"available semaphore"); check(vkCreateSemaphore(device,&semInfo,nullptr,&finished),"finished semaphore");
    VkFenceCreateInfo fenceInfo{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO}; fenceInfo.flags=VK_FENCE_CREATE_SIGNALED_BIT; VkFence fence;
    check(vkCreateFence(device,&fenceInfo,nullptr,&fence),"fence");

    Push drawPush{}; drawPush.tint[0]=drawPush.tint[1]=drawPush.tint[2]=drawPush.tint[3]=1;
    auto updateCamera = [&] {
      const float c = std::cos(gYaw) * gZoom, sn = std::sin(gYaw) * gZoom;
      // Isometric CAO projection with a user-controlled turn around the vertical axis.
      std::fill(std::begin(drawPush.mvp), std::end(drawPush.mvp), 0.0f);
      drawPush.mvp[0] = 0.80f * c;  drawPush.mvp[1] = 0.38f * c;
      drawPush.mvp[4] = -0.80f * sn; drawPush.mvp[5] = 0.38f * sn;
      drawPush.mvp[8] = 0.65f; drawPush.mvp[9] = -0.40f;
      drawPush.mvp[10] = 0.5f; drawPush.mvp[12] = gPanX; drawPush.mvp[13] = gPanY; drawPush.mvp[15] = 1.0f;
    };
    while (!glfwWindowShouldClose(window)) {
      glfwPollEvents(); updateCamera(); check(vkWaitForFences(device,1,&fence,VK_TRUE,UINT64_MAX),"wait fence"); check(vkResetFences(device,1,&fence),"reset fence");
      uint32_t image=0; VkResult acquire=vkAcquireNextImageKHR(device,swapchain,UINT64_MAX,available,VK_NULL_HANDLE,&image);
      if (acquire==VK_ERROR_OUT_OF_DATE_KHR) continue; check(acquire,"acquire image"); check(vkResetCommandBuffer(commands[image],0),"reset command");
      VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO}; check(vkBeginCommandBuffer(commands[image],&begin),"begin command");
      std::array<VkClearValue, 2> clear{}; clear[0].color={{0.025f,0.05f,0.11f,1}}; clear[1].depthStencil={1.0f,0};
      VkRenderPassBeginInfo render{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO}; render.renderPass=renderPass; render.framebuffer=framebuffers[image]; render.renderArea.extent=extent; render.clearValueCount=static_cast<uint32_t>(clear.size()); render.pClearValues=clear.data();
      vkCmdBeginRenderPass(commands[image],&render,VK_SUBPASS_CONTENTS_INLINE);
      vkCmdBindPipeline(commands[image],VK_PIPELINE_BIND_POINT_GRAPHICS,pipeline); VkDeviceSize offset=0;
      vkCmdBindVertexBuffers(commands[image],0,1,&vertexBuffer,&offset); vkCmdBindIndexBuffer(commands[image],indexBuffer,0,VK_INDEX_TYPE_UINT32);
      vkCmdPushConstants(commands[image],layout,VK_SHADER_STAGE_VERTEX_BIT,0,sizeof(Push),&drawPush); vkCmdDrawIndexed(commands[image],static_cast<uint32_t>(indices.size()),1,0,0,0);
      vkCmdEndRenderPass(commands[image]); check(vkEndCommandBuffer(commands[image]),"end command");
      VkPipelineStageFlags wait=VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT; VkSubmitInfo submit{VK_STRUCTURE_TYPE_SUBMIT_INFO};
      submit.waitSemaphoreCount=1; submit.pWaitSemaphores=&available; submit.pWaitDstStageMask=&wait; submit.commandBufferCount=1; submit.pCommandBuffers=&commands[image]; submit.signalSemaphoreCount=1; submit.pSignalSemaphores=&finished;
      check(vkQueueSubmit(queue,1,&submit,fence),"submit"); VkPresentInfoKHR present{VK_STRUCTURE_TYPE_PRESENT_INFO_KHR};
      present.waitSemaphoreCount=1; present.pWaitSemaphores=&finished; present.swapchainCount=1; present.pSwapchains=&swapchain; present.pImageIndices=&image; vkQueuePresentKHR(queue,&present);
    }
    vkDeviceWaitIdle(device);
    vkDestroyFence(device,fence,nullptr); vkDestroySemaphore(device,finished,nullptr); vkDestroySemaphore(device,available,nullptr);
    vkDestroyCommandPool(device,pool,nullptr); vkDestroyBuffer(device,indexBuffer,nullptr); vkFreeMemory(device,indexMemory,nullptr); vkDestroyBuffer(device,vertexBuffer,nullptr); vkFreeMemory(device,vertexMemory,nullptr);
    vkDestroyPipeline(device,pipeline,nullptr); vkDestroyPipelineLayout(device,layout,nullptr); for(auto fb:framebuffers)vkDestroyFramebuffer(device,fb,nullptr); vkDestroyRenderPass(device,renderPass,nullptr); vkDestroyImageView(device,depthView,nullptr); vkDestroyImage(device,depthImage,nullptr); vkFreeMemory(device,depthMemory,nullptr); for(auto view:views)vkDestroyImageView(device,view,nullptr);
    vkDestroySwapchainKHR(device,swapchain,nullptr); vkDestroyDevice(device,nullptr); vkDestroySurfaceKHR(instance,surface,nullptr); vkDestroyInstance(instance,nullptr); glfwDestroyWindow(window); glfwTerminate();
    return 0;
  } catch (const std::exception &error) { std::cerr << "Fatal: " << error.what() << "\n"; return 4; }
}
