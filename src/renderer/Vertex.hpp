#pragma once

#include <array>
#include <cstddef>
#include <vulkan/vulkan.h>

struct CadVertex {
  float position[3];
  float color[3];

  static VkVertexInputBindingDescription binding() {
    return VkVertexInputBindingDescription{
        0, static_cast<uint32_t>(sizeof(CadVertex)),
        VK_VERTEX_INPUT_RATE_VERTEX};
  }

  static std::array<VkVertexInputAttributeDescription, 2> attributes() {
    return {{
        {0, 0, VK_FORMAT_R32G32B32_SFLOAT,
         static_cast<uint32_t>(offsetof(CadVertex, position))},
        {1, 0, VK_FORMAT_R32G32B32_SFLOAT,
         static_cast<uint32_t>(offsetof(CadVertex, color))},
    }};
  }
};
