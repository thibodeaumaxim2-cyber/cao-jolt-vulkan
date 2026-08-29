#pragma once

#include <cstddef>
#include <cstdint>
#include <vulkan/vulkan.h>

struct CadVertex {
  float position[3];
  float color[3];

  static VkVertexInputBindingDescription binding() {
    return VkVertexInputBindingDescription{0, sizeof(CadVertex),
                                            VK_VERTEX_INPUT_RATE_VERTEX};
  }

  static VkVertexInputAttributeDescription attributes[2]() = delete;
};
