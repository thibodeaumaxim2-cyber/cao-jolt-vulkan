#pragma once

#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <vector>
#include <vulkan/vulkan.h>

inline std::vector<char> readSpirvFile(const char* path) {
  std::ifstream file(path, std::ios::ate | std::ios::binary);
  if (!file)
    throw std::runtime_error("Cannot open SPIR-V shader file");
  const std::streamsize size = file.tellg();
  if (size <= 0 || size % 4 != 0)
    throw std::runtime_error("Invalid SPIR-V shader size");
  std::vector<char> bytes(static_cast<size_t>(size));
  file.seekg(0);
  file.read(bytes.data(), size);
  return bytes;
}

inline VkShaderModule createShaderModule(VkDevice device,
                                         const std::vector<char>& code) {
  VkShaderModuleCreateInfo info{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
  info.codeSize = code.size();
  info.pCode = reinterpret_cast<const uint32_t*>(code.data());
  VkShaderModule module = VK_NULL_HANDLE;
  if (vkCreateShaderModule(device, &info, nullptr, &module) != VK_SUCCESS)
    throw std::runtime_error("Cannot create Vulkan shader module");
  return module;
}
