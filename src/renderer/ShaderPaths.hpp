#pragma once

#include <cstdlib>
#include <filesystem>
#include <string>

namespace cao::vk {

inline std::filesystem::path shaderDirectory(
    const std::filesystem::path& executableDirectory = {}) {
  if (const char* configured = std::getenv("CAO_SHADER_DIR")) {
    if (*configured != '\0')
      return std::filesystem::path(configured);
  }

  if (!executableDirectory.empty()) {
    const auto local = executableDirectory / "shaders";
    if (std::filesystem::exists(local))
      return local;

    const auto sibling = executableDirectory / "../share/cao-jolt/shaders";
    if (std::filesystem::exists(sibling))
      return std::filesystem::weakly_canonical(sibling);
  }

  return std::filesystem::path("assets/shaders");
}

inline std::filesystem::path vertexShaderPath(
    const std::filesystem::path& executableDirectory = {}) {
  return shaderDirectory(executableDirectory) / "cad.vert.spv";
}

inline std::filesystem::path fragmentShaderPath(
    const std::filesystem::path& executableDirectory = {}) {
  return shaderDirectory(executableDirectory) / "cad.frag.spv";
}

} // namespace cao::vk
