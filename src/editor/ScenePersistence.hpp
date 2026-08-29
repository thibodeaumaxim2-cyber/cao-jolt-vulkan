#pragma once

#include "SceneDocument.hpp"
#include <fstream>
#include <stdexcept>

inline void writeSceneName(const SceneDocument& scene,
                           const std::string& path) {
  std::ofstream out(path);
  if (!out) throw std::runtime_error("Cannot write CAO scene file");
  out << "{\n  \"version\": " << scene.version
      << ",\n  \"name\": \"" << scene.name
      << "\",\n  \"objectCount\": " << scene.objects.size()
      << "\n}\n";
}
