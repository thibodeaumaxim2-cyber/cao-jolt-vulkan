#pragma once

#include "StructuralPrimitive.hpp"
#include <string>
#include <vector>

struct SceneDocument {
  std::string name = "Untitled CAO Scene";
  std::vector<StructuralPrimitive> objects;
  unsigned int version = 1;
};
