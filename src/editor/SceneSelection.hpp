#pragma once

#include "StructuralPrimitive.hpp"
#include <vector>

inline StructuralPrimitive* findPrimitive(
    std::vector<StructuralPrimitive>& objects, uint32_t id) {
  for (auto& object : objects)
    if (object.id == id) return &object;
  return nullptr;
}

inline void selectPrimitive(std::vector<StructuralPrimitive>& objects,
                            uint32_t id) {
  for (auto& object : objects)
    object.selected = object.id == id;
}
