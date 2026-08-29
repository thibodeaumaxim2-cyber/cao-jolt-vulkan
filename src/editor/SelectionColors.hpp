#pragma once

#include <cstdint>

struct SelectionColors {
  float normal[4] = {0.20f, 0.70f, 0.95f, 1.0f};
  float selected[4] = {1.00f, 0.75f, 0.15f, 1.0f};
  float hovered[4] = {0.35f, 0.95f, 1.00f, 1.0f};
};

inline const float* colorForSelection(const SelectionColors& colors,
                                      bool selected,
                                      bool hovered) {
  if (selected)
    return colors.selected;
  if (hovered)
    return colors.hovered;
  return colors.normal;
}
