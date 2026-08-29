#pragma once

#include <algorithm>
#include <string>
#include <vector>

inline bool validationLayerAvailable(
    const std::vector<std::string>& available,
    const char* requested) {
  return std::find(available.begin(), available.end(), requested) !=
         available.end();
}
