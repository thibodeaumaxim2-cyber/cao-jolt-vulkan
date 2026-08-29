#pragma once

enum class StructuralHealth { Healthy, Warning, Overloaded, Failed };

struct StructuralMetrics {
  float axialLoad = 0.0f;
  float maxAxialLoad = 1.0f;
  float tension = 0.0f;
  float maxTension = 1.0f;

  StructuralHealth health() const {
    const float ratio = maxAxialLoad > 0.0f ? axialLoad / maxAxialLoad : 0.0f;
    if (ratio >= 1.0f) return StructuralHealth::Failed;
    if (ratio >= 0.85f) return StructuralHealth::Overloaded;
    if (ratio >= 0.65f) return StructuralHealth::Warning;
    return StructuralHealth::Healthy;
  }
};
