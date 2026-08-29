#pragma once

#include <cstdint>

struct NativeEditorState {
  uint32_t selectedObject = 0;
  bool simulationRunning = false;
  bool showGrid = true;
  bool showProperties = true;

  void resetViewState() { selectedObject = 0; }
  void toggleSimulation() { simulationRunning = !simulationRunning; }
};
