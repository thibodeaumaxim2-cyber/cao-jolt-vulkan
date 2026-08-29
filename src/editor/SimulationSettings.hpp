#pragma once

struct SimulationSettings {
  float gravity = -9.81f;
  float fixedTimeStep = 1.0f / 60.0f;
  bool paused = true;
  bool allowSleep = true;
  bool showForces = false;
  bool showStructuralHealth = true;
};
