#pragma once

struct PhysicsStepState {
  float accumulatedSeconds = 0.0f;
  float fixedTimeStep = 1.0f / 60.0f;
  unsigned int stepsLastFrame = 0;
};
