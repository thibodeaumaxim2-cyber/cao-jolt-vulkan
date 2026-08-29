#pragma once

struct PhysicsWorldSettings {
  int maxBodies = 2048;
  int collisionSteps = 1;
  int integrationSubSteps = 1;
  float gravityY = -9.81f;
};
