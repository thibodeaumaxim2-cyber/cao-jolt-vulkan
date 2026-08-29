#pragma once

#include <cstdint>

struct BeamConstraint {
  uint32_t beamObjectId = 0;
  float length = 1.0f;
  float crossSectionArea = 0.0025f;
  float secondMoment = 0.000001f;
  float effectiveLengthFactor = 1.0f;
};
