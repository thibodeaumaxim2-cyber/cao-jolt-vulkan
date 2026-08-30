#pragma once
#include <algorithm>
#include <cmath>

// Shared quadruped dimensions in metres. Keep renderer, Jolt anchors,
// telemetry, and the future IK solver on the same physical model.
namespace CaoLegGeometry {
inline constexpr float hipOffsetX = 0.64f;
inline constexpr float hipOffsetZ = 0.30f;
inline constexpr float torsoHipHeight = 2.11f;
inline constexpr float kneeHeight = 1.41f;
inline constexpr float ankleHeight = 0.70f;
inline constexpr float femurLength = 0.70f;
inline constexpr float tibiaLength = 0.71f;
inline constexpr float footLength = 0.30f;
inline constexpr float supportKneeAngle = 0.0f;
inline constexpr float hipPitchTorqueNm = 120.0f;
inline constexpr float hipPitchMotorFrequencyHz = 8.0f;
inline constexpr float hipPitchMotorDamping = 1.5f;
inline constexpr float swingKneeAngle = -1.5708f;

struct PlanarIK {
  float hip = 0.0f;
  float knee = supportKneeAngle;
};

// Solve a two-link leg in the vertical plane. Angles use the same
// convention as the Jolt hinge targets: a straight leg is knee = 0.
inline PlanarIK solvePlanar(float verticalDrop, float forwardReach) {
  const float a = femurLength;
  const float b = tibiaLength;
  const float distance = std::clamp(std::hypot(verticalDrop, forwardReach),
                                    0.05f, a + b - 0.001f);
  const float hip = std::atan2(forwardReach, verticalDrop)
      - std::acos(std::clamp((a*a + distance*distance - b*b) /
                             (2.0f*a*distance), -1.0f, 1.0f));
  // Convert the triangle's internal angle to the hinge convention:
  // collinear/extended links = 0 rad, folded 90 degrees = -pi/2.
  const float internalAngle = std::acos(std::clamp((a*a + b*b - distance*distance) /
                                                   (2.0f*a*b), -1.0f, 1.0f));
  const float knee = -(3.14159265358979323846f - internalAngle);
  return {hip, knee};
}
}
