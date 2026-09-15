#pragma once
#include <algorithm>
#include <cmath>

// Shared hexapod dimensions in metres. Keep renderer, Jolt anchors,
// telemetry, and the future IK solver on the same physical model.
namespace CaoLegGeometry {
inline constexpr float hipOffsetX = 0.64f;
// Front/rear separation gives the robot a dog-like rectangular support base.
inline constexpr float hipOffsetZ = 0.62f;
inline constexpr float torsoHipHeight = 0.925f;
inline constexpr float kneeHeight = 0.600f;
// Compact cat-paw contact pad: the long lower leg reaches a very short sole.
inline constexpr float ankleHeight = 0.09f;
inline constexpr float femurLength = 0.325f;
inline constexpr float tibiaLength = 0.51f;
inline constexpr float footLength = 0.21f;
inline constexpr float footWidth = 0.17f;
inline constexpr float supportKneeAngle = 0.0f;
// DOF safety envelope. Zero is the assembled, load-bearing neutral pose.
// The negative pitch stops prevent a falling torso from folding every leg
// underneath itself before the equilibrium gate can stop the gait.
inline constexpr float hipRollMinAngle = -0.20f;
inline constexpr float hipRollMaxAngle = 0.20f;
inline constexpr float hipPitchMinAngle = -0.40f;
inline constexpr float hipPitchMaxAngle = 0.40f;
inline constexpr float kneePitchMinAngle = -0.70f;
inline constexpr float kneePitchMaxAngle = 0.20f;
inline constexpr float anklePitchMinAngle = -0.30f;
inline constexpr float anklePitchMaxAngle = 0.30f;
inline constexpr float hipRollTorqueNm = 260.0f;
inline constexpr float hipPitchTorqueNm = 620.0f;
inline constexpr float kneePitchTorqueNm = 500.0f;
inline constexpr float anklePitchTorqueNm = 300.0f;
inline constexpr float hipPitchMotorFrequencyHz = 3.5f;
inline constexpr float hipPitchMotorDamping = 2.4f;
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
