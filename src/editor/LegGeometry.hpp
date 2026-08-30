#pragma once

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
inline constexpr float swingKneeAngle = -1.5708f;
}
