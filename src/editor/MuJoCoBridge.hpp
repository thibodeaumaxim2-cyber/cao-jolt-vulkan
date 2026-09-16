#pragma once

#include "Scene.hpp"
#include <array>
#include <memory>

inline constexpr size_t kRobotLegCount = 6;
inline constexpr size_t kRobotJointsPerLeg = 4;
inline constexpr size_t kRobotActuatorCount = kRobotLegCount * kRobotJointsPerLeg;

struct StandingTuning {
  float motorFrequencyHz = 4.0f;
  float motorDamping = 2.8f;
  float comGain = 0.18f;
  float velocityGain = 0.04f;
};

struct RobotTelemetry {
  int motionScript = 0;
  int activeSwingLeg = -1;
  int activeSwingTripod = -1;
  int linkCount = 0;
  float gaitCycle = 0.0f;
  float torsoSpeedMps = 0.0f;
  float equilibriumScore = 0.0f;
  bool walkingAllowed = false;
  std::array<bool,2> footContact{};
  std::array<float,2> footNormalForceN{};
  bool learningActive = false;
  int learningProfile = 0;
  int learningSamples = 0;
  float learningReward = 0.0f;
  float swingLiftForceN = 0.0f;
  std::array<int, kRobotLegCount> legState{};
  std::array<std::array<float, kRobotJointsPerLeg>, kRobotLegCount> targetAnglesRad{};
  std::array<float, kRobotLegCount> liftAssistForceN{};
  std::array<std::array<float, kRobotJointsPerLeg>, kRobotLegCount> measuredAnglesRad{};
  std::array<std::array<float, kRobotJointsPerLeg>, kRobotLegCount> angleErrorRad{};
  std::array<float, kRobotActuatorCount> estimatedTorqueDemandNm{};
  std::array<bool, kRobotActuatorCount> torqueSaturated{};
  std::array<float, kRobotLegCount> footFriction{};
  std::array<float, 4> torqueLimitsNm{{260.0f, 620.0f, 500.0f, 300.0f}};
};

// Owns MuJoCo's model and simulation data. Vulkan remains responsible for
// drawing the synchronized Scene transforms.
class MuJoCoBridge {
 public:
  MuJoCoBridge();
  ~MuJoCoBridge();
  MuJoCoBridge(const MuJoCoBridge&) = delete;
  MuJoCoBridge& operator=(const MuJoCoBridge&) = delete;

  void initialize();
  void rebuild(Scene&);
  void step(Scene&, float seconds);
  void demolish(const Scene&);
  void setRobotScript(int script);
  void setStandingTuning(const StandingTuning& tuning);
  int robotScript() const;
  const RobotTelemetry& telemetry() const;
  void shutdown();
  bool initialized() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
