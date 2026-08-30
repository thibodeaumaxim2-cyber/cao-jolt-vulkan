#pragma once

#include "Scene.hpp"
#include <array>
#include <memory>

struct RobotTelemetry {
  int motionScript = 0;
  int activeSwingLeg = -1;
  int linkCount = 0;
  float gaitCycle = 0.0f;
  float torsoSpeedMps = 0.0f;
  float swingLiftForceN = 0.0f;
  std::array<int, 4> legState{}; // 0 stance, 1 unload, 2 swing, 3 place
  std::array<std::array<float, 4>, 4> targetAnglesRad{};
  std::array<float, 4> liftAssistForceN{};
  std::array<std::array<float, 4>, 4> measuredAnglesRad{};
  std::array<std::array<float, 4>, 4> angleErrorRad{};
  std::array<float, 16> estimatedTorqueDemandNm{};
  std::array<bool, 16> torqueSaturated{};
  std::array<float, 4> footFriction{};
  std::array<float, 4> torqueLimitsNm{{55.0f, 85.0f, 75.0f, 35.0f}};
};

class JoltBridge {
 public:
  JoltBridge();
  ~JoltBridge();
  JoltBridge(const JoltBridge&) = delete;
  JoltBridge& operator=(const JoltBridge&) = delete;

  void initialize();
  void rebuild(Scene&);
  void step(Scene&, float seconds);
  void demolish(const Scene&);
  void setRobotScript(int script);
  int robotScript() const;
  const RobotTelemetry& telemetry() const;
  void shutdown();
  bool initialized() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
