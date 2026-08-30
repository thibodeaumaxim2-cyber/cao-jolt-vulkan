#pragma once
#include "Scene.hpp"
#include "JoltBridge.hpp"
#include <nlohmann/json.hpp>
#include <filesystem>
#include <string>

class RobotFrameRecorder {
 public:
  explicit RobotFrameRecorder(float durationSeconds = 5.0f);

  void start(int motionScript);
  void capture(const Scene &scene, float deltaSeconds, const RobotTelemetry &telemetry);
  bool recording() const { return recording_; }
  bool complete() const { return complete_; }
  float elapsedSeconds() const { return elapsedSeconds_; }
  float durationSeconds() const { return durationSeconds_; }
  bool write(const std::filesystem::path &path) const;
  const std::string& runId() const { return runId_; }

 private:
  float durationSeconds_;
  float elapsedSeconds_ = 0.0f;
  int motionScript_ = 0;
  bool recording_ = false;
  bool complete_ = false;
  nlohmann::json frames_ = nlohmann::json::array();
  std::string runId_;
};
