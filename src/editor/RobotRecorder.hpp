#pragma once
#include "Scene.hpp"
#include <nlohmann/json.hpp>
#include <filesystem>

class RobotFrameRecorder {
 public:
  explicit RobotFrameRecorder(float durationSeconds = 5.0f);

  void start(int motionScript);
  void capture(const Scene &scene, float deltaSeconds);
  bool recording() const { return recording_; }
  bool complete() const { return complete_; }
  float elapsedSeconds() const { return elapsedSeconds_; }
  float durationSeconds() const { return durationSeconds_; }
  bool write(const std::filesystem::path &path) const;

 private:
  float durationSeconds_;
  float elapsedSeconds_ = 0.0f;
  int motionScript_ = 0;
  bool recording_ = false;
  bool complete_ = false;
  nlohmann::json frames_ = nlohmann::json::array();
};
