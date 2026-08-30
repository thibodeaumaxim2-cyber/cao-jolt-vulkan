#include "RobotRecorder.hpp"

#include <algorithm>
#include <fstream>

RobotFrameRecorder::RobotFrameRecorder(float durationSeconds)
    : durationSeconds_(std::max(0.1f, durationSeconds)) {}

void RobotFrameRecorder::start(int motionScript) {
  motionScript_ = motionScript;
  elapsedSeconds_ = 0.0f;
  recording_ = true;
  complete_ = false;
  frames_ = nlohmann::json::array();
}

void RobotFrameRecorder::capture(const Scene &scene, float deltaSeconds, const RobotTelemetry &telemetry) {
  if (!recording_) return;

  nlohmann::json links = nlohmann::json::array();
  for (const SceneObject &object : scene.objects()) {
    links.push_back({
        {"id", object.id},
        {"name", object.name},
        {"dynamic", object.dynamic},
        {"position_m", {object.transform.position.x, object.transform.position.y, object.transform.position.z}},
        {"rotation_rad", {object.transform.rotation.x, object.transform.rotation.y, object.transform.rotation.z}},
        {"scale_m", {object.transform.scale.x, object.transform.scale.y, object.transform.scale.z}}
    });
  }
  frames_.push_back({
      {"time_s", std::min(elapsedSeconds_, durationSeconds_)},
      {"links", std::move(links)},
      {"telemetry", {
          {"motion_script", telemetry.motionScript},
          {"active_swing_leg", telemetry.activeSwingLeg},
          {"gait_cycle", telemetry.gaitCycle},
          {"torso_speed_mps", telemetry.torsoSpeedMps},
          {"target_angles_rad", telemetry.targetAnglesRad},
          {"torque_limits_Nm", telemetry.torqueLimitsNm},
          {"lift_assist_force_N", telemetry.liftAssistForceN},
          {"foot_friction", telemetry.footFriction},
          {"swing_lift_force_N", telemetry.swingLiftForceN},
          {"leg_state", telemetry.legState},
          {"target_angles_rad", telemetry.targetAnglesRad},
          {"lift_assist_force_N", telemetry.liftAssistForceN},
          {"foot_friction", telemetry.footFriction},
          {"torque_limits_Nm", telemetry.torqueLimitsNm}
      }}
  });

  elapsedSeconds_ += std::max(0.0f, deltaSeconds);
  if (elapsedSeconds_ >= durationSeconds_) {
    elapsedSeconds_ = durationSeconds_;
    recording_ = false;
    complete_ = true;
  }
}

bool RobotFrameRecorder::write(const std::filesystem::path &path) const {
  if (!complete_ || frames_.empty()) return false;
  nlohmann::json document = {
      {"format", "cao-jolt-robot-frame-recording"},
      {"format_version", 1},
      {"units", {{"position", "m"}, {"rotation", "rad"}, {"time", "s"}}},
      {"capture", {
          {"duration_seconds", durationSeconds_},
          {"sample_rate_hz", 60},
          {"motion_script", motionScript_},
          {"frame_count", frames_.size()}
      }},
      {"frames", frames_}
  };
  std::ofstream output(path);
  if (!output) return false;
  output << document.dump(2) << '\n';
  return static_cast<bool>(output);
}
