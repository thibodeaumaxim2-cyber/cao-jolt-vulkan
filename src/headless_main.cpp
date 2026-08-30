#include "editor/JoltBridge.hpp"
#include "editor/Scene.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>

int main() {
  Scene scene;
  scene.buildQuadruped();

  JoltBridge physics;
  physics.initialize();
  physics.rebuild(scene);
  physics.setRobotScript(0); // standing controller

  constexpr float dt = 1.0f / 240.0f;
  constexpr int steps = 240 * 8;
  const SceneObject *torso = nullptr;
  for (const auto &object : scene.objects())
    if (object.name == "Torso") { torso = &object; break; }

  if (!torso) {
    std::cerr << "headless: torso not found\n";
    return 2;
  }

  const Vec3 initial = torso->transform.position;
  float maxSpeed = 0.0f, maxDisplacement = 0.0f;
  float minHeight = std::numeric_limits<float>::max();
  float maxHeight = std::numeric_limits<float>::lowest();
  float maxJointError = 0.0f;
  int saturatedSamples = 0;

  for (int i = 0; i < steps; ++i) {
    physics.step(scene, dt);
    const Vec3 p = torso->transform.position;
    const float displacement = std::sqrt(
        (p.x - initial.x) * (p.x - initial.x) +
        (p.z - initial.z) * (p.z - initial.z));
    maxDisplacement = std::max(maxDisplacement, displacement);
    minHeight = std::min(minHeight, p.y);
    maxHeight = std::max(maxHeight, p.y);

    const RobotTelemetry &telemetry = physics.telemetry();
    maxSpeed = std::max(maxSpeed, telemetry.torsoSpeedMps);
    for (const auto &leg : telemetry.angleErrorRad)
      for (float error : leg) maxJointError = std::max(maxJointError, std::abs(error));
    saturatedSamples += static_cast<int>(std::count(
        telemetry.torqueSaturated.begin(), telemetry.torqueSaturated.end(), true));
  }

  nlohmann::json result = {
      {"duration_seconds", steps * dt},
      {"fixed_rate_hz", 240},
      {"controller", "stand"},
      {"stable", maxSpeed < 0.75f && maxDisplacement < 0.20f && minHeight > 1.50f},
      {"metrics", {
          {"initial_torso_height_m", initial.y},
          {"min_torso_height_m", minHeight},
          {"max_torso_height_m", maxHeight},
          {"max_horizontal_displacement_m", maxDisplacement},
          {"max_torso_speed_mps", maxSpeed},
          {"max_joint_error_rad", maxJointError},
          {"torque_saturated_samples", saturatedSamples}
      }}
  };

  std::ofstream output("headless_standing_result.json");
  output << result.dump(2) << '\n';
  std::cout << result.dump(2) << '\n';
  physics.shutdown();
  return result["stable"].get<bool>() ? 0 : 1;
}
