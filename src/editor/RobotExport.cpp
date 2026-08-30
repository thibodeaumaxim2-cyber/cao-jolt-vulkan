#include "RobotExport.hpp"

#include <nlohmann/json.hpp>
#include <fstream>
#include <algorithm>
#include <string>

using nlohmann::json;

static const char *primitiveName(Primitive p) {
  switch (p) {
    case Primitive::Box: return "box";
    case Primitive::Cylinder: return "cylinder";
    case Primitive::Sphere: return "sphere";
    case Primitive::Beam: return "beam";
  }
  return "unknown";
}

static float massFor(const SceneObject &object) {
  if (object.name == "Torso") return 12.0f;
  if (object.name.find("Hip") != std::string::npos) return 1.20f;
  if (object.name.find("Shin") != std::string::npos) return 0.80f;
  if (object.name.find("Foot") != std::string::npos) return 0.25f;
  return 1.0f;
}

bool exportRobotParameters(const Scene &scene, int motionScript,
                           const std::filesystem::path &path) {
  static const char *scripts[] = {"stand", "walk", "trot", "jump"};
  json root;
  root["format"] = "jolt_robot_parameters";
  root["format_version"] = 1;
  root["units"] = {{"length", "m"}, {"mass", "kg"}, {"angle", "rad"},
                   {"angular_torque", "N m"}, {"linear_force", "N"},
                   {"angular_velocity", "rad/s"}, {"linear_velocity", "m/s"}};
  root["simulation"] = {{"gravity_m_per_s2", {0.0, -9.81, 0.0}},
                         {"motion_script", scripts[std::clamp(motionScript, 0, 3)]},
                         {"jolt_collision_steps", 1}};
  root["robot"] = {{"name", "Quadruped"},
                   {"total_design_mass_kg", 20.0},
                   {"actuators", {
                     {"hip", {{"type", "hinge"}, {"count", 4}, {"limits_rad", {-0.85, 0.85}}, {"max_torque_Nm", 65.0}, {"servo_frequency_hz", 9.0}, {"damping_ratio", 1.0}}},
                     {"linear_knee", {{"type", "prismatic"}, {"count", 4}, {"stroke_limits_m", {-0.22, 0.22}}, {"target_m", -0.06}, {"max_force_N", 1200.0}, {"servo_frequency_hz", 10.0}, {"damping_ratio", 0.32}}},
                     {"ankle", {{"type", "hinge"}, {"count", 4}, {"limits_rad", {-0.55, 0.55}}, {"max_torque_Nm", 25.0}, {"servo_frequency_hz", 9.0}, {"damping_ratio", 1.0}}}
                   }}};
  root["links"] = json::array();
  for (const SceneObject &object : scene.objects()) {
    root["links"].push_back({
      {"id", object.id}, {"name", object.name}, {"shape", primitiveName(object.primitive)},
      {"dynamic", object.dynamic}, {"mass_kg", massFor(object)},
      {"position_m", {object.transform.position.x, object.transform.position.y, object.transform.position.z}},
      {"rotation_rad", {object.transform.rotation.x, object.transform.rotation.y, object.transform.rotation.z}},
      {"size_m", {object.transform.scale.x, object.transform.scale.y, object.transform.scale.z}}
    });
  }
  std::ofstream output(path);
  if (!output) return false;
  output << root.dump(2) << '\n';
  return static_cast<bool>(output);
}
