#include "editor/JoltBridge.hpp"
#include "editor/Scene.hpp"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <array>
#include <string>

using json = nlohmann::json;

static json runTrial(const StandingTuning &tuning, int script) {
  Scene scene; scene.buildQuadruped();
  JoltBridge physics; physics.setStandingTuning(tuning); physics.initialize();
  physics.rebuild(scene); physics.setRobotScript(script);
  constexpr float dt = 1.0f / 240.0f;
  constexpr int steps = 240 * 20;
  SceneObject *torso = nullptr;
  for (auto &o : scene.objects()) if (o.name == "Torso") torso = &o;
  if (!torso) return {{"stable", false}, {"score", 1e9}};
  const Vec3 initial = torso->transform.position;
  float maxSpeed=0, maxDisplacement=0, minHeight=std::numeric_limits<float>::max();
  float maxError=0; int saturated=0; float instabilityTime=-1.0f; json samples=json::array();
  for (int i=0;i<steps;++i) {
    physics.step(scene, dt);
    const Vec3 p=torso->transform.position;
    maxDisplacement=std::max(maxDisplacement,std::hypot(p.x-initial.x,p.z-initial.z));
    minHeight=std::min(minHeight,p.y);
    const auto &m=physics.telemetry();
    maxSpeed=std::max(maxSpeed,m.torsoSpeedMps);
    for (const auto &leg:m.angleErrorRad) for(float e:leg) maxError=std::max(maxError,std::abs(e));
    saturated += static_cast<int>(std::count(m.torqueSaturated.begin(),m.torqueSaturated.end(),true));
    if (i % 4 == 0) samples.push_back({{"time_s",(i+1)*dt},{"torso_height_m",p.y},
      {"torso_speed_mps",m.torsoSpeedMps},{"horizontal_displacement_m",maxDisplacement},
      {"max_joint_error_rad",maxError}});
    const bool unstable = m.torsoSpeedMps > 0.75f || maxDisplacement > 0.50f ||
                          p.y < 1.50f || maxError > 0.75f;
    if (unstable) { instabilityTime = (i+1)*dt; break; }
  }
  const float score=maxSpeed*2.0f+maxDisplacement*4.0f+
      std::max(0.0f,1.50f-minHeight)*3.0f+maxError+saturated*0.002f;
  return {{"stable",maxSpeed<0.75f && maxDisplacement<0.20f && minHeight>1.50f},
          {"score",score},{"max_torso_speed_mps",maxSpeed},
          {"max_horizontal_displacement_m",maxDisplacement},
          {"min_torso_height_m",minHeight},{"max_joint_error_rad",maxError},
          {"torque_saturated_samples",saturated},{"instability_time_s",instabilityTime},{"samples",samples},
          {"motor_frequency_hz",tuning.motorFrequencyHz},
          {"motor_damping",tuning.motorDamping},
          {"com_gain",tuning.comGain},{"velocity_gain",tuning.velocityGain}};
}

int main(int argc, char **argv) {
  const bool walking = argc > 1 && std::string(argv[1]) == "walk";
  const int script = walking ? 1 : 0;
  // Ten deterministic controller candidates: damping and balance gains are
  // varied around the current model, then the lowest-scoring trial wins.
  const std::array<StandingTuning,10> candidates{{
    {2.0f,1.6f,0.18f,0.04f},{2.5f,2.0f,0.24f,0.06f},
    {3.0f,2.4f,0.30f,0.08f},{3.5f,2.8f,0.36f,0.10f},
    {4.0f,3.2f,0.42f,0.12f},{2.5f,3.5f,0.30f,0.14f},
    {3.0f,4.0f,0.45f,0.16f},{4.0f,4.5f,0.55f,0.20f},
    {5.0f,3.0f,0.22f,0.18f},{1.8f,2.8f,0.50f,0.10f}
  }};
  json trials=json::array(); json best; float bestScore=std::numeric_limits<float>::max();
  for (const auto &candidate:candidates) {
    json result=runTrial(candidate, script); trials.push_back(result);
    if (result["score"].get<float>()<bestScore) { bestScore=result["score"]; best=result; }
  }
  json output={{"iterations",candidates.size()},{"controller",walking ? "walk" : "stand"},
               {"best",best},{"trials",trials}};
  std::ofstream file("headless_standing_result.json"); file<<output.dump(2)<<'\n';
  std::cout<<output.dump(2)<<'\n';
  return best.value("stable",false) ? 0 : 1;
}
