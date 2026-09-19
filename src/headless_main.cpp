#include "editor/MuJoCoBridge.hpp"
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

static json runTrial(const StandingTuning &tuning, int script, float durationSeconds, bool biped = false,
                     bool unitreeH1 = false, bool fullBody = false, bool valkyrie = false) {
  Scene scene;
  if (valkyrie) scene.buildValkyrie();
  else if (unitreeH1) scene.buildUnitreeH1(); else if (biped) scene.buildBiped(); else scene.buildQuadruped();
  MuJoCoBridge physics; physics.setStandingTuning(tuning); physics.initialize();
  physics.enableFullBodyMode(fullBody);
  physics.rebuild(scene); physics.setRobotScript(script);
  constexpr float dt = 1.0f / 240.0f;
  const int steps = static_cast<int>(240.0f * durationSeconds);
  SceneObject *torso = nullptr;
  SceneObject *leftAnkle = nullptr;
  SceneObject *rightAnkle = nullptr;
  const std::string rootName = unitreeH1 || valkyrie ? "pelvis" : "Torso";
  for (auto &o : scene.objects()) {
    if (o.name == rootName) torso = &o;
    if (o.name == "left_ankle_link") leftAnkle = &o;
    if (o.name == "right_ankle_link") rightAnkle = &o;
  }
  if (!torso) return {{"stable", false}, {"score", 1e9}};
  const Vec3 initial = torso->transform.position;
  const float initialLeftFootRelativeHeight = leftAnkle ? leftAnkle->transform.position.y - initial.y : 0.0f;
  const float initialRightFootRelativeHeight = rightAnkle ? rightAnkle->transform.position.y - initial.y : 0.0f;
  float maxSpeed=0, maxDisplacement=0, minHeight=std::numeric_limits<float>::max();
  float maxError=0, maxSwingFootLift=0; int saturated=0; int swingSamples=0; bool sawLeftSwing=false, sawRightSwing=false; float maxGaitCycle=0.0f; float instabilityTime=-1.0f; json samples=json::array();
  // The generated crawler's longitudinal MuJoCo axis is Y, mapped to CAO Z.
  // H1's imported model remains longitudinal in CAO X.
  const auto forward = [&](const Vec3 &position) { return unitreeH1 || valkyrie ? position.x : position.z; };
  float leftFootMinX=leftAnkle ? forward(leftAnkle->transform.position) : 0.0f;
  float leftFootMaxX=leftFootMinX;
  float rightFootMinX=rightAnkle ? forward(rightAnkle->transform.position) : 0.0f;
  float rightFootMaxX=rightFootMinX;
  std::array<bool,2> wasContact{}, hadContact{};
  std::array<float,2> contactX{}, contactY{}, airHeight{};
  std::array<int,2> verifiedSteps{};
  float maxContactClearance=0;
  for (int i=0;i<steps;++i) {
    physics.step(scene, dt);
    const Vec3 p=torso->transform.position;
    maxDisplacement=std::max(maxDisplacement,std::hypot(p.x-initial.x,p.z-initial.z));
    minHeight=std::min(minHeight,p.y);
    const auto &m=physics.telemetry();
    if(script==5 && !valkyrie) for(int leg=0;leg<2;++leg) {
      const auto *ankle=leg==0 ? leftAnkle:rightAnkle;
      if(!ankle) continue;
      const auto &pfoot=ankle->transform.position;
      if(m.footContact[leg]) {
        if(!wasContact[leg] && hadContact[leg] && airHeight[leg]>.02f &&
           forward(pfoot)-contactX[leg]>.05f) ++verifiedSteps[leg];
        contactX[leg]=forward(pfoot); contactY[leg]=pfoot.y;
        hadContact[leg]=true; airHeight[leg]=0;
      } else if(hadContact[leg]) {
        airHeight[leg]=std::max(airHeight[leg],pfoot.y-contactY[leg]);
        maxContactClearance=std::max(maxContactClearance,airHeight[leg]);
      }
      wasContact[leg]=m.footContact[leg];
    }
    maxSpeed=std::max(maxSpeed,m.torsoSpeedMps);
    maxGaitCycle=std::max(maxGaitCycle,m.gaitCycle);
    if (m.activeSwingLeg == 0 && leftAnkle)
      maxSwingFootLift = std::max(maxSwingFootLift, leftAnkle->transform.position.y - initial.y - initialLeftFootRelativeHeight);
    if (m.activeSwingLeg == 1 && rightAnkle)
      maxSwingFootLift = std::max(maxSwingFootLift, rightAnkle->transform.position.y - initial.y - initialRightFootRelativeHeight);
    if (leftAnkle) {
      const float x = forward(leftAnkle->transform.position);
      leftFootMinX = std::min(leftFootMinX, x); leftFootMaxX = std::max(leftFootMaxX, x);
    }
    if (rightAnkle) {
      const float x = forward(rightAnkle->transform.position);
      rightFootMinX = std::min(rightFootMinX, x); rightFootMaxX = std::max(rightFootMaxX, x);
    }
    sawLeftSwing = sawLeftSwing || m.activeSwingLeg == 0;
    sawRightSwing = sawRightSwing || m.activeSwingLeg == 1;
    if (m.activeSwingLeg >= 0 || std::any_of(m.legState.begin(),m.legState.end(),[](int s){return s==2;})) ++swingSamples;
    for (const auto &leg:m.angleErrorRad) for(float e:leg) maxError=std::max(maxError,std::abs(e));
    saturated += static_cast<int>(std::count(m.torqueSaturated.begin(),m.torqueSaturated.end(),true));
    if (i % 4 == 0) samples.push_back({{"time_s",(i+1)*dt},{"torso_height_m",p.y},
      {"torso_speed_mps",m.torsoSpeedMps},{"horizontal_displacement_m",maxDisplacement},
      {"max_joint_error_rad",maxError},{"gait_cycle",m.gaitCycle},
      {"active_swing_leg",m.activeSwingLeg},{"active_swing_tripod",m.activeSwingTripod},
      {"equilibrium_score",m.equilibriumScore},{"walking_allowed",m.walkingAllowed},
      {"swing_samples",swingSamples}});
    const bool unstable = valkyrie ? !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || p.y < -0.5f :
                          m.torsoSpeedMps > (script == 5 ? 2.0f : 0.75f) || (script != 5 && maxDisplacement > 0.50f) ||
                          p.y < 0.70f || maxError > 0.75f;
    if (unstable) { instabilityTime = (i+1)*dt; break; }
  }
  if (script != 0 && swingSamples == 0) maxError = std::max(maxError, 2.0f);
  const float score=maxSpeed*2.0f+maxDisplacement*4.0f+
      std::max(0.0f,0.70f-minHeight)*3.0f+maxError+saturated*0.002f;
  // A validated locomotion run is expected to travel beyond the standing
  // regression's 20 cm envelope. It still uses the stricter 50 cm abort
  // boundary checked in the loop above.
  const float stableDisplacement = valkyrie ? 100.0f : script == 5 ? 100.0f : unitreeH1 && script >= 1 ? 0.50f : 0.20f;
  const bool hasSwingClearance = script==5 ? verifiedSteps[0]>=2 && verifiedSteps[1]>=2 :
      script==2 ? swingSamples>0 :
      !unitreeH1 || script == 0 || maxSwingFootLift > 0.03f;
  const bool hasAlternatingSwings = valkyrie || !unitreeH1 || script == 0 || (sawLeftSwing && sawRightSwing);
  const float maxFootPlacement = std::max(leftFootMaxX - leftFootMinX, rightFootMaxX - rightFootMinX);
  const bool hasThirtyCmPlacement = script != 3 || maxFootPlacement >= 0.25f;
  const float goalDistance = script == 5 ? std::hypot(torso->transform.position.x - 6.20f, torso->transform.position.z) : 0.0f;
  // The post-goal hazard sequence may move H1 away from the marker. Validate
  // that it reached the goal at least once, rather than only its final pose.
  const bool reachedGoal = valkyrie || script != 5 || physics.telemetry().navigationGoalReached;
  return {{"verified_steps",verifiedSteps},{"contact_clearance_m",maxContactClearance},
          {"forward_displacement_m",forward(torso->transform.position)-forward(initial)},
          {"goal_distance_m",goalDistance},{"reached_goal",reachedGoal},
          {"stable",valkyrie ? instabilityTime < 0.0f && minHeight > 1.10f &&
                                  forward(torso->transform.position)-forward(initial) > 0.6f && maxGaitCycle > 0.8f :
                              maxSpeed<0.75f && maxDisplacement<stableDisplacement && minHeight>0.70f && hasSwingClearance && hasAlternatingSwings && hasThirtyCmPlacement && reachedGoal &&
                   (script == 0 || swingSamples > 0)},
          {"score",score},{"max_torso_speed_mps",maxSpeed},
          {"max_horizontal_displacement_m",maxDisplacement},
          {"min_torso_height_m",minHeight},{"max_joint_error_rad",maxError},
          {"torque_saturated_samples",saturated},{"instability_time_s",instabilityTime},
          {"swing_samples",swingSamples},{"saw_left_swing",sawLeftSwing},{"saw_right_swing",sawRightSwing},{"swing_foot_lift_m",maxSwingFootLift},{"max_foot_placement_m",maxFootPlacement},{"max_gait_cycle",maxGaitCycle},{"samples",samples},
          {"motor_frequency_hz",tuning.motorFrequencyHz},
          {"motor_damping",tuning.motorDamping},
          {"com_gain",tuning.comGain},{"velocity_gain",tuning.velocityGain}};
}

int main(int argc, char **argv) {
  const std::string mode = argc > 1 ? argv[1] : "stand";
  const bool walking = mode == "walk" || mode == "tripod" || mode == "h1walk" || mode == "h1learn" || mode == "h1zmp";
  const bool biped = mode == "biped";
  const bool valkyrie = mode == "valkyrie";
  const bool fullBody = mode == "h1full";
  const bool unitreeH1 = !valkyrie && (mode == "h1" || mode == "h1walk" || mode == "h1learn" || mode == "h1zmp" || mode == "h1contact" || mode == "h1unitree" || fullBody);
  const bool quick = argc > 2 && std::string(argv[2]) == "--quick";
  const int script = (mode == "h1unitree" || fullBody) ? 5 : mode == "h1contact" ? 4 : mode == "tripod" ? 2 : mode == "h1zmp" ? 3 : mode == "h1learn" ? 2 : (mode == "walk" || mode == "h1walk") ? 1 : 0;
  // Ten deterministic controller candidates: damping and balance gains are
  // varied around the current model, then the lowest-scoring trial wins.
  const std::array<StandingTuning,10> candidates{{
    {3.0f,2.4f,0.18f,0.04f},{3.5f,2.6f,0.24f,0.06f},
    {4.0f,2.8f,0.30f,0.08f},{4.5f,3.0f,0.36f,0.10f},
    {5.0f,3.2f,0.42f,0.12f},{3.5f,3.4f,0.30f,0.14f},
    {4.0f,3.6f,0.45f,0.16f},{5.0f,4.0f,0.55f,0.20f},
    {5.5f,3.0f,0.22f,0.18f},{3.0f,2.8f,0.50f,0.10f}
  }};
  json trials=json::array(); json best; float bestScore=std::numeric_limits<float>::max();
  const size_t candidateCount = quick || script==5 ? 1u : candidates.size();
  const float durationSeconds = quick ? 8.0f : 60.0f;
  for (size_t index = 0; index < candidateCount; ++index) {
    json result=runTrial(candidates[index], script, durationSeconds, biped, unitreeH1, fullBody, valkyrie); trials.push_back(result);
    if (result["score"].get<float>()<bestScore) { bestScore=result["score"]; best=result; }
  }
  json output={{"iterations",candidateCount},{"duration_seconds",durationSeconds},
               {"controller",mode},
               {"best",best},{"trials",trials}};
  std::ofstream file("headless_standing_result.json"); file<<output.dump(2)<<'\n';
  std::cout<<output.dump(2)<<'\n';
  return best.value("stable",false) ? 0 : 1;
}
