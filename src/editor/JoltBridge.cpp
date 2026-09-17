#include "JoltBridge.hpp"
#include "JoltLayers.hpp"
#include "LegGeometry.hpp"
#include "BalanceModel.hpp"
#include "EquilibriumPolicy.hpp"

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyLockInterface.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#include <algorithm>
#include <array>
#include <thread>
#include <unordered_map>

namespace {
constexpr float kPlantedPawFriction = 2.10f;
constexpr float kSwingPawFriction = 0.08f;
}

struct JoltBridge::Impl {
  bool ready = false;
  CaoBroadPhaseLayerInterface broadPhaseLayers;
  CaoObjectVsBroadPhaseFilter objectVsBroadPhase;
  CaoObjectLayerPairFilter objectPairs;
  std::unique_ptr<JPH::PhysicsSystem> physics;
  std::unique_ptr<JPH::TempAllocatorImpl> allocator;
  std::unique_ptr<JPH::JobSystemThreadPool> jobs;
  std::unordered_map<uint32_t, JPH::BodyID> bodies;
  std::vector<JPH::Ref<JPH::Constraint>> actuators;
  std::vector<JPH::Ref<JPH::HingeConstraint>> rotaryActuators;
  int script = 0;
  float scriptTime = 0.0f;
  JPH::BodyID ground;
  JPH::BodyID torso;
  std::array<JPH::BodyID, kRobotLegCount> feet{};
  std::array<JPH::BodyID, kRobotLegCount> shins{};
  RobotTelemetry telemetry;
  StandingTuning tuning;
  bool motorsEnabled = false;
  float settleTime = 0.0f;
  float motorBlend = 0.0f;
  FastEquilibriumPolicy equilibriumPolicy;
};

JoltBridge::JoltBridge() : impl_(std::make_unique<Impl>()) {}
JoltBridge::~JoltBridge() { shutdown(); }

void JoltBridge::initialize() {
  if (impl_->ready) return;
  JPH::RegisterDefaultAllocator();
  JPH::Factory::sInstance = new JPH::Factory();
  JPH::RegisterTypes();

  impl_->physics = std::make_unique<JPH::PhysicsSystem>();
  impl_->physics->Init(2048, 0, 4096, 4096,
                       impl_->broadPhaseLayers, impl_->objectVsBroadPhase,
                       impl_->objectPairs);
  impl_->allocator = std::make_unique<JPH::TempAllocatorImpl>(10 * 1024 * 1024);
  const unsigned int workers = std::max(1u, std::thread::hardware_concurrency() > 1
      ? std::thread::hardware_concurrency() - 1 : 1u);
  impl_->jobs = std::make_unique<JPH::JobSystemThreadPool>(
      JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, workers);
  impl_->ready = true;
}

void JoltBridge::rebuild(Scene &scene) {
  if (!impl_->ready) return;

  auto &bodies = impl_->physics->GetBodyInterface();
  for (const auto &actuator : impl_->actuators) impl_->physics->RemoveConstraint(actuator);
  impl_->actuators.clear(); impl_->rotaryActuators.clear();
  for (const auto &[id, body] : impl_->bodies) {
    bodies.RemoveBody(body);
    bodies.DestroyBody(body);
  }
  impl_->bodies.clear();
  impl_->feet.fill(JPH::BodyID());
  impl_->shins.fill(JPH::BodyID());
  impl_->torso = JPH::BodyID();
  impl_->motorsEnabled = false;
  impl_->settleTime = 0.0f;
  impl_->equilibriumPolicy.reset(false);

  if (!impl_->ground.IsInvalid()) {
    bodies.RemoveBody(impl_->ground);
    bodies.DestroyBody(impl_->ground);
  }

  JPH::BoxShapeSettings groundShape(JPH::Vec3(50.0f, 0.25f, 50.0f));
  groundShape.SetEmbedded();
  JPH::BodyCreationSettings groundSettings(
      &groundShape, JPH::RVec3(0.0, -0.25, 0.0), JPH::Quat::sIdentity(),
      JPH::EMotionType::Static, CaoObjectLayers::Static);
  // A robot needs traction, not a bouncing floor. This is a rubber-mat contact.
  groundSettings.mFriction = 1.05f;
  groundSettings.mRestitution = 0.0f;
  impl_->ground = bodies.CreateAndAddBody(groundSettings, JPH::EActivation::DontActivate);

  for (SceneObject &object : scene.objects()) {
    const JPH::Vec3 halfExtent(
        std::max(0.05f, object.transform.scale.x * 0.5f),
        std::max(0.05f, object.transform.scale.y * 0.5f),
        std::max(0.05f, object.transform.scale.z * 0.5f));
    JPH::BoxShapeSettings shape(halfExtent);
    // ShapeSettings inherits RefTarget; stack instances must not self-delete.
    shape.SetEmbedded();
    const JPH::EMotionType motion =
        object.dynamic ? JPH::EMotionType::Dynamic : JPH::EMotionType::Static;
    JPH::BodyCreationSettings settings(
        &shape,
        JPH::RVec3(object.transform.position.x, object.transform.position.y,
                   object.transform.position.z),
        // Vulkan renders Z * Y * X Euler rotations; Jolt's Euler helper uses
        // that same object-space orientation, so contacts cannot be offset
        // from a visibly rotated box.
        JPH::Quat::sEulerAngles(JPH::Vec3(object.transform.rotation.x,
                                          object.transform.rotation.y,
                                          object.transform.rotation.z)), motion,
        object.dynamic
            ? (scene.isQuadruped()
                ? CaoObjectLayers::RobotLink : CaoObjectLayers::Dynamic)
            : CaoObjectLayers::Static);
    // Feet are the contact pads. High tangential traction limits sliding
    // while the torso and links remain free to move.
    settings.mFriction = object.name.find("Foot") != std::string::npos ? kPlantedPawFriction : 0.58f;
    settings.mRestitution = 0.0f;
    // Lightweight prototype mass model: every robot-link mass is reduced 10x
    // to improve controller authority while preserving the same geometry.
    float massKg = 1.0f;
    if (object.name == "Torso") massKg = 0.90f;
    else if (object.name.find("Hip Roll") != std::string::npos) massKg = 0.03f;
    else if (object.name.find("Hip") != std::string::npos) massKg = 0.12f;
    else if (object.name.find("Shin") != std::string::npos) massKg = 0.08f;
    else if (object.name.find("Foot") != std::string::npos) massKg = 0.15f;
    if (object.dynamic) {
      settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
      settings.mMassPropertiesOverride.mMass = massKg;
    }
    const JPH::BodyID body = bodies.CreateAndAddBody(
        settings, object.dynamic ? JPH::EActivation::Activate
                                 : JPH::EActivation::DontActivate);
    impl_->bodies.emplace(object.id, body);
    if (object.name == "Torso") impl_->torso = body;
    object.joltBody = body.GetIndexAndSequenceNumber();
  }

  if (scene.isQuadruped()) {
    const auto findBody = [&](const std::string &name) -> JPH::BodyID {
      for (const SceneObject &object : scene.objects())
        if (object.name == name) return impl_->bodies.at(object.id);
      return JPH::BodyID();
    };
    const auto addHinge = [&](const std::string &parent, const std::string &child,
                               float x, float y, float z, const JPH::Vec3 &axis,
                               float minAngle, float maxAngle, float targetAngle, float maxTorque) {
      const JPH::BodyID parentId = findBody(parent), childId = findBody(child);
      if (parentId.IsInvalid() || childId.IsInvalid()) return;
      const auto &locks = impl_->physics->GetBodyLockInterface();
      const JPH::BodyID pair[] = {parentId, childId};
      JPH::BodyLockMultiWrite pairLock(locks, pair, 2);
      JPH::Body *parentBody = pairLock.GetBody(0);
      JPH::Body *childBody = pairLock.GetBody(1);
      if (parentBody == nullptr || childBody == nullptr) return;
      JPH::HingeConstraintSettings settings;
      settings.mSpace = JPH::EConstraintSpace::WorldSpace;
      settings.mPoint1 = settings.mPoint2 = JPH::RVec3(x, y, z);
      settings.mHingeAxis1 = settings.mHingeAxis2 = axis;
      settings.mNormalAxis1 = settings.mNormalAxis2 = JPH::Vec3::sAxisY();
      settings.mLimitsMin = minAngle; settings.mLimitsMax = maxAngle;
      settings.mMotorSettings.SetTorqueLimit(maxTorque); // N m
      // Support joints use the same damped response. Power arrives only after
      // the settle delay and is blended over one second in step().
      settings.mMotorSettings.mSpringSettings.mFrequency = impl_->tuning.motorFrequencyHz;
      settings.mMotorSettings.mSpringSettings.mDamping = impl_->tuning.motorDamping;
      JPH::Ref<JPH::HingeConstraint> actuator = new JPH::HingeConstraint(
          *parentBody, *childBody, settings);
      // Start unpowered. The rigid links settle under Jolt constraints before
      // position control is enabled, preventing an initialization impulse.
      actuator->SetMotorState(JPH::EMotorState::Off);
      actuator->SetTargetAngle(targetAngle); // radians
      impl_->physics->AddConstraint(actuator);
      impl_->rotaryActuators.emplace_back(actuator);
      impl_->actuators.emplace_back(std::move(actuator));
    };
    size_t leg = 0;
    for (int side : {-1, 1}) for (int station : {-1, 0, 1}) {
      const std::string prefix = std::string(station < 0 ? "Front" : station > 0 ? "Rear" : "Middle") +
          " " + (side < 0 ? "Left" : "Right");
      const float x = CaoLegGeometry::hipOffsetX * side, z = CaoLegGeometry::hipOffsetZ * station;
      impl_->feet[leg] = findBody(prefix + " Foot");
      impl_->shins[leg] = findBody(prefix + " Shin");
      // 4 revolute actuators per leg: hip roll, hip pitch, knee pitch, ankle pitch.
      addHinge("Torso", prefix + " Hip Roll", x, CaoLegGeometry::torsoHipHeight + 0.11f, z, JPH::Vec3::sAxisZ(),
               CaoLegGeometry::hipRollMinAngle, CaoLegGeometry::hipRollMaxAngle,
               0.0f, CaoLegGeometry::hipRollTorqueNm);
      addHinge(prefix + " Hip Roll", prefix + " Hip", x, CaoLegGeometry::torsoHipHeight, z, JPH::Vec3::sAxisX(),
               CaoLegGeometry::hipPitchMinAngle, CaoLegGeometry::hipPitchMaxAngle,
               0.0f, CaoLegGeometry::hipPitchTorqueNm);
      addHinge(prefix + " Hip", prefix + " Shin", x, CaoLegGeometry::kneeHeight, z, JPH::Vec3::sAxisX(),
               CaoLegGeometry::kneePitchMinAngle, CaoLegGeometry::kneePitchMaxAngle,
               0.0f, CaoLegGeometry::kneePitchTorqueNm);
      addHinge(prefix + " Shin", prefix + " Foot", x, CaoLegGeometry::ankleHeight, z, JPH::Vec3::sAxisX(),
               CaoLegGeometry::anklePitchMinAngle, CaoLegGeometry::anklePitchMaxAngle,
               0.0f, CaoLegGeometry::anklePitchTorqueNm);
      ++leg;
    }
  }
}


void JoltBridge::demolish(const Scene &scene) {
  if (!impl_->ready) return;
  auto &bodies = impl_->physics->GetBodyInterface();
  for (const SceneObject &object : scene.objects()) {
    if (!object.dynamic) continue;
    const auto it = impl_->bodies.find(object.id);
    if (it == impl_->bodies.end()) continue;
    const float side = (object.id % 2u == 0u) ? 1.0f : -1.0f;
    const float forward = ((object.id / 2u) % 2u == 0u) ? 1.0f : -1.0f;
    const float height = std::max(0.0f, object.transform.position.y);
    const JPH::Vec3 blast(side * (6.0f + height), 5.0f + height * 2.0f,
                               forward * (5.0f + height));
    bodies.ActivateBody(it->second);
    bodies.SetLinearVelocity(it->second, blast);
    bodies.AddImpulse(it->second, blast * 2.0f);
  }
}

void JoltBridge::step(Scene &scene, float seconds) {
  if (!impl_->ready || seconds <= 0.0f) return;
  impl_->scriptTime += seconds;
  impl_->settleTime += seconds;
  if (impl_->motorsEnabled) impl_->motorBlend = std::min(1.0f, impl_->motorBlend + seconds);
  if (!impl_->motorsEnabled && impl_->settleTime >= 0.50f) {
    impl_->motorsEnabled = true;
    impl_->motorBlend = 0.0f;
    for (const auto &actuator : impl_->rotaryActuators)
      actuator->SetMotorState(JPH::EMotorState::Position);
  }
  impl_->telemetry = {};
  impl_->telemetry.motionScript = impl_->script;
  impl_->telemetry.linkCount = static_cast<int>(scene.objects().size());
  impl_->telemetry.torqueLimitsNm = {{CaoLegGeometry::hipRollTorqueNm, CaoLegGeometry::hipPitchTorqueNm, CaoLegGeometry::kneePitchTorqueNm, CaoLegGeometry::anklePitchTorqueNm}};
  const float phase = impl_->scriptTime * (impl_->script == 3 ? 7.0f : 4.4f);
  // A standing foot is commanded to its assembled neutral position. Do not
  // pre-bend all legs to manufacture correction travel: that changes the
  // support height under every foot at once and can turn a small error into a
  // fall. Only the selected swing leg is allowed to bend during a walk.
  constexpr float supportHip = 0.0f;
  constexpr float supportKnee = 0.0f;
  constexpr float supportAnkle = 0.0f;
  float comY = 0.0f, comVx = 0.0f, comVz = 0.0f;
  if (!impl_->torso.IsInvalid()) {
    JPH::BodyLockRead torsoLock(impl_->physics->GetBodyLockInterface(), impl_->torso);
    if (torsoLock.Succeeded()) {
      const JPH::Body &body = torsoLock.GetBody();
      const JPH::RVec3 p = body.GetPosition();
      const JPH::Vec3 v = body.GetLinearVelocity();
      comY = static_cast<float>(p.GetY());
      comVx = v.GetX(); comVz = v.GetZ();
    }
  }
  int validFeet = 0;
  const auto nominalFootPoint = [](size_t leg) {
    const float side = leg < 3 ? -1.0f : 1.0f;
    const int station = static_cast<int>(leg % 3) - 1;
    return CaoBalance::Point{CaoLegGeometry::hipOffsetX * side,
                             CaoLegGeometry::hipOffsetZ * station};
  };
  for (size_t i = 0; i < kRobotLegCount; ++i) {
    if (!impl_->feet[i].IsInvalid()) {
      JPH::BodyLockRead footLock(impl_->physics->GetBodyLockInterface(), impl_->feet[i]);
      if (footLock.Succeeded()) {
        ++validFeet;
      }
    }
  }
  const float blend = impl_->motorsEnabled
      ? std::clamp(impl_->motorBlend, 0.0f, 1.0f) : 0.0f;
  const auto equilibrium = impl_->equilibriumPolicy.update({
      comY, std::hypot(comVx, comVz),
      static_cast<float>(validFeet) / static_cast<float>(kRobotLegCount)});
  impl_->telemetry.equilibriumScore = equilibrium.score;
  impl_->telemetry.walkingAllowed = equilibrium.walkingAllowed;
  const auto setLegPose = [&](size_t leg, float roll, float hip, float knee, float ankle) {
    const size_t first = leg * 4u;
    const std::array<float,4> desired{{roll, hip, knee, ankle}};
    std::array<float,4> target{};
    for (size_t joint=0; joint<4; ++joint) {
      const float current = impl_->rotaryActuators[first + joint]->GetCurrentAngle();
      target[joint] = current + (desired[joint] - current) * blend;
      impl_->rotaryActuators[first + joint]->SetTargetAngle(target[joint]);
    }
    impl_->telemetry.targetAnglesRad[leg] = target;
  };
  if (impl_->script == 0) { // Stand
    // A standing robot holds a constant geometry. Attitude recovery is done
    // by the bounded torso torque below, never by shortening every leg.
    for (size_t leg = 0; leg < kRobotLegCount; ++leg)
      setLegPose(leg, 0.0f, supportHip, supportKnee, supportAnkle);
  } else if (impl_->script == 1 || impl_->script == 2) { // Walk / trot
    // One foot moves at a time. Five feet remain at the fixed neutral height,
    // which gives the balance gate a large, non-changing support envelope.
    // Script 2 retains its faster cadence but is still a single-foot gait.
    const float cycleDuration = impl_->script == 1 ? 6.00f : 4.00f;
    // Fade in the locomotion envelope after the neutral pose has settled.
    // This avoids an initial impulse from commanding six different support
    // reaches when motors first engage.
    const float gaitTime = std::max(0.0f, impl_->scriptTime - 1.50f);
    const float gaitRamp = std::clamp(gaitTime / 5.0f, 0.0f, 1.0f);
    const float globalCycle = std::fmod(gaitTime / cycleDuration, 1.0f);
    const float stanceSweep = 0.025f * gaitRamp;
    impl_->telemetry.activeSwingTripod = -1;
    auto &bodyInterface = impl_->physics->GetBodyInterface();
    for (size_t leg = 0; leg < kRobotLegCount; ++leg) {
      // With six evenly phased legs and a 12% swing window, at most one leg
      // is airborne. The other five slowly sweep through stance.
      const float cycle = std::fmod(globalCycle +
          static_cast<float>(leg) / static_cast<float>(kRobotLegCount), 1.0f);
      const float side = leg < 3 ? -1.0f : 1.0f;
      const bool tripodA = leg == 0 || leg == 2 || leg == 4;
      const int tripod = tripodA ? 0 : 1;
      constexpr float swingStart = 0.76f;
      constexpr float swingEnd = 0.88f;
      const bool scheduledForSwing = cycle >= swingStart && cycle < swingEnd;
      // Do not lift the selected foot unless the torso projection remains
      // inside the five-foot support envelope.
      CaoBalance::Point com{};
      if (!impl_->torso.IsInvalid()) {
        JPH::BodyLockRead torsoLock(impl_->physics->GetBodyLockInterface(), impl_->torso);
        if (torsoLock.Succeeded()) {
          const JPH::RVec3 p = torsoLock.GetBody().GetPosition();
          com = {static_cast<float>(p.GetX()), static_cast<float>(p.GetZ())};
        }
      }
      std::array<CaoBalance::Point, kRobotLegCount> footPoints{};
      for (size_t i = 0; i < kRobotLegCount; ++i) {
        footPoints[i] = nominalFootPoint(i);
        if (!impl_->feet[i].IsInvalid()) {
          JPH::BodyLockRead footLock(impl_->physics->GetBodyLockInterface(), impl_->feet[i]);
          if (footLock.Succeeded()) {
            const JPH::RVec3 p = footLock.GetBody().GetPosition();
            footPoints[i] = {static_cast<float>(p.GetX()), static_cast<float>(p.GetZ())};
          }
        }
      }
      std::array<CaoBalance::Point, kRobotLegCount> nominalFeet{};
      for (size_t i = 0; i < kRobotLegCount; ++i)
        nominalFeet[i] = nominalFootPoint(i);
      // Prefer measured feet, but retain the nominal footprint when contacts
      // are temporarily degenerate during a simulation step.
      const bool balanceReady =
          CaoBalance::insideSupportBounds(com, footPoints, leg, 0.03f) ||
          CaoBalance::insideSupportBounds(com, nominalFeet, leg, 0.03f);
      // Jolt reports the assembled neutral hinge reference near -0.16 rad.
      // Offset the commanded joint angle so the physical femur/tibia pose
      // reaches the requested 90 degrees instead of accumulating the rest
      // pose offset as tracking error.
      float roll = 0.0f, hip = 0.0f, knee = supportKnee, ankle = supportAnkle;
      float swingLiftForceN = 0.0f;
      int state = 0;
      bool planted = cycle < (impl_->script == 1 ? swingStart : 0.66f) ||
          cycle >= (impl_->script == 1 ? swingEnd : 0.96f);
      if (cycle < (impl_->script == 1 ? 0.72f : 0.58f)) { // crawl stance: five legs support the body
        const float t = cycle / (impl_->script == 1 ? 0.72f : 0.58f);
        if (impl_->script == 1) {
          // A shallow planted-foot sweep creates forward ground reaction.
          // Hip pitch is the only support DOF used, so foot height remains
          // fixed and the knee/ankle stay load-bearing.
          hip = stanceSweep * (1.0f - 2.0f * t);
          roll = 0.0f;
          ankle = supportAnkle;
        } else {
          hip = 0.18f - 0.42f * t;
          ankle = -0.10f * hip;
        }
      } else if (cycle < (impl_->script == 1 ? swingStart : 0.66f)) { // unload before the single-leg swing
        state = 1;
        const float t = (cycle - (impl_->script == 1 ? 0.72f : 0.58f)) /
                        (impl_->script == 1 ? swingStart - 0.72f : 0.06f);
        hip = impl_->script == 1 ? -stanceSweep : -0.24f + 0.05f * t;
        roll = impl_->script == 1 ? 0.0f : side * 0.12f;
      } else if (cycle < (impl_->script == 1 ? swingEnd : 0.96f) &&
                 balanceReady && equilibrium.walkingAllowed && scheduledForSwing) {
        state = 2;
        const float t = (cycle - (impl_->script == 1 ? swingStart : 0.66f)) / (impl_->script == 1 ? swingEnd - swingStart : 0.30f);
        const float lift = std::sin(JPH::JPH_PI * t);
        hip = impl_->script == 1 ? -stanceSweep + 2.0f * stanceSweep * t : -0.19f + 0.43f * t;
        knee = supportKnee - (impl_->script == 1 ? 0.25f * gaitRamp : 0.62f) * lift;
        ankle = supportAnkle + (impl_->script == 1 ? 0.06f : 0.20f) * lift;
        roll = impl_->script == 1 ? 0.0f : side * 0.10f * (1.0f - lift);
        // Joint motors alone perform the lift. Do not inject an additional
        // vertical force into a single leg: it creates an impulse at the
        // contact and destabilizes the rest of the support polygon.
        planted = false;
      } else { // place: extend the knee before high traction returns
        state = 3;
        const float t = (cycle - (impl_->script == 1 ? swingEnd : 0.96f)) / (impl_->script == 1 ? 1.0f - swingEnd : 0.04f);
        hip = impl_->script == 1 ? stanceSweep : 0.24f - 0.06f * t;
        knee = impl_->script == 1 ? supportKnee : supportKnee - 0.18f * (1.0f - t);
        ankle = impl_->script == 1 ? supportAnkle : 0.05f * (1.0f - t);
      }
      setLegPose(leg, roll, hip, knee, ankle);
      impl_->telemetry.legState[leg] = state;
      impl_->telemetry.footFriction[leg] = planted ? kPlantedPawFriction : kSwingPawFriction;
      impl_->telemetry.liftAssistForceN[leg] = swingLiftForceN;
      if (state == 2) {
        if (impl_->telemetry.activeSwingLeg < 0)
          impl_->telemetry.activeSwingLeg = static_cast<int>(leg);
        impl_->telemetry.activeSwingTripod = tripod;
        impl_->telemetry.swingLiftForceN = swingLiftForceN;
      }
      if (!impl_->feet[leg].IsInvalid())
        bodyInterface.SetFriction(impl_->feet[leg], planted ? kPlantedPawFriction : kSwingPawFriction);
      // During crawl stance, a bounded body force supplies the horizontal
      // ground reaction that joint targets alone cannot create. It is applied
      // only while this leg is planted, so three legs share propulsion.
      // Crawl stability phase intentionally has no artificial propulsion.
      // First establish three-leg equilibrium; propulsion is added only after
      // this balance test is reliable.
      // A planted foot pushes against the floor to propel the body. The force
      // is applied to the contact body, allowing ground friction and the Jolt
      // constraints to transmit the reaction through the leg chain.
      if (impl_->script == 1 && planted && !impl_->feet[leg].IsInvalid()) {
        // Do not inject horizontal energy until footstep placement is
        // solved. The old push made the robot drift faster than its support
        // polygon and was the primary cause of the recorded collapse.
      }
    }
  } else if (impl_->script == 3) { // Repeated jump
    const float extension = std::max(0.0f, std::sin(phase));
    for (size_t leg = 0; leg < kRobotLegCount; ++leg)
      setLegPose(leg, 0.0f, 0.0f, -0.85f + 0.70f * extension, 0.0f);
  }
  impl_->telemetry.gaitCycle = impl_->script == 1 || impl_->script == 2
      ? std::fmod(impl_->scriptTime / (impl_->script == 1 ? 6.00f : 4.00f), 1.0f) : 0.0f;
  impl_->physics->Update(seconds, 4, impl_->allocator.get(), impl_->jobs.get());

  // Active torso stabilization: use the measured Jolt attitude and angular
  // velocity to apply a bounded physical recovery torque. This is enabled
  // only for Stand/Walk and does not teleport or overwrite body state.
  if ((impl_->script == 0 || impl_->script == 1) && !impl_->torso.IsInvalid()) {
    JPH::Vec3 correctiveTorque = JPH::Vec3::sZero();
    {
      JPH::BodyLockRead torsoLock(impl_->physics->GetBodyLockInterface(), impl_->torso);
      if (torsoLock.Succeeded()) {
        const JPH::Body &body = torsoLock.GetBody();
        const JPH::Quat q = body.GetRotation();
        const float roll = std::atan2(2.0f * (q.GetW()*q.GetX() + q.GetY()*q.GetZ()),
                                      1.0f - 2.0f * (q.GetX()*q.GetX() + q.GetY()*q.GetY()));
        const float pitch = std::asin(std::clamp(2.0f * (q.GetW()*q.GetY() - q.GetZ()*q.GetX()), -1.0f, 1.0f));
        const JPH::Vec3 angularVelocity = body.GetAngularVelocity();
        correctiveTorque = JPH::Vec3(
            std::clamp(-pitch * 180.0f - angularVelocity.GetX() * 22.0f, -90.0f, 90.0f),
            0.0f,
            std::clamp(-roll * 180.0f - angularVelocity.GetZ() * 22.0f, -90.0f, 90.0f));
      }
    }
    if (correctiveTorque.LengthSq() > 0.001f)
      impl_->physics->GetBodyInterface().AddTorque(impl_->torso, correctiveTorque);
  }

  // Safety guard for the experimental crawl drive: never let an accumulated
  // contact impulse turn the robot into a projectile.
  if (impl_->script == 1 && !impl_->torso.IsInvalid()) {
    JPH::Vec3 safeVelocity = JPH::Vec3::sZero();
    bool limitVelocity = false;
    {
      JPH::BodyLockRead torsoLock(impl_->physics->GetBodyLockInterface(), impl_->torso);
      if (torsoLock.Succeeded()) {
        safeVelocity = torsoLock.GetBody().GetLinearVelocity();
        const float horizontalSpeed = std::sqrt(safeVelocity.GetX() * safeVelocity.GetX() + safeVelocity.GetZ() * safeVelocity.GetZ());
        if (horizontalSpeed > 1.25f) {
          const float factor = 1.25f / horizontalSpeed;
          safeVelocity.SetX(safeVelocity.GetX() * factor);
          safeVelocity.SetZ(safeVelocity.GetZ() * factor);
          limitVelocity = true;
        }
        const float clampedY = std::clamp(safeVelocity.GetY(), -2.0f, 1.0f);
        if (clampedY != safeVelocity.GetY()) {
          safeVelocity.SetY(clampedY);
          limitVelocity = true;
        }
      }
    }
    if (limitVelocity)
      impl_->physics->GetBodyInterface().SetLinearVelocity(impl_->torso, safeVelocity);
  }

  // Read the actual Jolt hinge state and estimate motor demand from the
  // remaining position error. This is diagnostic telemetry, not a claim of
  // measured electrical current or exact constraint torque.
  for (size_t i = 0; i < impl_->rotaryActuators.size() && i < kRobotActuatorCount; ++i) {
    const size_t leg = i / 4u, joint = i % 4u;
    const float actual = impl_->rotaryActuators[i]->GetCurrentAngle();
    const float target = impl_->telemetry.targetAnglesRad[leg][joint];
    const float error = std::remainder(target - actual, 2.0f * JPH::JPH_PI);
    const float limit = impl_->telemetry.torqueLimitsNm[joint];
    const float demand = std::min(limit, std::abs(error) * limit / 0.50f);
    impl_->telemetry.measuredAnglesRad[leg][joint] = actual;
    impl_->telemetry.angleErrorRad[leg][joint] = error;
    impl_->telemetry.estimatedTorqueDemandNm[i] = demand;
    impl_->telemetry.torqueSaturated[i] = std::abs(error) >= 0.50f;
  }

  const auto &lockInterface = impl_->physics->GetBodyLockInterface();
  for (SceneObject &object : scene.objects()) {
    const auto it = impl_->bodies.find(object.id);
    if (it == impl_->bodies.end()) continue;
    JPH::BodyLockRead lock(lockInterface, it->second);
    if (!lock.Succeeded()) continue;
    const JPH::RVec3 position = lock.GetBody().GetPosition();
    if (object.name == "Torso")
      impl_->telemetry.torsoSpeedMps = lock.GetBody().GetLinearVelocity().Length();
    object.transform.position = {
        static_cast<float>(position.GetX()),
        static_cast<float>(position.GetY()),
        static_cast<float>(position.GetZ())};
    const JPH::Quat rotation = lock.GetBody().GetRotation();
    const float x = rotation.GetX(), y = rotation.GetY(), z = rotation.GetZ(), w = rotation.GetW();
    object.transform.rotation = {
        std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)),
        std::asin(std::clamp(2.0f * (w * y - z * x), -1.0f, 1.0f)),
        std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z))};
  }
}

void JoltBridge::shutdown() {
  if (!impl_ || !impl_->ready) return;
  auto &bodies = impl_->physics->GetBodyInterface();
  for (const auto &actuator : impl_->actuators) impl_->physics->RemoveConstraint(actuator);
  impl_->actuators.clear();
  for (const auto &[id, body] : impl_->bodies) {
    bodies.RemoveBody(body);
    bodies.DestroyBody(body);
  }
  impl_->bodies.clear();
  if (!impl_->ground.IsInvalid()) {
    bodies.RemoveBody(impl_->ground);
    bodies.DestroyBody(impl_->ground);
    impl_->ground = JPH::BodyID();
  }
  impl_->jobs.reset();
  impl_->allocator.reset();
  // PhysicsSystem owns Jolt shapes and must go away while Factory is valid.
  impl_->physics.reset();
  JPH::UnregisterTypes();
  delete JPH::Factory::sInstance;
  JPH::Factory::sInstance = nullptr;
  impl_->ready = false;
}

void JoltBridge::setStandingTuning(const StandingTuning& tuning) {
  if (!impl_) return;
  impl_->tuning.motorFrequencyHz = std::clamp(tuning.motorFrequencyHz, 1.0f, 8.0f);
  impl_->tuning.motorDamping = std::clamp(tuning.motorDamping, 0.5f, 5.0f);
  impl_->tuning.comGain = std::clamp(tuning.comGain, 0.05f, 0.80f);
  impl_->tuning.velocityGain = std::clamp(tuning.velocityGain, 0.0f, 0.30f);
}

void JoltBridge::setRobotScript(int script) {
  if (!impl_ || !impl_->ready) return;
  impl_->script = std::clamp(script, 0, 3);
  impl_->scriptTime = 0.0f;
  impl_->settleTime = 0.0f;
  impl_->motorBlend = 0.0f;
  impl_->motorsEnabled = false;
  impl_->equilibriumPolicy.reset(script == 0);
  for (const auto &actuator : impl_->rotaryActuators)
    actuator->SetMotorState(JPH::EMotorState::Off);
  auto &bodies = impl_->physics->GetBodyInterface();
  for (const auto &[id, body] : impl_->bodies) bodies.ActivateBody(body);
}

int JoltBridge::robotScript() const {
  return impl_ ? impl_->script : 0;
}

const RobotTelemetry& JoltBridge::telemetry() const {
  static const RobotTelemetry empty{};
  return impl_ ? impl_->telemetry : empty;
}

bool JoltBridge::initialized() const {
  return impl_ && impl_->ready;
}
