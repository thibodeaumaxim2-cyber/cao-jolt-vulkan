#include "JoltBridge.hpp"
#include "JoltLayers.hpp"

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
  std::array<JPH::BodyID, 4> feet{};
  std::array<JPH::BodyID, 4> shins{};
  RobotTelemetry telemetry;
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
        JPH::Quat::sIdentity(), motion,
        object.dynamic ? CaoObjectLayers::Dynamic : CaoObjectLayers::Static);
    // Feet are the contact pads. High tangential traction limits sliding
    // while the torso and links remain free to move.
    settings.mFriction = object.name.find("Foot") != std::string::npos ? 1.35f : 0.58f;
    settings.mRestitution = 0.0f;
    // SI mass model for the 20 kg quadruped: 12 kg torso, 1.2 kg thighs,
    // 0.8 kg shins and 0.25 kg feet. Jolt calculates matching inertia.
    float massKg = 1.0f;
    if (object.name == "Torso") massKg = 12.0f;
    else if (object.name.find("Hip Roll") != std::string::npos) massKg = 0.30f;
    else if (object.name.find("Hip") != std::string::npos) massKg = 1.20f;
    else if (object.name.find("Shin") != std::string::npos) massKg = 0.80f;
    else if (object.name.find("Foot") != std::string::npos) massKg = 0.25f;
    if (object.dynamic) {
      settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
      settings.mMassPropertiesOverride.mMass = massKg;
    }
    const JPH::BodyID body = bodies.CreateAndAddBody(
        settings, object.dynamic ? JPH::EActivation::Activate
                                 : JPH::EActivation::DontActivate);
    impl_->bodies.emplace(object.id, body);
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
      settings.mMotorSettings.mSpringSettings.mFrequency = 10.0f;
      settings.mMotorSettings.mSpringSettings.mDamping = 1.0f;
      JPH::Ref<JPH::HingeConstraint> actuator = new JPH::HingeConstraint(
          *parentBody, *childBody, settings);
      actuator->SetMotorState(JPH::EMotorState::Position);
      actuator->SetTargetAngle(targetAngle); // radians
      impl_->physics->AddConstraint(actuator);
      impl_->rotaryActuators.emplace_back(actuator);
      impl_->actuators.emplace_back(std::move(actuator));
    };
    size_t leg = 0;
    for (int side : {-1, 1}) for (int end : {-1, 1}) {
      const std::string prefix = std::string(end < 0 ? "Front" : "Rear") +
          " " + (side < 0 ? "Left" : "Right");
      const float x = 0.64f * side, z = 0.30f * end;
      impl_->feet[leg] = findBody(prefix + " Foot");
      impl_->shins[leg] = findBody(prefix + " Shin");
      // 4 revolute actuators per leg: hip roll, hip pitch, knee pitch, ankle pitch.
      addHinge("Torso", prefix + " Hip Roll", x, 2.22f, z, JPH::Vec3::sAxisZ(),
               -0.35f, 0.35f, 0.0f, 55.0f);
      addHinge(prefix + " Hip Roll", prefix + " Hip", x, 2.11f, z, JPH::Vec3::sAxisX(),
               -0.75f, 0.75f, 0.0f, 85.0f);
      addHinge(prefix + " Hip", prefix + " Shin", x, 1.41f, z, JPH::Vec3::sAxisX(),
               -1.30f, 0.15f, -0.35f, 75.0f);
      addHinge(prefix + " Shin", prefix + " Foot", x, 0.70f, z, JPH::Vec3::sAxisX(),
               -0.55f, 0.55f, 0.0f, 35.0f);
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
  impl_->telemetry = {};
  impl_->telemetry.motionScript = impl_->script;
  impl_->telemetry.linkCount = static_cast<int>(scene.objects().size());
  impl_->telemetry.torqueLimitsNm = {{55.0f, 85.0f, 75.0f, 35.0f}};
  const float phase = impl_->scriptTime * (impl_->script == 3 ? 7.0f : 4.4f);
  const auto setLegPose = [&](size_t leg, float roll, float hip, float knee, float ankle) {
    const size_t first = leg * 4u;
    impl_->rotaryActuators[first]->SetTargetAngle(roll);
    impl_->rotaryActuators[first + 1u]->SetTargetAngle(hip);
    impl_->rotaryActuators[first + 2u]->SetTargetAngle(knee);
    impl_->rotaryActuators[first + 3u]->SetTargetAngle(ankle);
    impl_->telemetry.targetAnglesRad[leg] = {{roll, hip, knee, ankle}};
  };
  if (impl_->script == 0) { // Stand
    for (size_t leg = 0; leg < 4; ++leg) setLegPose(leg, 0.0f, 0.0f, -0.48f, 0.0f);
  } else if (impl_->script == 1 || impl_->script == 2) { // Walk / trot
    // Each cycle is an explicit: stance -> unload -> lift/swing -> place.
    // The offsets produce a four-beat walk or diagonal-pair trot.
    constexpr std::array<float, 4> walkOffsets{0.0f, 0.25f, 0.50f, 0.75f};
    constexpr std::array<float, 4> trotOffsets{0.0f, 0.50f, 0.50f, 0.0f};
    const auto &offsets = impl_->script == 1 ? walkOffsets : trotOffsets;
    const float cycleDuration = impl_->script == 1 ? 2.40f : 1.05f;
    auto &bodyInterface = impl_->physics->GetBodyInterface();
    for (size_t leg = 0; leg < 4; ++leg) {
      const float cycle = std::fmod(impl_->scriptTime / cycleDuration + offsets[leg], 1.0f);
      const float side = leg < 2 ? -1.0f : 1.0f;
      float roll = 0.0f, hip = 0.0f, knee = -0.48f, ankle = 0.0f;
      float swingLiftForceN = 0.0f;
      int state = 0;
      bool planted = cycle < 0.66f || cycle >= 0.96f;
      if (cycle < (impl_->script == 1 ? 0.72f : 0.58f)) { // crawl stance: three legs support the body
        const float t = cycle / (impl_->script == 1 ? 0.72f : 0.58f);
        hip = 0.18f - 0.42f * t;
        ankle = -0.10f * hip;
      } else if (cycle < (impl_->script == 1 ? 0.78f : 0.66f)) { // unload before the single-leg swing
        state = 1;
        const float t = (cycle - (impl_->script == 1 ? 0.72f : 0.58f)) / 0.06f;
        hip = -0.24f + 0.05f * t;
        roll = side * 0.12f;
      } else if (cycle < (impl_->script == 1 ? 0.98f : 0.96f)) { // lift and swing exactly one leg
        state = 2;
        const float t = (cycle - (impl_->script == 1 ? 0.78f : 0.66f)) / (impl_->script == 1 ? 0.20f : 0.30f);
        const float lift = std::sin(JPH::JPH_PI * t);
        hip = -0.19f + 0.43f * t;
        knee = -0.48f - 0.62f * lift;
        ankle = 0.20f * lift;
        roll = side * 0.10f * (1.0f - lift);
        // Equal-and-opposite internal actuator force assists the rotary knee.
        // It has no net external force. A bounded PD term prevents energy
        // accumulation once the foot has cleared the floor.
        if (!impl_->feet[leg].IsInvalid()) {
          JPH::BodyLockRead footLock(impl_->physics->GetBodyLockInterface(), impl_->feet[leg]);
          if (footLock.Succeeded()) {
            const JPH::Body &footBody = footLock.GetBody();
            const float footY = static_cast<float>(footBody.GetPosition().GetY());
            const float footVy = footBody.GetLinearVelocity().GetY();
            if (footY < 0.20f && footVy < 0.45f)
              swingLiftForceN = std::clamp((0.20f - footY) * 42.0f - footVy * 3.5f, 0.0f, 8.0f) * lift;
          }
        }
        planted = false;
      } else { // place: extend the knee before high traction returns
        state = 3;
        const float t = (cycle - (impl_->script == 1 ? 0.98f : 0.96f)) / (impl_->script == 1 ? 0.02f : 0.04f);
        hip = 0.24f - 0.06f * t;
        knee = -0.48f - 0.18f * (1.0f - t);
        ankle = 0.05f * (1.0f - t);
      }
      setLegPose(leg, roll, hip, knee, ankle);
      impl_->telemetry.legState[leg] = state;
      impl_->telemetry.footFriction[leg] = planted ? 1.35f : 0.08f;
      impl_->telemetry.liftAssistForceN[leg] = swingLiftForceN;
      if (state == 2) {
        impl_->telemetry.activeSwingLeg = static_cast<int>(leg);
        impl_->telemetry.swingLiftForceN = swingLiftForceN;
      }
      if (!impl_->feet[leg].IsInvalid())
        bodyInterface.SetFriction(impl_->feet[leg], planted ? 1.35f : 0.08f);
      if (swingLiftForceN > 0.0f && !impl_->feet[leg].IsInvalid() &&
          !impl_->shins[leg].IsInvalid()) {
        const JPH::Vec3 liftForce(0.0f, swingLiftForceN, 0.0f);
        bodyInterface.AddForce(impl_->feet[leg], liftForce);
        bodyInterface.AddForce(impl_->shins[leg], -liftForce);
      }
    }
  } else if (impl_->script == 3) { // Repeated jump
    const float extension = std::max(0.0f, std::sin(phase));
    for (size_t leg = 0; leg < 4; ++leg)
      setLegPose(leg, 0.0f, 0.0f, -0.85f + 0.70f * extension, 0.0f);
  }
  impl_->telemetry.gaitCycle = impl_->script == 1 || impl_->script == 2
      ? std::fmod(impl_->scriptTime / (impl_->script == 1 ? 2.40f : 1.05f), 1.0f) : 0.0f;
  impl_->physics->Update(seconds, 2, impl_->allocator.get(), impl_->jobs.get());

  // Read the actual Jolt hinge state and estimate motor demand from the
  // remaining position error. This is diagnostic telemetry, not a claim of
  // measured electrical current or exact constraint torque.
  for (size_t i = 0; i < impl_->rotaryActuators.size() && i < 16; ++i) {
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

void JoltBridge::setRobotScript(int script) {
  if (!impl_ || !impl_->ready) return;
  impl_->script = std::clamp(script, 0, 3);
  impl_->scriptTime = 0.0f;
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
