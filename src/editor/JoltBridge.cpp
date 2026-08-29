#include "JoltBridge.hpp"
#include "JoltLayers.hpp"

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#include <algorithm>
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
  JPH::BodyID ground;
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
  for (const auto &[id, body] : impl_->bodies) {
    bodies.RemoveBody(body);
    bodies.DestroyBody(body);
  }
  impl_->bodies.clear();

  if (!impl_->ground.IsInvalid()) {
    bodies.RemoveBody(impl_->ground);
    bodies.DestroyBody(impl_->ground);
  }

  JPH::BoxShapeSettings groundShape(JPH::Vec3(50.0f, 0.25f, 50.0f));
  groundShape.SetEmbedded();
  JPH::BodyCreationSettings groundSettings(
      &groundShape, JPH::RVec3(0.0, -0.25, 0.0), JPH::Quat::sIdentity(),
      JPH::EMotionType::Static, CaoObjectLayers::Static);
  groundSettings.mFriction = 0.38f;
  groundSettings.mRestitution = 0.62f;
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
    settings.mFriction = 0.30f;
    settings.mRestitution = 0.68f;
    const JPH::BodyID body = bodies.CreateAndAddBody(
        settings, object.dynamic ? JPH::EActivation::Activate
                                 : JPH::EActivation::DontActivate);
    impl_->bodies.emplace(object.id, body);
    object.joltBody = body.GetIndexAndSequenceNumber();
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
  impl_->physics->Update(seconds, 1, impl_->allocator.get(), impl_->jobs.get());

  const auto &lockInterface = impl_->physics->GetBodyLockInterface();
  for (SceneObject &object : scene.objects()) {
    const auto it = impl_->bodies.find(object.id);
    if (it == impl_->bodies.end()) continue;
    JPH::BodyLockRead lock(lockInterface, it->second);
    if (!lock.Succeeded()) continue;
    const JPH::RVec3 position = lock.GetBody().GetPosition();
    object.transform.position = {
        static_cast<float>(position.GetX()),
        static_cast<float>(position.GetY()),
        static_cast<float>(position.GetZ())};
  }
}

void JoltBridge::shutdown() {
  if (!impl_ || !impl_->ready) return;
  auto &bodies = impl_->physics->GetBodyInterface();
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

bool JoltBridge::initialized() const {
  return impl_ && impl_->ready;
}
