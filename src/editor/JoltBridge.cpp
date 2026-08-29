#include "JoltBridge.hpp"
#include "JoltLayers.hpp"

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/JobSystem/JobSystemThreadPool.h>

#include <algorithm>
#include <thread>

struct JoltBridge::Impl {
  bool ready = false;
  CaoBroadPhaseLayerInterface broadPhaseLayers;
  CaoObjectVsBroadPhaseFilter objectVsBroadPhase;
  CaoObjectLayerPairFilter objectPairs;
  JPH::PhysicsSystem physics;
  std::unique_ptr<JPH::TempAllocatorImpl> allocator;
  std::unique_ptr<JPH::JobSystemThreadPool> jobs;
};

JoltBridge::JoltBridge() : impl_(std::make_unique<Impl>()) {}
JoltBridge::~JoltBridge() { shutdown(); }

void JoltBridge::initialize() {
  if (impl_->ready) return;
  JPH::RegisterDefaultAllocator();
  JPH::Factory::sInstance = new JPH::Factory();
  JPH::RegisterTypes();

  impl_->physics.Init(2048, 0, 4096, 4096,
                      impl_->broadPhaseLayers, impl_->objectVsBroadPhase,
                      impl_->objectPairs);
  impl_->allocator = std::make_unique<JPH::TempAllocatorImpl>(10 * 1024 * 1024);
  const unsigned int workers = std::max(1u, std::thread::hardware_concurrency() > 1
      ? std::thread::hardware_concurrency() - 1 : 1u);
  impl_->jobs = std::make_unique<JPH::JobSystemThreadPool>(
      JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, workers);
  impl_->ready = true;
}

void JoltBridge::rebuild(Scene&) {
  // Dynamic and static body creation is the next bridge step.
}

void JoltBridge::step(Scene&, float seconds) {
  if (!impl_->ready || seconds <= 0.0f) return;
  impl_->physics.Update(seconds, 1, impl_->allocator.get(), impl_->jobs.get());
}

void JoltBridge::shutdown() {
  if (!impl_ || !impl_->ready) return;
  impl_->jobs.reset();
  impl_->allocator.reset();
  JPH::UnregisterTypes();
  delete JPH::Factory::sInstance;
  JPH::Factory::sInstance = nullptr;
  impl_->ready = false;
}

bool JoltBridge::initialized() const {
  return impl_ && impl_->ready;
}
