#include "JoltBridge.hpp"

#include <Jolt/Core/Factory.h>
#include <Jolt/RegisterTypes.h>

struct JoltBridge::Impl {
  bool ready = false;
};

JoltBridge::JoltBridge() : impl_(std::make_unique<Impl>()) {}
JoltBridge::~JoltBridge() { shutdown(); }

void JoltBridge::initialize() {
  if (impl_->ready) return;
  JPH::Factory::sInstance = new JPH::Factory();
  JPH::RegisterTypes();
  impl_->ready = true;
}

void JoltBridge::rebuild(Scene&) {
  // Body creation is performed during the next integration step; the bridge
  // deliberately keeps physics independent from the Vulkan renderer.
}

void JoltBridge::step(Scene&, float) {
  // PhysicsSystem update is added with the native scene body registry.
}

void JoltBridge::shutdown() {
  if (!impl_ || !impl_->ready) return;
  JPH::UnregisterTypes();
  delete JPH::Factory::sInstance;
  JPH::Factory::sInstance = nullptr;
  impl_->ready = false;
}

bool JoltBridge::initialized() const {
  return impl_ && impl_->ready;
}
