#include "JoltBridge.hpp"
// Physics-system ownership is intentionally isolated here. The renderer only
// reads SceneObject::transform; this keeps Jolt independent from Vulkan.
void JoltBridge::initialize(){}
void JoltBridge::rebuild(Scene&){}
void JoltBridge::step(Scene&,float){}
void JoltBridge::shutdown(){}
