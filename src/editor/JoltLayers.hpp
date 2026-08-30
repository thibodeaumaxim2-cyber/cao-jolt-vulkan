#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>

namespace CaoObjectLayers {
static constexpr JPH::ObjectLayer Static = 0;
static constexpr JPH::ObjectLayer Dynamic = 1;
static constexpr JPH::ObjectLayer RobotLink = 2;
static constexpr uint32_t Count = 3;
}

namespace CaoBroadPhaseLayers {
static const JPH::BroadPhaseLayer Static{0};
static const JPH::BroadPhaseLayer Dynamic{1};
static constexpr uint32_t Count = 2;
}

class CaoObjectLayerPairFilter final : public JPH::ObjectLayerPairFilter {
 public:
  bool ShouldCollide(JPH::ObjectLayer first, JPH::ObjectLayer second) const override {
    if (first == CaoObjectLayers::RobotLink && second == CaoObjectLayers::RobotLink)
      return false;
    return first != CaoObjectLayers::Static || second != CaoObjectLayers::Static;
  }
};

class CaoBroadPhaseLayerInterface final : public JPH::BroadPhaseLayerInterface {
 public:
  uint GetNumBroadPhaseLayers() const override { return CaoBroadPhaseLayers::Count; }
  JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override {
    return layer == CaoObjectLayers::Static ? CaoBroadPhaseLayers::Static : CaoBroadPhaseLayers::Dynamic;
  }
};

class CaoObjectVsBroadPhaseFilter final : public JPH::ObjectVsBroadPhaseLayerFilter {
 public:
  bool ShouldCollide(JPH::ObjectLayer layer, JPH::BroadPhaseLayer broadPhase) const override {
    return layer == CaoObjectLayers::Dynamic || broadPhase == CaoBroadPhaseLayers::Dynamic;
  }
};
