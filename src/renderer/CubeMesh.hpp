#pragma once
#include <array>
#include <cstdint>
#include "VulkanRenderer.hpp"
struct CubeMeshData {
 std::array<CadVertex,8> vertices{{
  {{-.5f,-.5f,-.5f},{.10f,.75f,.42f}}, {{.5f,-.5f,-.5f},{.12f,.82f,.50f}},
  {{.5f,.5f,-.5f},{.18f,.90f,.62f}}, {{-.5f,.5f,-.5f},{.10f,.70f,.38f}},
  {{-.5f,-.5f,.5f},{.08f,.62f,.34f}}, {{.5f,-.5f,.5f},{.12f,.78f,.44f}},
  {{.5f,.5f,.5f},{.25f,.95f,.70f}}, {{-.5f,.5f,.5f},{.16f,.84f,.52f}}
 }};
 std::array<uint32_t,36> indices{{0,1,2,2,3,0,4,6,5,6,4,7,0,4,5,5,1,0,3,2,6,6,7,3,1,5,6,6,2,1,0,3,7,7,4,0}};
};