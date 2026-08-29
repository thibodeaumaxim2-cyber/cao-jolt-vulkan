#pragma once
#include <cstdint>
#include "Matrix.hpp"
struct CameraUniform { Mat4 viewProj; };
struct ObjectPushConstant { Mat4 model; float tint[4]{1.f,1.f,1.f,1.f}; };
struct DepthState { bool enabled=true; bool write=true; int compareLessOrEqual=1; };
struct IndexedDraw { uint32_t indexCount=36; uint32_t instanceCount=1; uint32_t firstIndex=0; int32_t vertexOffset=0; uint32_t firstInstance=0; };