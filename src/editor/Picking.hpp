#pragma once
#include "Scene.hpp"
#include <optional>
struct Ray { Vec3 origin,direction; };
std::optional<uint32_t> pickObject(const Scene&,Ray);
