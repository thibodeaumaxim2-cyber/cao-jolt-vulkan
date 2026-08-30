#pragma once

#include "Scene.hpp"
#include <filesystem>

bool exportRobotParameters(const Scene &scene, int motionScript,
                           const std::filesystem::path &path);
