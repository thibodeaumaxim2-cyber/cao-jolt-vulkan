#pragma once
#include "Scene.hpp"
class JoltBridge {
 public:
  void initialize();
  void rebuild(Scene&);
  void step(Scene&,float seconds);
  void shutdown();
};
