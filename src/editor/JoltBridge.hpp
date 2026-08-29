#pragma once

#include "Scene.hpp"
#include <memory>

class JoltBridge {
 public:
  JoltBridge();
  ~JoltBridge();
  JoltBridge(const JoltBridge&) = delete;
  JoltBridge& operator=(const JoltBridge&) = delete;

  void initialize();
  void rebuild(Scene&);
  void step(Scene&, float seconds);
  void demolish(const Scene&);
  void shutdown();
  bool initialized() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
