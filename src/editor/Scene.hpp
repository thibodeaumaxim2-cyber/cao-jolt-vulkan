#pragma once
#include "Camera.hpp"
#include <cstdint>
#include <string>
#include <vector>

enum class Primitive { Box, Cylinder, Sphere, Beam };
struct Transform { Vec3 position{0,1,0}; Vec3 rotation{0,0,0}; Vec3 scale{1,1,1}; };
struct SceneObject { uint32_t id=0; Primitive primitive=Primitive::Box; std::string name; Transform transform; bool dynamic=true; uint32_t joltBody=0; };

class Scene {
 public:
  SceneObject& add(Primitive p,const Transform&t={});
  void erase(uint32_t id);
  SceneObject* find(uint32_t id);
  void buildPyramid(int levels,bool dynamic=true);
  void clear();
  const std::vector<SceneObject>& objects()const{return objects_;}
  std::vector<SceneObject>& objects(){return objects_;}
 private: uint32_t nextId_=1; std::vector<SceneObject> objects_;
};
