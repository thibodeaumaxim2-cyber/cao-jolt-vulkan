#pragma once
#include "Camera.hpp"
#include "Scene.hpp"
#include "Picking.hpp"

enum class Tool { Select, Translate, Rotate, Scale };
class UiController {
 public:
  Tool tool=Tool::Select; int pyramidLevels=5; bool simulation=true; bool snap=false; uint32_t selected=0;
  void onKey(int key,int action,Scene& scene);
  void onMouseDrag(float dx,float dy,bool orbit,bool pan,OrbitCamera& camera);
  void selectFromRay(Scene&,Ray);
  void createBox(Scene&); void createSphere(Scene&); void createCylinder(Scene&); void createBeam(Scene&);
  void deleteSelection(Scene&);
};
