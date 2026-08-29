#include "UiController.hpp"
#include <algorithm>
void UiController::createBox(Scene&s){selected=s.add(Primitive::Box).id;}
void UiController::createSphere(Scene&s){selected=s.add(Primitive::Sphere).id;}
void UiController::createCylinder(Scene&s){selected=s.add(Primitive::Cylinder).id;}
void UiController::createBeam(Scene&s){Transform t;t.scale={3,.25f,.25f};selected=s.add(Primitive::Beam,t).id;}
void UiController::deleteSelection(Scene&s){if(selected){s.erase(selected);selected=0;}}
void UiController::selectFromRay(Scene&s,Ray r){selected=pickObject(s,r).value_or(0);}
void UiController::onMouseDrag(float dx,float dy,bool orbit,bool pan,OrbitCamera& c){if(orbit)c.orbit(dx,dy);if(pan)c.pan(dx,dy);}
void UiController::onKey(int key,int action,Scene&s){if(!action)return;switch(key){case '1':createBox(s);break;case '2':createCylinder(s);break;case '3':createSphere(s);break;case '4':createBeam(s);break;case 'P':s.buildPyramid(pyramidLevels,true);break;case ' ':simulation=!simulation;break;case 261:case 127:deleteSelection(s);break;case 'W':tool=Tool::Translate;break;case 'E':tool=Tool::Rotate;break;case 'R':tool=Tool::Scale;break;}}
