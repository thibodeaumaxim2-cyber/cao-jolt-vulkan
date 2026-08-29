#include "Scene.hpp"
#include <algorithm>
SceneObject& Scene::add(Primitive p,const Transform&t){std::string name=[&]{switch(p){case Primitive::Box:return "Box";case Primitive::Cylinder:return "Cylinder";case Primitive::Sphere:return "Sphere";default:return "Beam";}}();objects_.push_back({nextId_++,p,name+" "+std::to_string(nextId_-1),t});return objects_.back();}
void Scene::erase(uint32_t id){objects_.erase(std::remove_if(objects_.begin(),objects_.end(),[&](auto&o){return o.id==id;}),objects_.end());}
SceneObject* Scene::find(uint32_t id){for(auto&o:objects_)if(o.id==id)return&o;return nullptr;}
void Scene::clear(){objects_.clear();nextId_=1;}
void Scene::buildPyramid(int levels,bool dynamic){clear();levels=std::clamp(levels,2,12);for(int y=0;y<levels;y++)for(int x=0;x<levels-y;x++)for(int z=0;z<levels-y;z++){Transform t;t.position={x-(levels-y-1)*.5f,.5f+y,z-(levels-y-1)*.5f};auto&o=add(Primitive::Box,t);o.dynamic=dynamic;}}
