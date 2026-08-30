#include "Scene.hpp"
#include <algorithm>
SceneObject& Scene::add(Primitive p,const Transform&t){std::string name=[&]{switch(p){case Primitive::Box:return "Box";case Primitive::Cylinder:return "Cylinder";case Primitive::Sphere:return "Sphere";default:return "Beam";}}();objects_.push_back({nextId_++,p,name+" "+std::to_string(nextId_-1),t});return objects_.back();}
void Scene::erase(uint32_t id){objects_.erase(std::remove_if(objects_.begin(),objects_.end(),[&](auto&o){return o.id==id;}),objects_.end());}
SceneObject* Scene::find(uint32_t id){for(auto&o:objects_)if(o.id==id)return&o;return nullptr;}
void Scene::clear(){objects_.clear();nextId_=1;quadruped_=false;}
void Scene::buildPyramid(int levels,bool dynamic){clear();levels=std::clamp(levels,2,12);for(int y=0;y<levels;y++)for(int x=0;x<levels-y;x++)for(int z=0;z<levels-y;z++){Transform t;t.position={x-(levels-y-1)*.5f,.5f+y,z-(levels-y-1)*.5f};auto&o=add(Primitive::Box,t);o.dynamic=dynamic;}}

void Scene::buildQuadruped(){
  clear(); quadruped_=true;
  auto part=[&](const std::string &name,const Vec3&p,const Vec3&s){Transform t;t.position=p;t.scale=s;auto&o=add(Primitive::Box,t);o.name=name;o.dynamic=true;};
  // Keep the spawn pose consistent with LegGeometry/JoltBridge:
  // straight 1.41 m legs, foot soles at y=0, and a lower COM.
  part("Torso", {0,2.11f,0}, {1.55f,0.48f,0.72f});
  for(int side : {-1,1}) for(int end : {-1,1}){
    const char *front=end<0?"Front":"Rear"; const char *lr=side<0?"Left":"Right";
    const std::string prefix=std::string(front)+" "+lr;
    const float x=.64f*side,z=.30f*end;
    // Four rotary links per leg: roll carrier, thigh, shin, and foot.
    part(prefix+" Hip Roll", {x,2.22f,z}, {.22f,.26f,.32f});
    part(prefix+" Hip", {x,1.76f,z}, {.28f,.70f,.28f});
    part(prefix+" Shin", {x,1.06f,z}, {.24f,.70f,.24f});
    // The 0.18 m high foot is centered at 0.09 m: its sole rests on ground.
    part(prefix+" Foot", {x,.09f,z+.12f*end}, {.36f,.18f,.48f});
  }
}