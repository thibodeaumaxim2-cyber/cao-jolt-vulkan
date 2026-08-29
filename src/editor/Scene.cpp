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
  part("Torso", {0,2.45f,0}, {1.55f,0.48f,0.72f});
  for(int side : {-1,1}) for(int end : {-1,1}){
    const char *front=end<0?"Front":"Rear"; const char *lr=side<0?"Left":"Right";
    const float x=.64f*side,z=.30f*end;
    part(std::string(front)+" "+lr+" Hip", {x,2.00f,z}, {.26f,.78f,.26f});
    part(std::string(front)+" "+lr+" Shin", {x,1.15f,z}, {.22f,.75f,.22f});
    part(std::string(front)+" "+lr+" Foot", {x,.48f,z+.12f*end}, {.34f,.18f,.46f});
  }
}
