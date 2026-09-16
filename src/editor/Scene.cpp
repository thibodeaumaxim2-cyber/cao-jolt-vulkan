#include "Scene.hpp"
#include <algorithm>
#include <array>
#include <stdexcept>
SceneObject& Scene::add(Primitive p,const Transform&t){std::string name=[&]{switch(p){case Primitive::Box:return "Box";case Primitive::Cylinder:return "Cylinder";case Primitive::Sphere:return "Sphere";default:return "Beam";}}();objects_.push_back({nextId_++,p,name+" "+std::to_string(nextId_-1),t});return objects_.back();}
SceneObject& Scene::addWithId(uint32_t id, Primitive p, const Transform &t){if(id==0||find(id)!=nullptr)throw std::invalid_argument("Scene object ID is invalid or already used");std::string name=[&]{switch(p){case Primitive::Box:return "Box";case Primitive::Cylinder:return "Cylinder";case Primitive::Sphere:return "Sphere";default:return "Beam";}}();objects_.push_back({id,p,name+" "+std::to_string(id),t});nextId_=std::max(nextId_,id+1);return objects_.back();}
void Scene::erase(uint32_t id){objects_.erase(std::remove_if(objects_.begin(),objects_.end(),[&](auto&o){return o.id==id;}),objects_.end());}
SceneObject* Scene::find(uint32_t id){for(auto&o:objects_)if(o.id==id)return&o;return nullptr;}
void Scene::clear(){objects_.clear();nextId_=1;quadruped_=false;biped_=false;unitreeH1_=false;}
void Scene::buildPyramid(int levels,bool dynamic){clear();levels=std::clamp(levels,2,12);for(int y=0;y<levels;y++)for(int x=0;x<levels-y;x++)for(int z=0;z<levels-y;z++){Transform t;t.position={x-(levels-y-1)*.5f,.5f+y,z-(levels-y-1)*.5f};auto&o=add(Primitive::Box,t);o.dynamic=dynamic;}}

void Scene::buildQuadruped(){
  clear(); quadruped_=true;
  auto part=[&](const std::string &name,const Vec3&p,const Vec3&s){Transform t;t.position=p;t.scale=s;auto&o=add(Primitive::Box,t);o.name=name;o.dynamic=true;};
  // Cat-like hexapod pose: compact paws at floor level and long lower legs.
  part("Torso", {0,.925f,0}, {1.55f,.28f,1.45f});
  for(int side : {-1,1}) for(int station : {-1,0,1}){
    const char *position=station<0?"Front":station>0?"Rear":"Middle"; const char *lr=side<0?"Left":"Right";
    const std::string prefix=std::string(position)+" "+lr;
    const float x=.64f*side,z=.62f*station;
    // Four rotary links per leg: roll carrier, thigh, shin, and foot.
    part(prefix+" Hip Roll", {x,.98f,z}, {.22f,.14f,.32f});
    part(prefix+" Hip", {x,.7625f,z}, {.28f,.275f,.28f});
    part(prefix+" Shin", {x,.345f,z}, {.24f,.51f,.24f});
    // Low-profile paw: short and narrow while retaining a flat floor contact.
    part(prefix+" Foot", {x,.045f,z}, {.34f,.09f,.42f});
  }
}

void Scene::buildBiped(){
  clear(); biped_=true;
  auto part=[&](const std::string &name,const Vec3&p,const Vec3&s){Transform t;t.position=p;t.scale=s;auto&o=add(Primitive::Box,t);o.name=name;o.dynamic=true;};
  // Broad, light torso and deliberately long feet: this is a stable starting
  // point for future single-support biped balance control.
  part("Torso", {0,.925f,0}, {.84f,.28f,.80f});
  for(int side : {-1,1}) {
    const char *lr=side<0?"Left":"Right";
    const std::string prefix=std::string("Front ")+lr;
    const float x=.44f*side;
    part(prefix+" Hip Roll", {x,.98f,0}, {.22f,.14f,.32f});
    part(prefix+" Hip", {x,.7625f,0}, {.28f,.275f,.28f});
    part(prefix+" Shin", {x,.345f,0}, {.24f,.51f,.24f});
    part(prefix+" Foot", {x,.045f,0}, {.42f,.09f,.92f});
  }
}

void Scene::buildUnitreeH1(){
  clear(); unitreeH1_=true;
  auto part=[&](const char *name, const Vec3 &scale) { Transform t; t.position={0, 1.1f, 0}; t.scale=scale; auto &o=add(Primitive::Box,t); o.name=name; o.dynamic=true; };
  // These are renderer proxies. Their transforms are synchronized from the
  // original Unitree H1 MuJoCo bodies; mesh rendering remains a future Vulkan
  // import task.
  part("pelvis", {.38f,.22f,.22f});
  part("left_hip_yaw_link", {.12f,.12f,.12f}); part("left_hip_roll_link", {.12f,.12f,.12f});
  part("left_hip_pitch_link", {.14f,.30f,.14f}); part("left_knee_link", {.12f,.30f,.12f}); part("left_ankle_link", {.18f,.08f,.32f});
  part("right_hip_yaw_link", {.12f,.12f,.12f}); part("right_hip_roll_link", {.12f,.12f,.12f});
  part("right_hip_pitch_link", {.14f,.30f,.14f}); part("right_knee_link", {.12f,.30f,.12f}); part("right_ankle_link", {.18f,.08f,.32f});
  part("torso_link", {.34f,.42f,.22f});
  part("left_shoulder_pitch_link", {.12f,.14f,.12f}); part("left_shoulder_roll_link", {.12f,.14f,.12f});
  part("left_shoulder_yaw_link", {.12f,.16f,.12f}); part("left_elbow_link_ball_hand", {.10f,.24f,.10f});
  part("right_shoulder_pitch_link", {.12f,.14f,.12f}); part("right_shoulder_roll_link", {.12f,.14f,.12f});
  part("right_shoulder_yaw_link", {.12f,.16f,.12f}); part("right_elbow_link_ball_hand", {.10f,.24f,.10f});
  // Physical obstacle test: three staggered rows, ahead of H1 on its
  // forward +X axis. Names match bodies in assets/unitree_h1/scene.xml.
  for (int row=0; row<3; ++row) {
    const int count=3-row;
    for (int column=0; column<count; ++column) {
      Transform t; t.position={2.4f+0.38f*row, 0.20f+0.38f*row,
          (column-(count-1)*0.5f)*0.38f}; t.scale={.18f,.18f,.18f};
      auto &o=add(Primitive::Box,t);
      o.name="pyramid_box_"+std::to_string(row)+"_"+std::to_string(column); o.dynamic=true;
    }
  }
  const std::array<Vec3, 10> obstaclePositions{{
      {1.45f,.38f,-.65f},{1.45f,.38f,-.22f},{1.45f,.38f,.22f},
      {3.65f,.38f,0},{3.65f,.38f,.44f},{3.65f,.38f,.88f},
      {4.85f,.38f,-.88f},{4.85f,.38f,-.44f},{4.85f,.38f,0},{5.70f,.38f,.45f}}};
  for (size_t i=0;i<obstaclePositions.size();++i) {
    Transform t; t.position=obstaclePositions[i]; t.scale={.20f,.38f,.20f};
    auto &o=add(Primitive::Box,t); o.name="nav_obstacle_"+std::to_string(i); o.dynamic=false;
  }
  Transform target; target.position={6.2f,.012f,0}; target.scale={.36f,.012f,.36f};
  auto &goal=add(Primitive::Cylinder,target); goal.name="goal_circle"; goal.dynamic=false;
}
