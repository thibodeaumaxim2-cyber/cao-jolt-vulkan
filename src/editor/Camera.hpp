#pragma once
#include <algorithm>
#include <cmath>

struct Vec3 { float x=0,y=0,z=0; };
inline Vec3 operator+(Vec3 a,Vec3 b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
inline Vec3 operator-(Vec3 a,Vec3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
inline Vec3 operator*(Vec3 a,float s){return {a.x*s,a.y*s,a.z*s};}
inline float dot(Vec3 a,Vec3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
inline Vec3 cross(Vec3 a,Vec3 b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
inline Vec3 normalize(Vec3 a){float n=std::sqrt(dot(a,a));return n>0?a*(1.f/n):Vec3{};}

class OrbitCamera {
 public:
  Vec3 target{0,2,0}; float yaw=.72f,pitch=.52f,distance=18.f;
  void orbit(float dx,float dy){yaw+=dx*.008f;pitch=std::clamp(pitch+dy*.008f,-1.48f,1.48f);}
  void zoom(float delta){distance=std::clamp(distance*(1.f-delta*.1f),1.5f,100.f);}
  void pan(float dx,float dy){Vec3 r=right(),u=up();target=target-r*(dx*.01f*distance)+u*(dy*.01f*distance);}
  Vec3 position()const{return target+Vec3{std::cos(pitch)*std::sin(yaw),std::sin(pitch),std::cos(pitch)*std::cos(yaw)}*distance;}
  Vec3 forward()const{return normalize(target-position());}
  Vec3 right()const{return normalize(cross(forward(),{0,1,0}));}
  Vec3 up()const{return normalize(cross(right(),forward()));}
};
