#pragma once
#include <cmath>
struct Mat4 { float v[16]{}; };
inline Mat4 identity4(){Mat4 m{};m.v[0]=m.v[5]=m.v[10]=m.v[15]=1;return m;}
inline Mat4 translate4(float x,float y,float z){Mat4 m=identity4();m.v[12]=x;m.v[13]=y;m.v[14]=z;return m;}
inline Mat4 scale4(float x,float y,float z){Mat4 m{};m.v[0]=x;m.v[5]=y;m.v[10]=z;m.v[15]=1;return m;}
inline Mat4 multiply4(const Mat4&a,const Mat4&b){Mat4 r{};for(int c=0;c<4;c++)for(int row=0;row<4;row++)for(int k=0;k<4;k++)r.v[c*4+row]+=a.v[k*4+row]*b.v[c*4+k];return r;}
inline Mat4 perspective4(float fovy,float aspect,float nearZ,float farZ){Mat4 m{};float f=1/std::tan(fovy*.5f);m.v[0]=f/aspect;m.v[5]=f;m.v[10]=farZ/(nearZ-farZ);m.v[11]=-1;m.v[14]=(farZ*nearZ)/(nearZ-farZ);return m;}