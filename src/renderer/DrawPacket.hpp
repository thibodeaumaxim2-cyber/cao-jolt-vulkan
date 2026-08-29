#pragma once
#include "editor/Scene.hpp"
#include "Matrix.hpp"
struct DrawPacket { Primitive primitive; Mat4 model; bool selected=false; };
inline DrawPacket makeDrawPacket(const SceneObject&o,uint32_t selected){
 Mat4 model=multiply4(translate4(o.transform.position.x,o.transform.position.y,o.transform.position.z),scale4(o.transform.scale.x,o.transform.scale.y,o.transform.scale.z));
 return {o.primitive,model,o.id==selected};
}