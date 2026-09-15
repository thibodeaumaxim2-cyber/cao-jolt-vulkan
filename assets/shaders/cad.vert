#version 450
layout(location=0) in vec3 inPosition;
layout(location=1) in vec3 inColor;
layout(location=0) out vec3 color;
layout(location=1) out vec3 surfacePosition;
layout(push_constant) uniform Object { mat4 mvp; vec4 tint; } object;
void main(){
  color=inColor*object.tint.rgb;
  surfacePosition=inPosition;
  gl_Position=object.mvp*vec4(inPosition,1.0);
}
