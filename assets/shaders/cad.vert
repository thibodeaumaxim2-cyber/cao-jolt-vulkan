#version 450
layout(location=0) in vec3 inPosition;
layout(location=1) in vec3 inColor;
layout(location=0) out vec3 color;
layout(set=0,binding=0) uniform Camera { mat4 viewProj; } camera;
layout(push_constant) uniform Object { mat4 model; vec4 tint; } object;
void main(){ color=inColor*object.tint.rgb; gl_Position=camera.viewProj*object.model*vec4(inPosition,1.0); }
