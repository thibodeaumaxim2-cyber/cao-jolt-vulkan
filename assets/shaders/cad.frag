#version 450
layout(location=0) in vec3 color;
layout(location=1) in vec3 surfacePosition;
layout(location=0) out vec4 outColor;
void main(){
  // Flat geometry-derived normals keep imported STL links readable without
  // requiring a separate normal vertex stream.
  vec3 normal=normalize(cross(dFdx(surfacePosition), dFdy(surfacePosition)));
  vec3 lightDirection=normalize(vec3(-0.45, 0.70, 0.55));
  float diffuse=max(dot(normal, lightDirection), 0.0);
  float rim=pow(1.0-max(abs(normal.z), 0.0), 2.0)*0.18;
  vec3 illuminated=color*(0.34+0.76*diffuse+rim);
  outColor=vec4(illuminated/(illuminated+vec3(0.55)),1.0);
}
