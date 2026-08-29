#version 450

layout(location = 0) out vec3 color;

vec2 positions[3] = vec2[](
  vec2(0.0, -0.65),
  vec2(0.65, 0.65),
  vec2(-0.65, 0.65)
);

void main() {
  gl_Position = vec4(positions[gl_VertexIndex], 0.0, 1.0);
  color = vec3(0.15 + 0.2 * gl_VertexIndex,
               0.75 - 0.1 * gl_VertexIndex,
               1.0);
}
