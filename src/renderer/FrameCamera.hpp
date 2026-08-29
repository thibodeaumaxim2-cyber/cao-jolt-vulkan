#pragma once

#include "Matrix.hpp"
#include "editor/Camera.hpp"

struct CameraFrame {
  Mat4 view;
  Mat4 projection;
};

inline Mat4 lookAt4(Vec3 eye, Vec3 target, Vec3 upDirection) {
  const Vec3 forward = normalize(target - eye);
  const Vec3 right = normalize(cross(forward, upDirection));
  const Vec3 up = cross(right, forward);

  Mat4 result{};
  result.v[0] = right.x;
  result.v[1] = up.x;
  result.v[2] = -forward.x;
  result.v[4] = right.y;
  result.v[5] = up.y;
  result.v[6] = -forward.y;
  result.v[8] = right.z;
  result.v[9] = up.z;
  result.v[10] = -forward.z;
  result.v[12] = -dot(right, eye);
  result.v[13] = -dot(up, eye);
  result.v[14] = dot(forward, eye);
  result.v[15] = 1.0f;
  return result;
}

inline CameraFrame makeCameraFrame(const OrbitCamera& camera,
                                   float aspect,
                                   float nearPlane = 0.1f,
                                   float farPlane = 500.0f) {
  CameraFrame frame;
  frame.view = lookAt4(camera.position(), camera.target, {0.0f, 1.0f, 0.0f});
  frame.projection = perspective4(1.0471976f, aspect, nearPlane, farPlane);
  return frame;
}
