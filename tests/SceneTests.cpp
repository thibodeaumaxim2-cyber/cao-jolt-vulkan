#include "editor/Scene.hpp"
#include <cassert>

int main() {
  Scene scene;

  auto &first = scene.add(Primitive::Box);
  const auto firstId = first.id;
  auto &second = scene.add(Primitive::Sphere);
  const auto secondId = second.id;
  assert(firstId != secondId);
  assert(scene.find(firstId) != nullptr);
  assert(scene.find(999999) == nullptr);

  scene.buildPyramid(1);
  assert(scene.objects().size() == 5); // 2^2 + 1; levels are clamped to 2
  assert(scene.objects().front().dynamic);

  scene.buildPyramid(20, false);
  assert(scene.objects().size() == 650); // 12^2 + 11^2 + ... + 1^2
  for (const auto &object : scene.objects())
    assert(!object.dynamic);

  scene.clear();
  assert(scene.objects().empty());
  auto &resetObject = scene.add(Primitive::Cylinder);
  assert(resetObject.id == 1); // clear starts a new document identity sequence
  return 0;
}
