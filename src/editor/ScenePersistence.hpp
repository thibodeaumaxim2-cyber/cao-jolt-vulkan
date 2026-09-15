#pragma once

#include "Scene.hpp"
#include "SceneFileFormat.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>
#include <utility>

inline const char *primitiveName(Primitive primitive) {
  switch (primitive) {
    case Primitive::Box: return "box";
    case Primitive::Cylinder: return "cylinder";
    case Primitive::Sphere: return "sphere";
    case Primitive::Beam: return "beam";
  }
  return "box";
}

inline Primitive primitiveFromName(const std::string &name) {
  if (name == "cylinder") return Primitive::Cylinder;
  if (name == "sphere") return Primitive::Sphere;
  if (name == "beam") return Primitive::Beam;
  if (name == "box") return Primitive::Box;
  throw std::runtime_error("Unknown CAO primitive: " + name);
}

inline nlohmann::json sceneToJson(const Scene &scene) {
  nlohmann::json objects = nlohmann::json::array();
  for (const auto &object : scene.objects()) {
    objects.push_back({
      {"id", object.id}, {"primitive", primitiveName(object.primitive)},
      {"name", object.name}, {"dynamic", object.dynamic},
      {"position", {object.transform.position.x, object.transform.position.y, object.transform.position.z}},
      {"rotation", {object.transform.rotation.x, object.transform.rotation.y, object.transform.rotation.z}},
      {"scale", {object.transform.scale.x, object.transform.scale.y, object.transform.scale.z}}
    });
  }
  return {{"version", kCaoSceneFormatVersion}, {"objects", objects}};
}

inline void saveScene(const Scene &scene, const std::string &path) {
  std::ofstream out(path);
  if (!out) throw std::runtime_error("Cannot write CAO scene file");
  out << sceneToJson(scene).dump(2) << '\n';
  if (!out) throw std::runtime_error("Cannot finish writing CAO scene file");
}

inline void sceneFromJson(Scene &scene, const nlohmann::json &document) {
  const unsigned version = document.value("version", 0u);
  if (version != kCaoSceneFormatVersion)
    throw std::runtime_error("Unsupported CAO scene version: " + std::to_string(version));
  if (!document.contains("objects") || !document["objects"].is_array())
    throw std::runtime_error("CAO scene is missing its objects array");

  // Build into a temporary document first. If any object is invalid, the
  // caller keeps its current scene instead of being left partially loaded.
  Scene loaded;
  for (const auto &value : document["objects"]) {
    Transform transform;
    auto readVec3 = [&](const char *key, Vec3 &target) {
      if (!value.contains(key) || !value[key].is_array() || value[key].size() != 3)
        throw std::runtime_error(std::string("CAO object has invalid ") + key);
      target = {value[key][0].get<float>(), value[key][1].get<float>(), value[key][2].get<float>()};
    };
    readVec3("position", transform.position);
    readVec3("rotation", transform.rotation);
    readVec3("scale", transform.scale);
    const auto id = value.value("id", 0u);
    auto &object = loaded.addWithId(id, primitiveFromName(value.value("primitive", "box")), transform);
    object.name = value.value("name", object.name);
    object.dynamic = value.value("dynamic", true);
  }
  scene = std::move(loaded);
}

inline void loadScene(Scene &scene, const std::string &path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("Cannot open CAO scene file: " + path);
  nlohmann::json document;
  try {
    in >> document;
  } catch (const nlohmann::json::exception &error) {
    throw std::runtime_error("Invalid CAO scene JSON: " + std::string(error.what()));
  }
  sceneFromJson(scene, document);
}
