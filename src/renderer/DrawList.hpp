#pragma once

#include "ObjectDraw.hpp"
#include <vector>

class DrawList {
 public:
  void clear() { objects_.clear(); }

  void add(const ObjectDrawData& object) {
    if (object.indexCount != 0)
      objects_.push_back(object);
  }

  const std::vector<ObjectDrawData>& objects() const { return objects_; }
  bool empty() const { return objects_.empty(); }
  std::size_t size() const { return objects_.size(); }

 private:
  std::vector<ObjectDrawData> objects_;
};
