#pragma once

#include <cstdint>
#include <unordered_map>

class PhysicsObjectMap {
 public:
  void bind(uint32_t objectId, uint32_t bodyId) { map_[objectId] = bodyId; }
  void erase(uint32_t objectId) { map_.erase(objectId); }
  uint32_t bodyFor(uint32_t objectId) const {
    const auto it = map_.find(objectId);
    return it == map_.end() ? 0u : it->second;
  }
  void clear() { map_.clear(); }
 private:
  std::unordered_map<uint32_t, uint32_t> map_;
};
