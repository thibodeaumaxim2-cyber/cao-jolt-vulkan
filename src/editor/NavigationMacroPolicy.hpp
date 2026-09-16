#pragma once

#include <array>
#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>

// Small locally trained macro policy. It turns an A* waypoint in the robot's
// body frame into velocity/yaw commands for Unitree's locomotion policy.
class NavigationMacroPolicy {
  using Matrix = std::vector<std::vector<float>>;
  Matrix first_, second_;
  std::vector<float> firstBias_, secondBias_;
 public:
  void load(const std::string &path) {
    std::ifstream stream(path);
    if (!stream) throw std::runtime_error("Missing navigation macro policy: " + path);
    nlohmann::json json; stream >> json;
    if (json.value("format", "") != "cao_navigation_macro_mlp_v1")
      throw std::runtime_error("Unsupported navigation macro policy format");
    first_ = json.at("layer1_weight").get<Matrix>(); firstBias_ = json.at("layer1_bias").get<std::vector<float>>();
    second_ = json.at("layer2_weight").get<Matrix>(); secondBias_ = json.at("layer2_bias").get<std::vector<float>>();
    if (first_.size() != 64 || second_.size() != 3 || firstBias_.size() != 64 || secondBias_.size() != 3)
      throw std::runtime_error("Invalid navigation macro policy dimensions");
    for (const auto &row : first_) if (row.size() != 4) throw std::runtime_error("Invalid navigation macro input width");
    for (const auto &row : second_) if (row.size() != 64) throw std::runtime_error("Invalid navigation macro hidden width");
  }
  std::array<float, 3> infer(const std::array<float, 4> &input) const {
    std::array<float, 64> hidden{};
    for (size_t i=0;i<hidden.size();++i) {
      hidden[i]=firstBias_[i]; for(size_t j=0;j<input.size();++j) hidden[i]+=first_[i][j]*input[j];
      hidden[i]=std::tanh(hidden[i]);
    }
    std::array<float, 3> result{};
    for(size_t i=0;i<result.size();++i) {
      result[i]=secondBias_[i]; for(size_t j=0;j<hidden.size();++j) result[i]+=second_[i][j]*hidden[j];
      result[i]=std::tanh(result[i]);
    }
    return result;
  }
};
