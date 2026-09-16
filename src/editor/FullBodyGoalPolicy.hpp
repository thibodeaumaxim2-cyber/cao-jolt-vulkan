#pragma once
#include <array>
#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>

class FullBodyGoalPolicy {
  using Matrix = std::vector<std::vector<float>>;
  Matrix first_, second_; std::vector<float> firstBias_, secondBias_;
 public:
  void load(const std::string &path) {
    std::ifstream in(path); if (!in) throw std::runtime_error("Missing full-body goal policy: " + path);
    nlohmann::json j; in >> j;
    if (j.value("format", "") != "cao_fullbody_goal_mlp_v1") throw std::runtime_error("Unsupported full-body goal policy");
    first_=j.at("layer1_weight").get<Matrix>(); firstBias_=j.at("layer1_bias").get<std::vector<float>>();
    second_=j.at("layer2_weight").get<Matrix>(); secondBias_=j.at("layer2_bias").get<std::vector<float>>();
    if(first_.size()!=96 || firstBias_.size()!=96 || second_.size()!=19 || secondBias_.size()!=19) throw std::runtime_error("Invalid full-body goal policy dimensions");
    for(const auto &r:first_) if(r.size()!=42) throw std::runtime_error("Invalid full-body policy input width");
    for(const auto &r:second_) if(r.size()!=96) throw std::runtime_error("Invalid full-body policy hidden width");
  }
  std::array<float,19> infer(const std::array<float,42> &input) const {
    std::array<float,96> h{}; std::array<float,19> result{};
    for(size_t i=0;i<h.size();++i) { h[i]=firstBias_[i]; for(size_t k=0;k<input.size();++k) h[i]+=first_[i][k]*input[k]; h[i]=std::tanh(h[i]); }
    for(size_t i=0;i<result.size();++i) { result[i]=secondBias_[i]; for(size_t k=0;k<h.size();++k) result[i]+=second_[i][k]*h[k]; result[i]=std::tanh(result[i]); }
    return result;
  }
};
