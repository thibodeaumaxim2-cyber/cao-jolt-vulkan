#pragma once
#include <array>
#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <vector>

// Native implementation of the Unitree H1 PolicyExporterLSTM architecture.
// Deployment reference: unitreerobotics/unitree_rl_gym, BSD-3-Clause.
// See assets/unitree_h1/LICENSE and README.md for license and pinned revision.
// PyTorch gate order i,f,g,o; ELU actor.
class UnitreePolicy {
  using Matrix = std::vector<std::vector<float>>;
  Matrix wi, wh, w0, w2;
  std::vector<float> bi, bh, b0, b2;
  std::array<float,64> h{}, c{};
public:
  void load(const std::string &path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("Missing Unitree H1 policy weights: " + path);
    nlohmann::json j; file >> j;
    wi=j.at("memory.weight_ih_l0").get<Matrix>();
    wh=j.at("memory.weight_hh_l0").get<Matrix>();
    bi=j.at("memory.bias_ih_l0").get<std::vector<float>>();
    bh=j.at("memory.bias_hh_l0").get<std::vector<float>>();
    w0=j.at("actor.0.weight").get<Matrix>(); b0=j.at("actor.0.bias").get<std::vector<float>>();
    w2=j.at("actor.2.weight").get<Matrix>(); b2=j.at("actor.2.bias").get<std::vector<float>>();
    const auto check=[](const Matrix &m, size_t rows,size_t cols) {
      if(m.size()!=rows) throw std::runtime_error("Invalid policy rows");
      for(const auto &r:m) if(r.size()!=cols) throw std::runtime_error("Invalid policy columns");
    };
    check(wi,256,41); check(wh,256,64); check(w0,32,64); check(w2,10,32);
    if(bi.size()!=256 || bh.size()!=256 || b0.size()!=32 || b2.size()!=10)
      throw std::runtime_error("Invalid policy biases");
    reset();
  }
  void reset() { h.fill(0); c.fill(0); }
  std::array<float,10> infer(const std::array<float,41> &x) {
    std::array<float,256> gates{};
    for(size_t i=0;i<256;++i) {
      gates[i]=bi[i]+bh[i];
      for(size_t k=0;k<41;++k) gates[i]+=wi[i][k]*x[k];
      for(size_t k=0;k<64;++k) gates[i]+=wh[i][k]*h[k];
    }
    const auto sigmoid=[](float v){return 1.0f/(1.0f+std::exp(-v));};
    for(size_t i=0;i<64;++i) {
      c[i]=sigmoid(gates[i+64])*c[i]+sigmoid(gates[i])*std::tanh(gates[i+128]);
      h[i]=sigmoid(gates[i+192])*std::tanh(c[i]);
    }
    std::array<float,32> a{};
    for(size_t i=0;i<32;++i) {
      a[i]=b0[i]; for(size_t k=0;k<64;++k) a[i]+=w0[i][k]*h[k];
      if(a[i]<0) a[i]=std::expm1(a[i]);
    }
    std::array<float,10> y{};
    for(size_t i=0;i<10;++i) {
      y[i]=b2[i]; for(size_t k=0;k<32;++k) y[i]+=w2[i][k]*a[k];
      if(!std::isfinite(y[i])) throw std::runtime_error("Nonfinite Unitree policy action");
    }
    return y;
  }
};
