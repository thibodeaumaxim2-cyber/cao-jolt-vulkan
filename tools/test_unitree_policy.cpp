#include "../src/editor/UnitreePolicy.hpp"
#include <iostream>
int main(int argc,char **argv) {
  if(argc!=2) return 2;
  std::string root=argv[1];
  UnitreePolicy policy; policy.load(root+"/weights.json");
  std::ifstream file(root+"/inference_test.json"); nlohmann::json tests; file>>tests;
  float error=0;
  for(const auto &test:tests) {
    auto result=policy.infer(test.at("observation").get<std::array<float,41>>());
    for(int i=0;i<10;++i) error=std::max(error,std::abs(result[i]-test.at("action").at(i).get<float>()));
  }
  std::cout<<"Maximum recurrent inference error: "<<error<<'\n';
  return error<1e-4f ? 0:1;
}
