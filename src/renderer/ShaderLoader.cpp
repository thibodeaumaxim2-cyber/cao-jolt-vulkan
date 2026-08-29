#include "ShaderLoader.hpp"
#include <fstream>
#include <stdexcept>
#include <vector>
VkShaderModule loadShaderModule(VkDevice device,const std::string& path){
 std::ifstream file(path,std::ios::ate|std::ios::binary);
 if(!file) throw std::runtime_error("Cannot open SPIR-V shader: "+path);
 size_t size=(size_t)file.tellg(); if(size%4) throw std::runtime_error("SPIR-V size is not aligned: "+path);
 std::vector<char> bytes(size); file.seekg(0); file.read(bytes.data(),(std::streamsize)size);
 VkShaderModuleCreateInfo ci{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO,nullptr,0,size,reinterpret_cast<const uint32_t*>(bytes.data())};
 VkShaderModule module{}; if(vkCreateShaderModule(device,&ci,nullptr,&module)!=VK_SUCCESS) throw std::runtime_error("Cannot create shader module: "+path);
 return module;
}