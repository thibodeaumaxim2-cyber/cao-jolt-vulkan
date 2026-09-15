#include "ShaderLoader.hpp"
#include <fstream>
#include <stdexcept>
#include <vector>
VkShaderModule loadShaderModule(VkDevice device,const std::string& path){
 std::ifstream file(path,std::ios::ate|std::ios::binary);
 if(!file) throw std::runtime_error("Cannot open SPIR-V shader: "+path+
                                    ". Set CAO_SHADER_DIR or install the shader assets.");
 size_t size=(size_t)file.tellg(); if(size%4) throw std::runtime_error("SPIR-V size is not aligned: "+path);
 if(size < sizeof(uint32_t)) throw std::runtime_error("SPIR-V shader is empty or truncated: "+path);
 std::vector<char> bytes(size); file.seekg(0); file.read(bytes.data(),(std::streamsize)size);
 if(!file) throw std::runtime_error("Cannot read complete SPIR-V shader: "+path);
 const auto *words = reinterpret_cast<const uint32_t *>(bytes.data());
 if(words[0] != 0x07230203u)
   throw std::runtime_error("Shader is not valid SPIR-V: "+path);
 VkShaderModuleCreateInfo ci{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO,nullptr,0,size,words};
 VkShaderModule module{}; if(vkCreateShaderModule(device,&ci,nullptr,&module)!=VK_SUCCESS) throw std::runtime_error("Cannot create shader module: "+path);
 return module;
}
