#include <iostream>
#include "dependency/include/yaml.h"
#include <array>

 
int main(int argc,char** argv)
{
    YAML::Node config = YAML::LoadFile("./config.yaml");
 
    std::cout << "name:" << config["name"].as<std::string>() << std::endl;
    std::cout << "sex:" << config["sex"].as<std::string>() << std::endl;
    std::cout << "age:" << config["age"].as<int>() << std::endl;

    std::array<double, 6> a;
    YAML::Node yaml_a = config["a"];
    std::cout << "a的大小为: "<<yaml_a.size()<<std::endl;
    for (std::size_t i = 0; i < yaml_a.size(); ++i) {
        a[i] = yaml_a[i].as<double>();
    }
    std::cout << "a: " ;
    for(int i=0; i<a.size(); i++)
    {
        std::cout<< a[i] << ", " ;
    }
    std::cout<< std::endl;
    return 0;
}
