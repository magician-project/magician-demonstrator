#include "yaml-cpp/yaml.h"
#include <iostream>
#include <string>
#include <vector>



struct RobotConfig{

    std::string name;
    std::string joint_states;
    std::string service_name;
    std::vector<double> home_vec;
};

struct CobotConfig
{

    RobotConfig cleaning_group;
    RobotConfig sensing_group;
};


class ConfigLoader{

    public:
    CobotConfig static load_file(const std::string& yaml_path);


};

