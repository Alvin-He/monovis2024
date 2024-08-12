
#include <iostream>
#include <fmt/include/fmt/ranges.h>
#include <fmt/include/fmt/core.h>
#include <fmt/include/fmt/std.h>

#include <tomlplusplus/toml.hpp>

#include "conf/serviceConfigDef.cpp"

typedef std::string str; 

struct ConfigOptions {
    //Comms
    str Comms_NetworkTable_server;
    
    //Camera
    size_t Camera_id = 0; 
    str Camera_calibrationFile; 

    //Apriltag
    bool Apriltag_enabled = true;
    str Apriltag_csvFile; 


    //PoseEstimation
    bool PoseEstimation_enabled = true;

}; 

template <typename T> T parse(const toml::table& configTable, const std::string& path, std::optional<T> _default = std::nullopt) {
    try {

    if (_default.has_value()) {
        return configTable.at_path(path).value<T>().template value_or<T>(std::move(_default.value()));
    }
    return configTable.at_path(path).value<T>().value(); 
    
    } catch(const std::bad_optional_access& e) {
        std::cerr << "ERROR: " + path + " is required, but is not set in config file!" << std::endl; 
        throw e;
    }
}

int main(int argc, char* argv[]) {

    toml::table config; 

    str firstArg = argv[1]; 
    
    config = toml::parse_file(firstArg); 

    
    std::cout << config << "\n"; 
    // ConfigOptions opt {
        // .Apriltag_enabled = parse<bool>(config, "Apriltag.enabled", true),
        // .Apriltag_csvFile = parse<str>(config, "Apriltag.csvFile")
        // 
    // }; 
    
    ServiceConfig conf =ParseServiceConfig(config); 
    
    fmt::println("{}", conf.Comms_NetworkTable_server);
    fmt::println("{}", conf.Apriltag_enabled);
    fmt::println("{}", conf.Apriltag_worldSizeX);
    fmt::println("{}", conf.Apriltag_worldSizeY);
    fmt::println("{}", conf.PoseEstimation_enabled);
    fmt::println("{}", conf.Cameras_configFiles);

    return 0; 
}
