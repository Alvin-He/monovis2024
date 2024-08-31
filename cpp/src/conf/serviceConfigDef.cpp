
#pragma once 


#include "parser.cpp"

namespace Conf {
struct ServiceConfig {
    //Comms
    std::string Comms_NetworkTable_server; 
    uint Comms_NetworkTable_port; 

    //Apriltag
    bool Apriltag_enabled; 
    std::string Apriltag_csvFile; // csv file describing where the apriltags are in the world 
    uint Apriltag_worldSizeX;
    uint Apriltag_worldSizeY;

    //PoseEstimation
    bool PoseEstimation_enabled;

    //Cameras
    std::vector<std::string> Cameras_configFiles; // a list of paths pointing to camera config files 
}; 

ServiceConfig ParseServiceConfig(const toml::table& configTable) {
    return ServiceConfig {
        .Comms_NetworkTable_server = Conf::parse<std::string>(configTable, "Comms.NetworkTable.server"), 
        .Comms_NetworkTable_port = Conf::parse<uint>(configTable, "Comms.NetworkTable.port"),
        .Apriltag_enabled = Conf::parse<bool>(configTable, "Apriltag.enabled"),
        .Apriltag_csvFile = Conf::parse<std::string>(configTable, "Apriltag.csvFile"), 
        .Apriltag_worldSizeX = Conf::parse<uint>(configTable, "Apriltag.worldSizeX"), 
        .Apriltag_worldSizeY = Conf::parse<uint>(configTable, "Apriltag.worldSizeY"), 
        .PoseEstimation_enabled = Conf::parse<bool>(configTable, "PoseEstimation.enabled"), 
        .Cameras_configFiles = Conf::ParseArray<std::string>(configTable, "Cameras.configFiles")
    };
}
}