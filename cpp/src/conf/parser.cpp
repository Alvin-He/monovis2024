
#pragma once 

#include <tomlplusplus/toml.hpp>
#include <iostream>
namespace Conf {

// toml::parse_file with error handling
toml::table LoadToml(std::string file, bool critical = true) {
    toml::table retVal; 
    try {
        retVal = toml::parse_file(file); 
    } catch (const toml::parse_error& err) {
        std::cerr
            << "Error parsing file '" << *err.source().path
            << "':\n" << err.description()
            << "\n (" << err.source().begin << ")\n";
        
        if (critical) throw err;
    }
    return retVal; 
}

// parse a toml item
template <typename T> T parse(const toml::table& tab, const std::string& path, std::optional<T> _default = std::nullopt) {
    try {

    if (_default.has_value()) {
        return tab.at_path(path).value<T>().template value_or<T>(std::move(_default.value()));
    }
    return tab.at_path(path).value<T>().value(); 
    
    } catch(const std::exception& e) {
        std::cerr << "ERROR: " + path + " is required, but is not set in config file!" << std::endl; 
        throw e;
    }
}

// parse a single type toml array into a vector 
template <typename T> std::vector<T> ParseArray(const toml::table& configTable, const std::string& path, std::optional<std::vector<T>> _default = std::nullopt) {
    try {
    
    std::vector<T> returnVec; 

    auto nodeAtPath = configTable.at_path(path); 
    if (!nodeAtPath.is_array()) {
        if (_default.has_value()) return _default.value(); 
        else throw std::exception();
    }
    toml::array arr = *nodeAtPath.as_array();

    for (auto&& elem : arr) {
        auto value = elem.value<T>(); 
        if (value.has_value()) {
            returnVec.push_back(value.value()); 
        }
    }

    if (returnVec.size() == 0) {
        if (_default.has_value()) return _default.value(); 
        else throw std::exception();
    }
    return returnVec;  
    
    } catch(const std::exception& e) {
        std::cerr << "ERROR: " + path + " is required, but is not set in config file!" << std::endl; 
        throw e;
    }
}

}