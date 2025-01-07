#pragma once

#include <cstddef>
#include <fmt/include/fmt/std.h>
#include "conf/parser.cpp"
#include <fstream>
#include <ios>
#include <map>
#include <stdexcept>
#include <string>
#include <boost/json.hpp>
#include <Eigen/Geometry> 
#include "helpers.cpp"

namespace World {
namespace Field {

    // Storage container for individual Tag info
    struct TagLocation {
        int id = 0; // int
        double x = 0; // cm
        double y = 0; // cm
        double z = 0; // cm 
        double yaw = 0; // deg
    }; // struct TagLocation

    
    double xWidth = 1000; // cm, default 10mx10m 
    double yHeight = 1000; // cm
    
    std::map<int, TagLocation> tags;

    void TOMLReadFieldConfig(toml::table fieldConfigRoot) {
        fmt::println("Parsing field config");
        if (Conf::parse<bool>(fieldConfigRoot, "field.use_json_file", false)) {
            auto jsonPath = Conf::parse<std::string>(fieldConfigRoot, "field.game_apriltag_json_file"); 
            fmt::println("Opening JSON Field File at {}", jsonPath);

            std::ifstream jsonFile {jsonPath, std::ios_base::in};
            auto json = boost::json::parse(jsonFile, {}, {
                .numbers = boost::json::number_precision::precise,
                .allow_comments = true,
                .allow_trailing_commas = true,
            });

            auto field = json.at("field").as_object();
            xWidth = field.at("length").as_double() * 100;
            yHeight = field.at("width").as_double() * 100;

            auto tagsArr = json.at("tags").as_array();
            for (auto& i : tagsArr) {
                auto translationObj = i.at("pose").at("translation");

                auto quaternionObj = i.at("pose").at("rotation").at("quaternion"); 
                Eigen::Quaternion<double> quaternion {
                    quaternionObj.at("W").as_double(),
                    quaternionObj.at("X").as_double(),
                    quaternionObj.at("Y").as_double(),
                    quaternionObj.at("Z").as_double(),
                };

                auto id = static_cast<int>(i.at("ID").as_int64());
                tags.emplace(id, TagLocation {
                    .id = id,
                    .x = translationObj.at("x").as_double() * 100,
                    .y = translationObj.at("y").as_double() * 100,
                    .z = translationObj.at("z").as_double() * 100,
                    .yaw = h::NormalizeAngle(h::rad2deg(quaternion.normalized().toRotationMatrix().eulerAngles(0, 1, 2)[2]))
                });
            }

            jsonFile.close();
        };

        toml::table overridesTab;
        auto tab = fieldConfigRoot.get("override");
        if (tab != nullptr && tab->is_table()) {
            overridesTab = *tab->as_table();
        } else throw std::runtime_error("`override` is not a table");

        for (auto& [k, v] : overridesTab) {
            auto id = std::stoi(std::string(k.str())); 
            if (!v.is_table()) throw std::runtime_error(fmt::format("{} is not a table", id)); 
            auto tab = *v.as_table();

            if (Conf::parse<bool>(tab, "enabled", false)) {
                tags.insert_or_assign(id, TagLocation {
                    .id = id,
                    .x = Conf::parse<double>(tab, "x"),
                    .y = Conf::parse<double>(tab, "y"),
                    .z = Conf::parse<double>(tab, "z"),
                    .yaw = Conf::parse<double>(tab, "r")
                });
            } else {
                tags.erase(id);
            }
        }

        fmt::println("Loaded {:.4f}cm X {:.4f}cm Field with:", xWidth, yHeight);
        for (auto& [k, v] : tags) {
            fmt::println("Tag {}, X: {:.4f}, Y:{:.4f}, Z:{:.4f}, R: {:.4f}", k, v.x, v.y, v.z, v.yaw);
        }
    }

}; // namespace Field
} // namespace World

