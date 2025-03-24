#pragma once
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <geometry_msgs/Point.h>
#include "add_furni_gui/Furni.h"

namespace YAML {
    template<>
    struct convert<geometry_msgs::Point> 
    {
        static Node encode(const geometry_msgs::Point& rhs) {
            Node node;
            node.push_back(rhs.x);
            node.push_back(rhs.y);
            node.push_back(rhs.z);
            return node;
        }
        static bool decode(const Node& node, geometry_msgs::Point& rhs) {
            if (!node.IsSequence() || node.size() > 3) {
                return false;
            }
            rhs.x = node[0].as<double>();
            rhs.y = node[1].as<double>();
            rhs.z = (node.size() == 3) ? node[2].as<double>() : 0.0;
            return true;
        }
    };

    template<>
    struct convert<add_furni_gui::Furni> {
        static Node encode(const add_furni_gui::Furni& rhs) {
            Node node;
            node["belong"] = rhs.belong;
            node["frame"] = rhs.frame;
            node["plane"] = rhs.plane;

            Node access;
            for (size_t i=0;i<(rhs.access.size()-1);i+=2) {
                Node set;
                set.push_back(rhs.access[i]);
                set.push_back(rhs.access[i+1]);
                access.push_back(set);
            }
            node["access"] = access;

            return node;
        }
        
        static bool decode(const Node& node, add_furni_gui::Furni& rhs) {
            if (!node.IsMap() || 
                !node["belong"] || !node["frame"] || 
                !node["plane"] || !node["access"]) {
                return false;
            }

            rhs.belong = node["belong"].as<std::string>();
            rhs.frame = node["frame"].as<std::string>();

            // Clear previous data (to avoid appending when decoding multiple times)
            rhs.plane.clear();
            rhs.access.clear();

            for (const auto& p : node["plane"]) {
                rhs.plane.push_back(p.as<geometry_msgs::Point>());
            }

            for (const auto& set : node["access"]) {
                if (set.size() != 2) {
                    return false; // Each set must have exactly 2 points
                }
                rhs.access.push_back(set[0].as<geometry_msgs::Point>());
                rhs.access.push_back(set[1].as<geometry_msgs::Point>());
            }

            return true;
        }
    };
}// end namespace YAML