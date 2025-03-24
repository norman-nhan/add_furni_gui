#include "add_furni_gui/AddFurniNode.h"

#include <ros/param.h>
#include <ros/package.h>

#include <yaml-cpp/yaml.h>

#include <stdlib.h>

namespace add_furni_gui
{
AddFurniNode::AddFurniNode(ros::NodeHandle& nh): nh_(nh) {
    // Configuring ROS communication
    server_ = nh_.advertiseService("open_file", &AddFurniNode::handle_open_file_req, this);

    ROS_INFO("Initialized AddFurniNode completed!");
}

AddFurniNode::~AddFurniNode() = default;

void AddFurniNode::readFurni(std::string fpath) {
    YAML::Node node = YAML::LoadFile(fpath);
    YAML::Node furni_node = node["furniture"];
    if(!furni_node.IsNull()) {
        ROS_INFO("File read successfully!");
        for (YAML::const_iterator it=furni_node.begin();it!=furni_node.end();++it) {
            FurniWithName tmp;
            tmp.name = it->first.as<std::string>();
            tmp.furni = it->second.as<Furni>();
            furni_list_.emplace_back(tmp);
            ROS_INFO("Adding %s", tmp.name.c_str());
        }
    }
    else {
        ROS_ERROR("furniture does not exist");
    }
}

bool AddFurniNode::handle_open_file_req(OpenFile::Request& req, OpenFile::Response& res) {
    if (fpath_ != req.fpath) {
        fpath_ = req.fpath;
    }
    readFurni(fpath_);
    // send furni list to panel
    res.furni_list = furni_list_;

    return true;
}

} // end namespace add_furni_gui
