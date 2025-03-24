#pragma once
#include "add_furni_gui/furni-yaml.h"
#include "add_furni_gui/OpenFile.h"
#include "add_furni_gui/FurniWithName.h"

#include <ros/ros.h>

#include <vector>
#include <string>

typedef std::vector<add_furni_gui::FurniWithName> FurniList;

namespace add_furni_gui
{
    class AddFurniNode {
    public:
        AddFurniNode(ros::NodeHandle& nh);
        ~AddFurniNode();
    private:
        ros::NodeHandle& nh_;
        ros::ServiceServer server_;

        std::string fpath_;
        FurniList furni_list_;

    private:
        void readFurni(std::string fpath);
        bool handle_open_file_req(OpenFile::Request& req, OpenFile::Response& res);
    };
} // end namespace add_furni_gui
