#include "add_furni_gui/AddFurniNode.h"

int main (int argc, char** argv) {
    ros::init(argc, argv, "add_furni_gui_node");
    ros::NodeHandle nh;
    add_furni_gui::AddFurniNode add_furni_node(nh);
    ros::spin();
    return 0;
}