#include "add_furni_gui/AddFurniPanel.h"

#include <QTabWidget>
#include <QLabel>
#include <QFileDialog>
#include <QDir>

#include <ros/package.h>

#include <yaml-cpp/yaml.h>
#include "add_furni_gui/furni-yaml.h"

#include <string>
#include <cmath>
#include <iostream>
#include <fstream>

using string = std::string;

namespace add_furni_gui
{
AddFurniPanel::AddFurniPanel(QWidget* parent): rviz::Panel(parent), nh_() {
    // LAYOUT //

    // Widget1: Input & Output Files //
    QWidget* wid1 = new QWidget();
    QHBoxLayout* layout1 = new QHBoxLayout;
    
    layout1->addWidget(open_button_);
    layout1->addWidget(save_button_);
    wid1->setLayout(layout1);
    // end widget1

    // Widget2: Add furnitures's properties //
    QWidget* wid2 = new QWidget;
    QVBoxLayout* layout2 = new QVBoxLayout;
    
    layout2->addLayout(createComboBox("Furniture name: ", name_box_));
    layout2->addLayout(createComboBox("Belong: ", belong_box_));

    QHBoxLayout* button = new QHBoxLayout;
    button->addWidget(record_button_);
    button->addWidget(delete_button_);
    layout2->addLayout(button);
    wid2->setLayout(layout2);
    // end widget2

    // Add widgets
    QTabWidget* tab_widget = new QTabWidget;
    tab_widget->addTab(wid1, "File");
    tab_widget->addTab(wid2, "Add Plane");
    
    // Main layout
    QVBoxLayout* main_layout = new QVBoxLayout;
    main_layout->addWidget(tab_widget);
    setLayout(main_layout);

    // CONNECT //
    connect(open_button_, SIGNAL(clicked()), this, SLOT(open_file()));
    connect(save_button_, SIGNAL(clicked()), this, SLOT(save_file()));
    connect(record_button_, SIGNAL(clicked()), this, SLOT(start_record()));
    connect(delete_button_, SIGNAL(clicked()), this, SLOT(delete_furni()));
}

AddFurniPanel::~AddFurniPanel() = default;

void AddFurniPanel::onInitialize() {
    // ROS //
    sub_ = nh_.subscribe("clicked_point", 1, &AddFurniPanel::addPoint, this);
    client_ = nh_.serviceClient<OpenFile>("open_file");
    pub_ = nh_.advertise<visualization_msgs::MarkerArray>("makers", 1);
}

double AddFurniPanel::round(double x) {
    return std::round(x * 10.0) / 10.0;
}

void AddFurniPanel::delete_furni() {
    std::string name = name_box_->currentText().toStdString(); // get delete furni name

    auto it = std::remove_if(furni_list_.begin(), furni_list_.end(),
        [&name](const FurniWithName& furni) {
            return furni.name == name;
        }
    );

    if (it != furni_list_.end()) {
        furni_list_.erase(it, furni_list_.end());
    }

    clearMarker();
    pubMarker();
}

void AddFurniPanel::start_record() {
    recording_ = true;
    points_.clear();
    clearMarker(plane_markers_, "plane");
    ROS_INFO("Start recording plane's points");

}

int AddFurniPanel::getFurniIndex(const std::string& name) {
    auto it = std::find_if(furni_list_.begin(), furni_list_.end(),
        [&name] (const FurniWithName& furni) {
            return furni.name == name;
        }
    );
    if (it != furni_list_.end()) {
        return std::distance(furni_list_.begin(), it);
    }
    return -1; // not found
}

void AddFurniPanel::addPoint(const geometry_msgs::PointStamped::ConstPtr& msg) {
    if (recording_ && points_.size() < 4) {
        geometry_msgs::Point p = msg->point;
        p.z = 0.0;
        p.x = AddFurniPanel::round(p.x);
        p.y = AddFurniPanel::round(p.y);
        points_.push_back(p);
        plane_markers_.markers.push_back(createSphere(p));
        pub_.publish(plane_markers_);
        if (points_.size() == 4) {
            recording_ = false;
            
            QString name = name_box_->currentText();
            
            FurniWithName furni;
            int idx = getFurniIndex(name.toStdString());
            
            if (idx != -1) { // furni is already existing
                furni_list_[idx].furni.plane.clear();
                furni_list_[idx].furni.plane = points_;
            } else {
                QString belong = belong_box_->currentText();
                addUnique(belong_box_, belong);
                addUnique(name_box_, name);
                
                furni.name = name.toStdString();
                furni.furni.belong = belong.toStdString();
                furni.furni.plane = points_;
                furni.furni.frame = "map";
                furni.furni.access.push_back(points_[0]);
                furni.furni.access.push_back(points_[1]);
    
                furni_list_.push_back(furni);
            }

            points_.clear();
            clearMarker();
            pubMarker();
        }
    }
}

QHBoxLayout* AddFurniPanel::createComboBox(QString label, QComboBox* box) {
    box->setEditable(true);
    box->setInsertPolicy(QComboBox::NoInsert);
    return createLayout(label, box);
}

QHBoxLayout* AddFurniPanel::createLayout(QString label, QWidget* wid) {
    QHBoxLayout* layout = new QHBoxLayout;
    layout->addWidget(new QLabel(label));
    layout->addWidget(wid);
    return layout;
}

void AddFurniPanel::addUnique(QComboBox* box, const QString& item) {
    if (box->findText(item) == -1) {
        box->addItem(item);
    }
}

void AddFurniPanel::open_file() {
    QString fpath = QFileDialog::getOpenFileName(nullptr, "Open File", QDir::homePath(), "YAML Files (*.yaml *.yml)");
    OpenFile openfile;
    if(!fpath.isEmpty()) {
        openfile.request.fpath = fpath.toStdString();
    } else {
        openfile.request.fpath = ros::package::getPath("add_furni_gui") + "/yaml/furniture.yaml";
    }
    if (client_.call(openfile)) {
        ROS_INFO("Opening file: %s", openfile.request.fpath.c_str());
        if (!openfile.response.furni_list.empty()) {
            ROS_INFO("Received furni list!");
            furni_list_.clear();
            name_box_->clear();
            belong_box_->clear();

            furni_list_ = openfile.response.furni_list;
            for (const auto& it : furni_list_) {
                addUnique(name_box_, QString::fromStdString(it.name));
                addUnique(belong_box_, QString::fromStdString(it.furni.belong));
            }
            // Visualization
            clearMarker();
            pubMarker();
        }
        else {
            ROS_INFO("Failed to receive furni list!");
        }
    }
    else {
        ROS_ERROR("Failed to call open file service!");
    }
}

int AddFurniPanel::indent(int x) {
    return x*4;
}

void AddFurniPanel::writeFile(const std::string& fo) {
    std::ofstream fout(fo, std::ios::trunc);
    fout << std::fixed << std::setprecision(1); // round up to 1 digit after decimal point
    fout << "furniture:" << std::endl;
    for (const auto& furni : furni_list_) {
        int lv=1;
        Furni info = furni.furni;
        fout << string(indent(lv), ' ') << furni.name << ":" << std::endl;
        lv=2;
        fout << string(indent(lv), ' ') << "frame: " << info.frame << std::endl;
        fout << string(indent(lv), ' ') << "belong: " << info.belong << std::endl;

        fout << string(indent(lv), ' ') << "plane: [" << std::endl;
        lv=3;
        for (const auto& p : info.plane) {
            fout << string(indent(lv), ' ') << "[" << p.x << ", " << p.y << "]," << std::endl;
        }
        lv=2;
        fout << string(indent(lv), ' ') << "]" << std::endl;

        fout << string(indent(lv), ' ') << "access: [" << std::endl;
        lv=3;
        fout << string(indent(lv), ' ') << "[";
        for (const auto& p : info.access) {
            fout << "[" << p.x << ", " << p.y << "], ";
        }
        fout << "]" << std::endl;
        lv=2;
        fout << string(indent(lv), ' ') <<  "]" << std::endl;
        fout << std::endl;
    }
    fout.close();
}

void AddFurniPanel::save_file() {
    QString fpath = QFileDialog::getSaveFileName(nullptr, "Save As", QDir::homePath(), "YAML Files (*.yaml *.yml)");
    if (fpath.isEmpty()) {
        fpath = QString::fromStdString(string(ros::package::getPath("add_furni_gui") + "/yaml/output.yaml"));
    }
    writeFile(fpath.toStdString());
    ROS_INFO("Saving file completed");
}

void AddFurniPanel::pubMarker() {
    for (const auto& it : furni_list_) {
        visualization_msgs::Marker polygon = createPolygon(it.furni.plane, furni_markers_.markers.size());
        furni_markers_.markers.push_back(polygon);
    }
    pub_.publish(furni_markers_);
    ROS_INFO("Publishing furni/markers");
}

visualization_msgs::Marker AddFurniPanel::createPolygon(const std::vector<geometry_msgs::Point>& plane, int id) {
    visualization_msgs::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = ros::Time::now();
    m.ns = "furni";
    m.id = id;
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.color.a = 1.0;
    m.color.r = (rand() % 256) / 255.0;
    m.color.g = (rand() % 256) / 255.0;
    m.color.b = (rand() % 256) / 255.0;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;

    for (size_t i = 0; i < plane.size(); ++i) {
        m.points.push_back(plane[i]);                       // Current point
        m.points.push_back(plane[(i + 1) % plane.size()]);  // Next point (wraps to first)
    }    

    return m;
}

// visualization_msgs::Marker AddFurniPanel::createPolygon(const std::vector<geometry_msgs::Point>& plane, int id) {
//     visualization_msgs::Marker m;
//     m.header.frame_id = "map";
//     m.header.stamp = ros::Time::now();
//     m.ns = "furni";
//     m.id = id;
//     m.type = visualization_msgs::Marker::CUBE_LIST;
//     m.action = visualization_msgs::Marker::ADD;
//     m.pose.orientation.w = 1.0;
//     m.pose.orientation.x = 0.0;
//     m.pose.orientation.y = 0.0;
//     m.pose.orientation.z = 0.0;
//     m.color.a = 1.0;   // Alpha (opacity)
//     m.color.r = (rand() % 256) / 255.0;
//     m.color.g = (rand() % 256) / 255.0;
//     m.color.b = (rand() % 256) / 255.0;

//     geometry_msgs::Point middle_point;
//     middle_point.z = 0.0;
//     for (const auto& point : plane) {
//         middle_point.x += point.x;
//         middle_point.y += point.y;
//     }
//     middle_point.x /= 4;
//     middle_point.y /= 4;
//     m.scale.x = fabs(plane[1].x - plane[0].x);
//     m.scale.y = fabs(plane[1].y - plane[2].y);
//     m.scale.z = 0.1;
//     ROS_INFO("middle_point.x: %f, middle_point.y: %f", middle_point.x, middle_point.y);
//     m.points.push_back(middle_point);

//     return m;
// }

void AddFurniPanel::clearMarker(visualization_msgs::MarkerArray& ma, const std::string& ns) {
    size_t size = ma.markers.size();
    ma.markers.clear();

    visualization_msgs::Marker m;
    m.action = visualization_msgs::Marker::DELETE;
    m.header.frame_id = "map";
    m.ns = ns;
    for (size_t i=0;i<size;i++) {
        m.id = i;
        ma.markers.push_back(m);
    }

    pub_.publish(ma);

    ROS_INFO("Clearing all %s/markers", ns.c_str());
}

void AddFurniPanel::clearMarker() {
    clearMarker(plane_markers_, "plane");
    clearMarker(furni_markers_, "furni");
}

visualization_msgs::Marker AddFurniPanel::createSphere(const geometry_msgs::Point& p) {
    visualization_msgs::Marker m;

    m.header.frame_id = "map";
    m.header.stamp = ros::Time::now();
    m.ns = "plane";
    m.id = plane_markers_.markers.size();
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position = p;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.color.a = 1.0;
    
    return m;
}

} // namespace add_furni_gui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(add_furni_gui::AddFurniPanel, rviz::Panel)

