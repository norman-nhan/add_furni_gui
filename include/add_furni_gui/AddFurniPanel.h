#pragma once
#ifndef Q_MOC_RUN
#include <rviz/panel.h>
#include <ros/ros.h>
#endif

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

#include <QWidget>
#include <QLineEdit>
#include <QPushButton>
#include <QComboBox>
#include <QString>
#include <QBoxLayout>

#include "add_furni_gui/OpenFile.h"
#include "add_furni_gui/FurniWithName.h"
#include <vector>

namespace add_furni_gui {
class AddFurniPanel: public rviz::Panel
{
    Q_OBJECT
public:
    AddFurniPanel(QWidget* parent=0);
    ~AddFurniPanel();

protected Q_SLOTS:
    void open_file();
    void save_file();
    void start_record();
    void delete_furni();

protected:
    // Qt //
    QComboBox* name_box_{new QComboBox};
    QComboBox* belong_box_{new QComboBox};
    QPushButton* open_button_{new QPushButton("Open File")};
    QPushButton* save_button_{new QPushButton("Save As")};
    QPushButton* record_button_{new QPushButton("Record Plane")};
    QPushButton* delete_button_{new QPushButton("Delete")};
    // ROS //
    ros::NodeHandle nh_;
    ros::ServiceClient client_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    // Markers
    visualization_msgs::MarkerArray furni_markers_;
    visualization_msgs::MarkerArray plane_markers_;
    // Stores
    std::vector<FurniWithName> furni_list_;
    std::vector<geometry_msgs::Point> points_;
    bool recording_{false};

protected:
    void onInitialize() override;

    QHBoxLayout* createLayout(QString label, QWidget* wid);
    QHBoxLayout* createComboBox(QString label, QComboBox* wid);
    void addUnique(QComboBox* box, const QString& item);
    int indent(int x);
    void writeFile(const std::string& fo);
    int getFurniIndex(const std::string& name);
    double round(double x);
    void addPoint(const geometry_msgs::PointStamped::ConstPtr& msg);
    visualization_msgs::Marker createPolygon(const std::vector<geometry_msgs::Point>& plane, int id);
    visualization_msgs::Marker createSphere(const geometry_msgs::Point& p);
    void pubMarker();
    void clearMarker(visualization_msgs::MarkerArray& ma, const std::string& ns);
    void clearMarker(); //clear all markers
};
} //end add_furni_gui