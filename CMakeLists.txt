cmake_minimum_required(VERSION 3.0.2)
project(add_furni_gui)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    pluginlib
    rviz
    message_generation
    std_msgs
    geometry_msgs
)

# Add custom msg
set(MSG_FILES
    Furni.msg
    FurniWithName.msg
)
add_message_files(
    FILES
    ${MSG_FILES}
)

# Add custom srv
set(SRV_FILES
    OpenFile.srv
)
add_service_files(
    FILES
    ${SRV_FILES}
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

find_package(yaml-cpp REQUIRED)

# Qt configulation
set(CMAKE_AUTOMOC ON)
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED
    Core
    Widgets
)

set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp roslib pluginlib rviz message_runtime
    DEPENDS YAML_CPP
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
)

set(SRC_FILES
    src/AddFurniPanel.cpp
    include/${PROJECT_NAME}/AddFurniPanel.h
)

add_library(${PROJECT_NAME}
    ${SRC_FILES}
)

set(EXEC_FILES
    src/add_furni_gui_node.cpp
    src/AddFurniNode.cpp
)

add_executable(${PROJECT_NAME}_node
    ${EXEC_FILES}
)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${QT_LIBRARIES}
    ${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
    plugin_description.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY srv
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)