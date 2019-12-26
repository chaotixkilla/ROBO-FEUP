cmake_minimum_required(VERSION 2.8.3)

project(simple_reactive_robot)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    cv_bridge
)

find_package(OpenCV REQUIRED)

catkin_package(
    LIBRARIES
    CATKIN_DEPENDS
    roscpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    cv_bridge
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(simple_reactive_robot 
    src/main.cpp
    src/utils.cpp
    src/simple_reactive_robot.cpp
    src/wall_following.cpp
)

target_link_libraries(
    simple_reactive_robot
    ${catkin_LIBRARIES}
)
target_link_libraries(
    simple_reactive_robot
    ${OpenCV_LIBRARIES}
)
