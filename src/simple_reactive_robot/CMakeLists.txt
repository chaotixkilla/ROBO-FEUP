cmake_minimum_required(VERSION 2.8.3)

project(simple_reactive_robot)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs
)

catkin_package(
    LIBRARIES
    CATKIN_DEPENDS
    roscpp
    sensor_msgs
    geometry_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(simple_reactive_robot 
    src/simple_reactive_robot.cpp
)

target_link_libraries(
    simple_reactive_robot
    ${catkin_LIBRARIES}
)