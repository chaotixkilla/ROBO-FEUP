#include "ros/ros.h"
#include "simple_reactive_robot.hpp"

using namespace std;

int main(int argc, char** argv) {
    //Initialize the ROS system.
    ros::init(argc, argv, "simple_reactive_robot");

    SimpleReactiveRobot robot(argv[1]);

    ros::Rate rosRate = 10;

    //ros::spin();

    while(ros::ok()){
        ros::spinOnce();
        rosRate.sleep();
    }
}