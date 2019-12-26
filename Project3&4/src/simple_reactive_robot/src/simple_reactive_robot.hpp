#ifndef SIMPLE_REACTIVE_ROBOT
#define SIMPLE_REACTIVE_ROBOT

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "wall_following.hpp"

class SimpleReactiveRobot
{
private:
    ros::NodeHandle nh;
    ros::Publisher publisher;   //publishes to robot velocity
    ros::Subscriber subscriber; //subscribes to robot laser
    sensor_msgs::LaserScan laserScan;
    bool isWallLeftOnStart;
    bool alreadyCheckedSide;

public:
    SimpleReactiveRobot(char* operation);

    ~SimpleReactiveRobot();

    ros::Publisher getPublisher();

    void laserCallback(const sensor_msgs::LaserScan &scan);
};

#endif