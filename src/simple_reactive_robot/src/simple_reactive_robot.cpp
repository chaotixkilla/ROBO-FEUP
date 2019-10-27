//This is a ROS version of the standard "hello, world"

//This header defines the standard ROS classes.
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

using namespace std;

class SimpleReactiveRobot {
    private:
        ros::NodeHandle nh;
        ros::Publisher publisher; //publishes to robot velocity
        ros::Subscriber subscriber; //subscribes to robot laser
        //sensor_msgs::LaserScan laserScan;
    
    public:

};

int main(int argc, char** argv) {
    //Initialize the ROS system.
    ros::init(argc, argv, "hello_ros");

    //Establish this program as a ROS node.
    ros::NodeHandle nh;

    //Send some output as a log message.
    ROS_INFO_STREAM("Hello, ROS!");
}