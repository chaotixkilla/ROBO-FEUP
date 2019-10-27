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
        SimpleReactiveRobot() {
            publisher = nh.advertise<geometry_msgs::Twist>("robot0/cmd_vel", 1000);
            //subscriber = nh.subscribe("robot0/laser_0", 1000, laserCallback);
        }

        ~SimpleReactiveRobot() {

        }

        ros::Publisher getPublisher() {
            return this->publisher;
        }

};

int main(int argc, char** argv) {
    //Initialize the ROS system.
    ros::init(argc, argv, "simple_reactive_robot");

    SimpleReactiveRobot robot;

    ros::Rate rate(2);
    while(ros::ok()) {
        geometry_msgs::Twist message;
        message.linear.x = 0.1;
        message.angular.z = 0.1;
        robot.getPublisher().publish(message);

        ROS_INFO_STREAM("Sending velocity command: " << "linear=" << message.linear.x << " angular:" << message.angular.z);
        rate.sleep();
    }

    //ros::spin();
}