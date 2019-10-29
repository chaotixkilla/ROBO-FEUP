//This is a ROS version of the standard "hello, world"

//This header defines the standard ROS classes.
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

using namespace std;

const float PI = 3.141592653589793238463;

class SimpleReactiveRobot {
    private:
        ros::NodeHandle nh;
        ros::Publisher publisher; //publishes to robot velocity
        ros::Subscriber subscriber; //subscribes to robot laser
        sensor_msgs::LaserScan laserScan;
    
    public:
        SimpleReactiveRobot() {
            publisher = nh.advertise<geometry_msgs::Twist>("robot0/cmd_vel", 1000);
            subscriber = nh.subscribe("robot0/laser_0", 1000, &SimpleReactiveRobot::laserCallback, this);
        }

        ~SimpleReactiveRobot() {

        }

        ros::Publisher getPublisher() {
            return this->publisher;
        }

        float radToDeg(float radians) {
            return radians * (180.0 / PI);
        }

        float degToRad(float degrees) {
            return degrees * (PI / 180.0);
        }

        float getScanLineAngle(int index) {
            return this->radToDeg(this->laserScan.angle_min + this->laserScan.angle_increment * index);
        }

        int getShortestLaserScanIndex() {
            float shortestLaserScan = 9999999.0;
            int shortestLaserScanIndex = -1;

             for(int i = 0; i < this->laserScan.ranges.size(); i++) {
                if(this->laserScan.ranges[i] < shortestLaserScan) {
                    shortestLaserScan = this->laserScan.ranges[i];
                    shortestLaserScanIndex = i;
                }
            }

            return shortestLaserScanIndex;
        }

        void laserCallback(const sensor_msgs::LaserScan &scan) {
            this->laserScan = scan;
            int index = this->getShortestLaserScanIndex();
            float minimumDistance = this->laserScan.ranges[index];

            geometry_msgs::Twist message;

            if(minimumDistance < this->laserScan.range_max) {
                message.linear.x = 0.5;
                float aux = (- 5 * (sin(this->degToRad(90 - this->getScanLineAngle(index))) - (minimumDistance - 1.5)) * message.linear.x);
                ROS_INFO_STREAM("ANGULAR VELOCITY: " << aux);
                message.angular.z = aux; //needs better formula
            } else {
                message.linear.x = 0.5;
                message.angular.z = 0.0;
            }

            this->getPublisher().publish(message);
        }

};

int main(int argc, char** argv) {
    //Initialize the ROS system.
    ros::init(argc, argv, "simple_reactive_robot");

    SimpleReactiveRobot robot;

    /*ros::Rate rate(2);
    while(ros::ok()) {
        geometry_msgs::Twist message;
        message.linear.x = 0.1;
        message.angular.z = 0.1;
        robot.getPublisher().publish(message);

        ROS_INFO_STREAM("Sending velocity command: " << "linear=" << message.linear.x << " angular:" << message.angular.z);
        rate.sleep();
    }*/

    ros::spin();
}