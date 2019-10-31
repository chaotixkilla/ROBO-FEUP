//This is a ROS version of the standard "hello, world"

//This header defines the standard ROS classes.
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

using namespace std;

const float PI = 3.141592653589793238463;
const float MIN_DISTANCE_TO_WALL = 0.5;

class SimpleReactiveRobot {
    private:
        ros::NodeHandle nh;
        ros::Publisher publisher; //publishes to robot velocity
        ros::Subscriber subscriber; //subscribes to robot laser
        sensor_msgs::LaserScan laserScan;
        bool isWallLeftOnStart;
        bool alreadyCheckedSide;
    
    public:
        SimpleReactiveRobot() {
            publisher = nh.advertise<geometry_msgs::Twist>("robot0/cmd_vel", 1000);
            subscriber = nh.subscribe("robot0/laser_0", 1000, &SimpleReactiveRobot::laserCallback, this);
            alreadyCheckedSide = false;
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

        bool isWallLeft() {
            int index = this->getShortestLaserScanIndex();
            return getScanLineAngle(index) > 0;
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

            if(!alreadyCheckedSide) {
                this->isWallLeftOnStart = isWallLeft();
                //ROS_INFO_STREAM(getScanLineAngle(index));
                alreadyCheckedSide = true;
            }

            float modifier = isWallLeftOnStart ? -1.0 : 1.0;

            geometry_msgs::Twist message;

            if(minimumDistance < this->laserScan.range_max) {
                message.linear.x = 0.5;
                float aux = (modifier * 10 * (sin(this->degToRad(90 - this->getScanLineAngle(index))) - (minimumDistance - MIN_DISTANCE_TO_WALL)) * message.linear.x);
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

    ros::spin();
}