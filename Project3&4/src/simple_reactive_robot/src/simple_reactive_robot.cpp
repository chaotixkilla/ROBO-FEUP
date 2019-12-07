//This is a ROS version of the standard "hello, world"

//This header defines the standard ROS classes.
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

using namespace std;

const float PI = 3.141592653589793238463;
const float IDEAL_DISTANCE_TO_WALL = 1;
const float MIN_DISTANCE_TO_WALL = 0.5 * IDEAL_DISTANCE_TO_WALL;
const float MAX_LINEAR_VELOCITY = 0.5;
const float MIN_LINEAR_VELOCITY = 0.1 * MAX_LINEAR_VELOCITY;
const float MAX_ANGULAR_VELOCITY = 1.25;
const float ROBOT_RADIUS = 0.2;

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
            publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
            subscriber = nh.subscribe("/scan", 1000, &SimpleReactiveRobot::laserCallback, this);
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

        bool isInFront(int shortestDistanceIndex) {
            return getScanLineAngle(shortestDistanceIndex) < 80.0 && getScanLineAngle(shortestDistanceIndex) > -80.0;
        }

        void laserCallback(const sensor_msgs::LaserScan &scan) {
            this->laserScan = scan;
            int index = this->getShortestLaserScanIndex();
            float minimumDistance = this->laserScan.ranges[index];

            if(!alreadyCheckedSide) {
                this->isWallLeftOnStart = isWallLeft();
                alreadyCheckedSide = true;
            }

            float modifier = isWallLeftOnStart ? -1.0 : 1.0;

            geometry_msgs::Twist message;

            if(minimumDistance < this->laserScan.range_max) {

                if(isInFront(index)) {
                    message.linear.x = (minimumDistance - ROBOT_RADIUS - MIN_DISTANCE_TO_WALL) * MAX_LINEAR_VELOCITY * abs(sin(degToRad(getScanLineAngle(index)))) / sin(degToRad(80.0)) / (IDEAL_DISTANCE_TO_WALL - MIN_DISTANCE_TO_WALL);
                } else {
                    message.linear.x = MAX_LINEAR_VELOCITY;
                }

                if(message.linear.x > MAX_LINEAR_VELOCITY) {
                    message.linear.x = MAX_LINEAR_VELOCITY;
                } else if (message.linear.x < MIN_LINEAR_VELOCITY) {
                    message.linear.x = MIN_LINEAR_VELOCITY;
                }

                //message.linear.x = 0.1;
                float aux = (modifier * 10 * (sin(this->degToRad(90 - this->getScanLineAngle(index))) - (minimumDistance - ROBOT_RADIUS - IDEAL_DISTANCE_TO_WALL)) * message.linear.x);
                message.angular.z = aux;

                /*float print1 = sin(this->degToRad(90 - this->getScanLineAngle(index)));
                float print2 = minimumDistance - ROBOT_RADIUS - IDEAL_DISTANCE_TO_WALL;

                ROS_INFO_STREAM("sin(this->degToRad(90 - this->getScanLineAngle(index))");
                ROS_INFO_STREAM(print1);
                ROS_INFO_STREAM("minimumDistance - ROBOT_RADIUS - IDEAL_DISTANCE_TO_WALL");
                ROS_INFO_STREAM(print2);
                ROS_INFO_STREAM("ANGULAR VALUE BEFORE LIMITATION");
                ROS_INFO_STREAM(aux);*/
            } else {
                message.linear.x = 0.1;
                message.angular.z = 0.0;
            }

            if(message.angular.z > MAX_ANGULAR_VELOCITY) {
                message.angular.z = MAX_ANGULAR_VELOCITY;
            } else if (message.angular.z < -MAX_ANGULAR_VELOCITY) {
                message.angular.z = -MAX_ANGULAR_VELOCITY;
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