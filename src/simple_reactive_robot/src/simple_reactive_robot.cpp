//This is a ROS version of the standard "hello, world"

//This header defines the standard ROS classes.
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

using namespace std;

const float PI = 3.141592653589793238463;
const float IDEAL_DISTANCE_TO_WALL = 0.15;
const float MAX_LINEAR_VELOCITY = 0.5;
const float MIN_LINEAR_VELOCITY = 0.1 * MAX_LINEAR_VELOCITY;
const float MAX_ANGULAR_VELOCITY = 1.2;
const float ROBOT_RADIUS = 0.1;
const float IN_FRONT_ANGLE = 90.0;

class SimpleReactiveRobot {
    private:
        ros::NodeHandle nh;
        ros::Publisher publisher; //publishes to robot velocity
        ros::Subscriber subscriber; //subscribes to robot laser
        sensor_msgs::LaserScan laserScan;
        bool isWallLeftOnStart;
        bool alreadyCheckedSide;
        float maxDistanceFromWall;
        float minDistanceFromWall;
        bool isNextToWall;
    
    public:
        SimpleReactiveRobot() {
            publisher = nh.advertise<geometry_msgs::Twist>("robot0/cmd_vel", 1000);
            subscriber = nh.subscribe("robot0/laser_0", 1000, &SimpleReactiveRobot::laserCallback, this);
            alreadyCheckedSide = false;
            maxDistanceFromWall = 0;
            minDistanceFromWall = 5;
            isNextToWall = false;
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
            return getScanLineAngle(shortestDistanceIndex) < IN_FRONT_ANGLE && getScanLineAngle(shortestDistanceIndex) > -IN_FRONT_ANGLE;
        }

        void laserCallback(const sensor_msgs::LaserScan &scan) {
            this->laserScan = scan;
            int index = this->getShortestLaserScanIndex();
            float currentDistance = this->laserScan.ranges[index];

            if(!alreadyCheckedSide) {
                this->isWallLeftOnStart = isWallLeft();
                alreadyCheckedSide = true;
            }

            float modifier = isWallLeftOnStart ? -1.0 : 1.0;

            geometry_msgs::Twist message;

            if(currentDistance < this->laserScan.range_max) {

                if((currentDistance - ROBOT_RADIUS) <= 2 * IDEAL_DISTANCE_TO_WALL){
                    isNextToWall = true;
                }
                if(isNextToWall){
                    if((currentDistance - ROBOT_RADIUS) < minDistanceFromWall)
                    minDistanceFromWall = (currentDistance - ROBOT_RADIUS);
                    if((currentDistance - ROBOT_RADIUS) > maxDistanceFromWall)
                    maxDistanceFromWall = (currentDistance - ROBOT_RADIUS);
                }

                if(isInFront(index)) {
                    message.linear.x = (currentDistance - ROBOT_RADIUS) * MAX_LINEAR_VELOCITY * abs(sin(degToRad(getScanLineAngle(index)))) / sin(degToRad(IN_FRONT_ANGLE)) / (IDEAL_DISTANCE_TO_WALL);
                } else {
                    message.linear.x = MAX_LINEAR_VELOCITY;
                }

                if(message.linear.x > MAX_LINEAR_VELOCITY) {
                    message.linear.x = MAX_LINEAR_VELOCITY;
                } else if (message.linear.x < MIN_LINEAR_VELOCITY) {
                    message.linear.x = MIN_LINEAR_VELOCITY;
                }

                //message.linear.x = 0.1;
                float aux = (modifier * 10 * (sin(this->degToRad(90 - this->getScanLineAngle(index))) - (currentDistance - ROBOT_RADIUS - IDEAL_DISTANCE_TO_WALL)) * message.linear.x);
                message.angular.z = aux;
            } else {
                message.linear.x = 0.1;
                message.angular.z = 0.0;
            }

            if(message.angular.z > MAX_ANGULAR_VELOCITY) {
                message.angular.z = MAX_ANGULAR_VELOCITY;
            } else if (message.angular.z < -MAX_ANGULAR_VELOCITY) {
                message.angular.z = -MAX_ANGULAR_VELOCITY;
            }

            //ROS_INFO_STREAM(maxDistanceFromWall);
            //ROS_INFO_STREAM(minDistanceFromWall);

            this->getPublisher().publish(message);
        }

};

int main(int argc, char** argv) {
    //Initialize the ROS system.
    ros::init(argc, argv, "simple_reactive_robot");

    SimpleReactiveRobot robot;

    ros::spin();
}