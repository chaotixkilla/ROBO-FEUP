#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include "utils.h"

const float IDEAL_DISTANCE_TO_WALL = 1;
const float MIN_DISTANCE_TO_WALL = 0.5 * IDEAL_DISTANCE_TO_WALL;
const float MAX_LINEAR_VELOCITY = 0.5;
const float MIN_LINEAR_VELOCITY = 0.1 * MAX_LINEAR_VELOCITY;
const float MAX_ANGULAR_VELOCITY = 1.25;
const float ROBOT_RADIUS = 0.2;

float getScanLineAngle(sensor_msgs::LaserScan laserScan, int index);

int getShortestLaserScanIndex(sensor_msgs::LaserScan laserScan);

bool isWallLeft(sensor_msgs::LaserScan laserScan);

bool isInFront(sensor_msgs::LaserScan laserScan, int shortestDistanceIndex);
