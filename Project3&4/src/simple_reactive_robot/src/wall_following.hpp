#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include "utils.hpp"

const float IDEAL_DISTANCE_TO_WALL = 0.3;
const float MIN_DISTANCE_TO_WALL = 0.05;

float getScanLineAngle(sensor_msgs::LaserScan laserScan, int index);

int getShortestLaserScanIndex(sensor_msgs::LaserScan laserScan);

bool isWallLeft(sensor_msgs::LaserScan laserScan);

bool isInFront(sensor_msgs::LaserScan laserScan, int shortestDistanceIndex);
