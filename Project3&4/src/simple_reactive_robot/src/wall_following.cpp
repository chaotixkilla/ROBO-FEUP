#include "wall_following.h"

using namespace std;

float getScanLineAngle(sensor_msgs::LaserScan laserScan, int index)
{
    return radToDeg(laserScan.angle_min + laserScan.angle_increment * index);
}

int getShortestLaserScanIndex(sensor_msgs::LaserScan laserScan)
{
    float shortestLaserScan = 9999999.0;
    int shortestLaserScanIndex = -1;

    for (int i = 0; i < laserScan.ranges.size(); i++)
    {
        if (laserScan.ranges[i] < shortestLaserScan)
        {
            shortestLaserScan = laserScan.ranges[i];
            shortestLaserScanIndex = i;
        }
    }

    return shortestLaserScanIndex;
}

bool isWallLeft(sensor_msgs::LaserScan laserScan)
{
    int index = getShortestLaserScanIndex(laserScan);
    return getScanLineAngle(laserScan, index) > 0;
}

bool isInFront(sensor_msgs::LaserScan laserScan, int shortestDistanceIndex)
{
    return getScanLineAngle(laserScan, shortestDistanceIndex) < 80.0 && getScanLineAngle(laserScan, shortestDistanceIndex) > -80.0;
}
