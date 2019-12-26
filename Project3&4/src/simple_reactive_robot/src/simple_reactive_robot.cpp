#include "simple_reactive_robot.hpp"

using namespace std;

SimpleReactiveRobot::SimpleReactiveRobot(char* operation)
{
    int aux = atoi(operation);
    switch(aux) {
        case 1:
            ROS_INFO_STREAM("WALL FOLLOWING INITIALIZED");
            publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
            subscriber = nh.subscribe("/scan", 1000, &SimpleReactiveRobot::laserCallback, this);
            alreadyCheckedSide = false;
            break;
        case 2:
            ROS_INFO_STREAM("LINE FOLLOWING INITIALIZED");
            publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
            subscriber = nh.subscribe("/camera/rgb/image_raw", 1, &SimpleReactiveRobot::imageCallback, this);
            break;
        case 3:
            ROS_INFO_STREAM("OBJECT FOLLOWING INITIALIZED");
            break;
        default:
            ROS_INFO_STREAM("ERROR: Invalid Operation Mode on Simple Reactive Robot");
            break;
    }
}

SimpleReactiveRobot::~SimpleReactiveRobot()
{
}

ros::Publisher SimpleReactiveRobot::getPublisher()
{
    return this->publisher;
}

void SimpleReactiveRobot::laserCallback(const sensor_msgs::LaserScan &scan)
{
    this->laserScan = scan;
    int index = getShortestLaserScanIndex(this->laserScan);
    float minimumDistance = this->laserScan.ranges[index];

    if (!alreadyCheckedSide)
    {
        this->isWallLeftOnStart = isWallLeft(this->laserScan);
        alreadyCheckedSide = true;
    }

    float modifier = isWallLeftOnStart ? -1.0 : 1.0;

    geometry_msgs::Twist message;

    if (minimumDistance < this->laserScan.range_max)
    {

        if (isInFront(this->laserScan, index))
        {
            message.linear.x = (minimumDistance - ROBOT_RADIUS - MIN_DISTANCE_TO_WALL) * MAX_LINEAR_VELOCITY * abs(sin(degToRad(getScanLineAngle(this->laserScan, index)))) / sin(degToRad(80.0)) / (IDEAL_DISTANCE_TO_WALL - MIN_DISTANCE_TO_WALL);
        }
        else
        {
            message.linear.x = MAX_LINEAR_VELOCITY;
        }

        if (message.linear.x > MAX_LINEAR_VELOCITY)
        {
            message.linear.x = MAX_LINEAR_VELOCITY;
        }
        else if (message.linear.x < MIN_LINEAR_VELOCITY)
        {
            message.linear.x = MIN_LINEAR_VELOCITY;
        }

        float aux = (modifier * 10 * (sin(degToRad(90 - getScanLineAngle(this->laserScan, index))) - (minimumDistance - ROBOT_RADIUS - IDEAL_DISTANCE_TO_WALL)) * message.linear.x);
        message.angular.z = aux;
    }
    else
    {
        message.linear.x = 0.1;
        message.angular.z = 0.0;
    }

    if (message.angular.z > MAX_ANGULAR_VELOCITY)
    {
        message.angular.z = MAX_ANGULAR_VELOCITY;
    }
    else if (message.angular.z < -MAX_ANGULAR_VELOCITY)
    {
        message.angular.z = -MAX_ANGULAR_VELOCITY;
    }

    this->getPublisher().publish(message);
}

void SimpleReactiveRobot::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_pointer;
    
    try {
        cv_pointer = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch(cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    cv::Mat img = cv_pointer->image;

    /*if(img.empty()) {
        ROS_INFO_STREAM("IMAGE ERROR");
    } else {
        ROS_INFO_STREAM("NO IMAGE ERROR");
        cv::imshow("Robot View", img);
        cv::waitKey(0);
    }*/
}