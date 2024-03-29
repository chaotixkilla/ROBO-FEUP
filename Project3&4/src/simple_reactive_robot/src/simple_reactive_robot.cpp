#include "simple_reactive_robot.hpp"

using namespace std;

SimpleReactiveRobot::SimpleReactiveRobot(char *operation)
{
    int aux = atoi(operation);
    switch (aux)
    {
    case 1:
        ROS_INFO_STREAM("WALL FOLLOWING INITIALIZED");
        publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        subscriber = nh.subscribe("/scan", 1, &SimpleReactiveRobot::laserCallback, this);
        alreadyCheckedSide = false;
        break;
    case 2:
        ROS_INFO_STREAM("LINE FOLLOWING INITIALIZED");
        opMode = 1;
        publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        subscriber = nh.subscribe("/camera/rgb/image_raw", 1, &SimpleReactiveRobot::imageCallback, this);
        break;
    case 3:
        ROS_INFO_STREAM("OBJECT FOLLOWING INITIALIZED");
        opMode = 2;
        publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        subscriber = nh.subscribe("/camera/color/image_raw", 1, &SimpleReactiveRobot::imageCallback, this);
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

    if (!alreadyCheckedSide && minimumDistance >= this->laserScan.range_min)
    {
        this->isWallLeftOnStart = isWallLeft(this->laserScan);
        alreadyCheckedSide = true;
    }

    float modifier = isWallLeftOnStart ? -1.0 : 1.0;

    geometry_msgs::Twist message;

    if (minimumDistance < this->laserScan.range_max && minimumDistance >= this->laserScan.range_min)
    {
        if (isInFront(this->laserScan, index))
        {
            message.linear.x = 
            MAX_LINEAR_VELOCITY * 
            
            (minimumDistance - ROBOT_RADIUS - MIN_DISTANCE_TO_WALL) / 
            (IDEAL_DISTANCE_TO_WALL - MIN_DISTANCE_TO_WALL) * 
            
            abs(sin(degToRad(getScanLineAngle(this->laserScan, index)))) / 
            sin(degToRad(80.0));
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

        message.angular.z = (modifier * 2 * (sin(degToRad(90 - getScanLineAngle(this->laserScan, index))) - (minimumDistance - ROBOT_RADIUS - IDEAL_DISTANCE_TO_WALL)));
    }
    else
    {
        message.linear.x = MAX_LINEAR_VELOCITY;
        message.angular.z = 0.1;
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

void SimpleReactiveRobot::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv::Scalar color1;
    cv::Scalar color2;
    cv_bridge::CvImagePtr cv_pointer;

    if (opMode == 1)
    {
        color1 = cv::Scalar(20, 100, 100);
        color2 = cv::Scalar(30, 255, 255);
    }
    else if (opMode == 2)
    {
        color1 = cv::Scalar(170, 120, 120);
        color2 = cv::Scalar(190, 255, 255);
    }
    else
    {
        ROS_INFO_STREAM("ERROR: Invalid Operation Mode on Simple Reactive Robot");
        return;
    }

    try
    {
        cv_pointer = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat img = cv_pointer->image;

    cv::Mat hsvImage, gaussBlurredImage, maskedImage, gaussBlurredImageAux;
    cv::GaussianBlur(img, gaussBlurredImage, cv::Size(3, 3), 0.1, 0.1);
    int imageWidth = gaussBlurredImage.size().width;
    int imageHeight = gaussBlurredImage.size().height;
    cv::cvtColor(gaussBlurredImage, hsvImage, CV_BGR2HSV);
    cv::inRange(hsvImage, color1, color2, maskedImage);

    if (opMode == 1) //cut top 65% of image
    {
        maskedImage(cv::Rect(0, 0, imageWidth, 0.65 * imageHeight)) = 0;
    }

    cv::Moments m = cv::moments(maskedImage);
    cv::Point centroid(m.m10 / m.m00, m.m01 / m.m00);
    cv::circle(maskedImage, centroid, 5, cv::Scalar(128, 0, 0), -1); //show circle on screen

    geometry_msgs::Twist message;

    const int WIDTH_TOLERANCE = imageWidth * 0.1;

    message.linear.x = MAX_LINEAR_VELOCITY * WIDTH_TOLERANCE / abs(centroid.x - imageWidth / 2);

    if (message.linear.x > MAX_LINEAR_VELOCITY)
    {
        message.linear.x = MAX_LINEAR_VELOCITY;
    }
    else if (message.linear.x < MIN_LINEAR_VELOCITY)
    {
        message.linear.x = MIN_LINEAR_VELOCITY;
    }

    if (centroid.x < imageWidth / 2 - WIDTH_TOLERANCE)
    {
        message.angular.z = IDEAL_ANGULAR_VELOCITY * (MAX_LINEAR_VELOCITY - message.linear.x) / MAX_LINEAR_VELOCITY;
    }
    else if (centroid.x > imageWidth / 2 + WIDTH_TOLERANCE)
    {
        message.angular.z = -IDEAL_ANGULAR_VELOCITY * (MAX_LINEAR_VELOCITY - message.linear.x) / MAX_LINEAR_VELOCITY;
    }
    else
    {
        message.angular.z = 0.0;
    }

    publisher.publish(message);

    cv::namedWindow("Robot View", CV_WINDOW_NORMAL);
    cv::imshow("Robot View", gaussBlurredImage);
    cv::namedWindow("Masked View", CV_WINDOW_NORMAL);
    cv::imshow("Masked View", maskedImage);
    cv::waitKey(30);
}