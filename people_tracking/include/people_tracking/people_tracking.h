/*
 * @Description: 
 * @Author: ubuntu
 * @Date: 2021/9/2 上午9:39
 * @LastEditors: ubuntu
 * @LastEditTime: 2021/9/2 上午9:39
 * @Version 1.0
 */
#ifndef SRC_PEOPLE_TRACKING_H
#define SRC_PEOPLE_TRACKING_H

// C++
#include <algorithm>
#include <chrono>
#include <iostream>

// ROS
#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

// Opencv
#include <opencv2/opencv.hpp>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>

#include "kcftracker.hpp"

using namespace std;
using namespace cv;

namespace PeopleTrackNS
{
class PID
{
public:
    double e0;
    double e1;

    double p;
    double d;
    PID()
    {
        e0 = 0;
        e1 = 0;
        p = 0;
        d = 0;
    }
};

class PeopleTrack
{
private:

    ros::NodeHandle pnh_;
    ros::NodeHandle nh_;

    // Subscriber
    ros::Subscriber bbox_sub_;
    ros::Subscriber rgb_image_sub_;
    ros::Subscriber depth_image_sub_;

    // Publisher
    ros::Publisher twist_pub_;
    ros::Publisher state_pub_;

    bool kcf_init_;
    KCFTracker kcfTracker_;
    Rect rect_;


    PID pid_speed;
    PID pid_angle;

    // speed limit for PID
    double max_linear_speed_;
    double min_linear_spped_;

    // angle limit for PID
    double max_angle_speed_;

    // Tracking Param
    double target_distance_;
    string target_object_;
    bool twist_stop_;

    // callback
    void rgb_cb(const sensor_msgs::ImageConstPtr& msg);
    void depth_cb(const sensor_msgs::ImageConstPtr& msg);
    void bbox_cb(const darknet_ros_msgs::BoundingBoxesConstPtr& msg);

    // function
    double PID_control(double now, double target, PID pid);

    // init
    void initROS();

public:
    PeopleTrack();
    ~PeopleTrack();
    void run();

};

}


#endif //SRC_PEOPLE_TRACKING_H
