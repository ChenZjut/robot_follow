/*
 * @Description: 
 * @Author: ubuntu
 * @Date: 2021/9/2 上午10:33
 * @LastEditors: ubuntu
 * @LastEditTime: 2021/9/2 上午10:33
 * @Version 1.0
 */

#include <people_tracking/people_tracking.h>

namespace PeopleTrackNS
{
    PeopleTrack::PeopleTrack()
        : nh_()
        , pnh_("~")
        , kcf_init_(false)
        , kcfTracker_(true, false, true, false)
        , target_object_("people")
        , twist_stop_(true)

    {
        initROS();
    }

    PeopleTrack::~PeopleTrack()
    {

    }

    void PeopleTrack::initROS()
    {
        pnh_.param<double>("max_linear_speed", max_linear_speed_, 0.5);
        pnh_.param<double>("min_linear_speed", min_linear_spped_, -0.2);
        pnh_.param<double>("max_angle_speed", max_angle_speed_, 1.0);
        pnh_.param<double>("target_distance", target_distance_, 1.0);
        pnh_.param<double>("speed_Kp", pid_speed.p, 0.1);
        pnh_.param<double>("speed_Kd", pid_speed.d, 0.1);
        pnh_.param<double>("angle_Kp", pid_angle.p, 0.002);
        pnh_.param<double>("angle_Kd", pid_angle.d, 0.005);

        rgb_image_sub_ = nh_.subscribe("rgb/image_raw", 10, &PeopleTrack::rgb_cb, this);
        depth_image_sub_ = nh_.subscribe("depth/image_raw", 10, &PeopleTrack::depth_cb, this);
        bbox_sub_ = nh_.subscribe("darknet_ros/bounding_boxes", 10, &PeopleTrack::bbox_cb, this);

        twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    }

    void PeopleTrack::bbox_cb(const darknet_ros_msgs::BoundingBoxesConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr.reset();

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg->image, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& ex)
        {
            ROS_ERROR("cv_bridge exception %s", ex.what());
            return;
        }

        for (auto& box : msg->bounding_boxes)
        {
            if (box.Class == target_object_)
            {
                Rect rect(box.xmin, box.ymin, box.xmax - box.xmin, box.ymax - box.ymin);
                kcfTracker_.init(rect, cv_ptr->image);
                kcf_init_ = true;
                break;
            }
        }
    }

    void PeopleTrack::rgb_cb(const sensor_msgs::ImageConstPtr &msg)
    {
        if (kcf_init_ == false)
            return;

        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr.reset();

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& ex)
        {
            ROS_ERROR("rgb exception: %s", ex.what());
            return;
        }
        rect_ = kcfTracker_.update(cv_ptr->image);
    }

    void PeopleTrack::depth_cb(const sensor_msgs::ImageConstPtr &msg)
    {
        geometry_msgs::Twist speed_cmd;

        // while the box is missing, stop the robot
        if (kcf_init_ == false || twist_stop_ == true)
        {
            speed_cmd.linear.x = 0;
            speed_cmd.angular.z = 0;
            kcf_init_ = false;
            twist_pub_.publish(speed_cmd);
        }

        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr.reset();
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch (cv_bridge::Exception& ex)
        {
            ROS_ERROR("depth exception %s", ex.what());
            return;
        }

        double target_angle;
        double now_angle;
        double diff_angle;

        double target_distance;
        double now_distance;
        double diff_distance;

        target_angle = cv_ptr->image.cols / 2;
        now_angle = rect_.x + rect_.width / 2;
        diff_angle = PID_control(now_angle, target_angle, pid_angle);

        speed_cmd.angular.z = diff_angle;
        cout << "diff_angle " << diff_angle << endl;

        if (speed_cmd.angular.z > max_angle_speed_)
        {
            speed_cmd.angular.z = max_angle_speed_;
        }
        else if (speed_cmd.angular.z < -max_angle_speed_)
        {
            speed_cmd.angular.z = -max_angle_speed_;
        }

        if (fabs(target_angle - now_angle) > 90)
        {
            target_distance = target_distance_;
            vector<double> dist_list;

            int center_x = rect_.x + rect_.width / 2;
            int center_y = rect_.y + rect_.height / 2;

            for (int i = center_y - 10; i <= center_y + 10; i++)
            {
                for (int j = center_x - 10; j <= center_x + 10; j++)
                {
                    dist_list.push_back(cv_ptr->image.at<float>(i, j));
                }
            }

            double dis = 0;
            int num = dist_list.size();
            for(size_t i = 0; i < dist_list.size(); i++)
            {
                if (dist_list[i] > 0.1 && dist_list[i] < 8.0)
                {
                    dis += dist_list[i];
                }
                else
                {
                    num--;
                }
            }

            if (num == 0)
            {
                return;
            }

            dis /= num;

            if (dis > 2.0)
            {
                cout << "too far " << endl;
            }

            if (dis < target_distance_)
            {
                cout << "too close " << endl;
            }

            now_distance = dis;
            diff_distance = PID_control(now_distance, target_distance, pid_speed);

            speed_cmd.linear.x = -diff_distance;

            if (speed_cmd.linear.x > max_linear_speed_)
            {
                speed_cmd.linear.x = max_linear_speed_;
            }
            else if (speed_cmd.linear.x < min_linear_spped_)
            {
                speed_cmd.linear.x = min_linear_spped_;
            }
        }

        twist_pub_.publish(speed_cmd);
    }

    double PeopleTrack::PID_control(double now, double target, PID pid)
    {
        pid.e0 = target - now;
        double pe = pid.e0;
        double de = pid.e0 - pid.e1;
        double diff = pid.p * pe + pid.d * de;
        pid.e1 = pid.e0;
        return diff;
    }

    void PeopleTrack::run()
    {
        ros::Rate loop_rate(10);

        while (ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
}