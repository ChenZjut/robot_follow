/*
 * @Description: 
 * @Author: ubuntu
 * @Date: 2021/9/2 上午10:12
 * @LastEditors: ubuntu
 * @LastEditTime: 2021/9/2 上午10:12
 * @Version 1.0
 */

#include <people_tracking/people_tracking.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "people_tracking");
    PeopleTrackNS::PeopleTrack app;
    app.run();
    ros::spin();
    return 0;
}