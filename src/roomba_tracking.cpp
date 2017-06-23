#include <ros/ros.h>
#include "m7_tracking/tracking.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "m7_tracking", ros::init_options::AnonymousName);
    Tracker track;

    ros::spin();
    return 0;
}
