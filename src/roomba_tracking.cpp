#include <ros/ros.h>
#include "../include/m7_tracking/controller.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "m7_tracking", ros::init_options::AnonymousName);
    Controller controlMain;
    controlMain.run();

    ros::spin();
    return 0;
}
