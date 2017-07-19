#include <ros/ros.h>
#include "../include/m7_tracking/controller.h"



int main(int argc, char** argv)
{
    ros::init(argc, argv, "m7_tracking", ros::init_options::AnonymousName);
    listener = new tf::TransformListener();
//    Tracker track;
//    ros::spin();


    Controller controlMain;
    while(ros::ok())
    {
    	controlMain.run();
    	ros::spinOnce();
    }
    return 0;
}
