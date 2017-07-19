#include <ros/ros.h>
#include "../include/m7_tracking/controller.h"

class Test{
	std::vector<Tracker> track;

public:
	Test()
{
		track.push_back( ( Tracker(CAMERA_TOPIC_5, CAMERA_FRAME_5, REDILOWHUE, REDIHIGHHUE, REDILOWSATURATION, REDIHIGHSATURATION, REDILOWVALUE, REDIHIGHVALUE)));
}
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "m7_tracking", ros::init_options::AnonymousName);
    listener = new tf::TransformListener();
    Test controlMain;
//	std::vector<Tracker> redTargetTracker;
//	redTargetTracker.push_back( Tracker(CAMERA_TOPIC_5, CAMERA_FRAME_5, REDILOWHUE, REDIHIGHHUE, REDILOWSATURATION, REDIHIGHSATURATION, REDILOWVALUE, REDIHIGHVALUE));
//	redTargetTracker.push_back( Tracker("back_camera/image_raw", "back_camera", REDILOWHUE, REDIHIGHHUE, REDILOWSATURATION, REDIHIGHSATURATION, REDILOWVALUE, REDIHIGHVALUE));
//    Tracker track("bottom_camera/image_raw", "bottom_camera", REDILOWHUE, REDIHIGHHUE, REDILOWSATURATION, REDIHIGHSATURATION, REDILOWVALUE, REDIHIGHVALUE);
    ros::spin();


//    Controller controlMain;
//    while(ros::ok())
//    {
//    	controlMain.run();
//    	ros::spinOnce();
//    }
    return 0;
}
