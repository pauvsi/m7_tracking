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

Controller* controlMain;

void callback(const sensor_msgs::ImageConstPtr& img1, const sensor_msgs::CameraInfoConstPtr& cam1, const sensor_msgs::ImageConstPtr& img2, const sensor_msgs::CameraInfoConstPtr& cam2, const sensor_msgs::ImageConstPtr& img3, const sensor_msgs::CameraInfoConstPtr& cam3, const sensor_msgs::ImageConstPtr& img4, const sensor_msgs::CameraInfoConstPtr& cam4, const sensor_msgs::ImageConstPtr& img5)
{
	(*controlMain).callback(img1, cam1, img2, cam2, img3, cam3, img4, cam4, img5);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "m7_tracking", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    controlMain = new Controller;
    listener = new tf::TransformListener();
    message_filters::Subscriber<sensor_msgs::Image> image1_sub(nh, CAMERA_TOPIC_1, 20);
    message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo1_sub(nh, CAMERA_INFO_1, 20);
    message_filters::Subscriber<sensor_msgs::Image> image2_sub(nh, CAMERA_TOPIC_2, 20);
    message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo2_sub(nh, CAMERA_INFO_2, 20);
    message_filters::Subscriber<sensor_msgs::Image> image3_sub(nh, CAMERA_TOPIC_3, 20);
    message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo3_sub(nh, CAMERA_INFO_3, 20);
    message_filters::Subscriber<sensor_msgs::Image> image4_sub(nh, CAMERA_TOPIC_4, 20);
    message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo4_sub(nh, CAMERA_INFO_4, 20);
    message_filters::Subscriber<sensor_msgs::Image> image5_sub(nh, CAMERA_TOPIC_5, 20);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image> MySyncPolicy;

	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(50), image1_sub, cinfo1_sub, image2_sub, cinfo2_sub, image3_sub, cinfo3_sub, image4_sub, cinfo4_sub, image5_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5, _6, _7, _8, _9));
//    Controller controlMain;
//	std::vector<Tracker*> redTargetTracker;
//	redTargetTracker.push_back( (new Tracker(CAMERA_TOPIC_1, CAMERA_FRAME_1, REDILOWHUE, REDIHIGHHUE, REDILOWSATURATION, REDIHIGHSATURATION, REDILOWVALUE, REDIHIGHVALUE)));
//	redTargetTracker.push_back( (new Tracker("back_camera/image_raw", "back_camera", REDILOWHUE, REDIHIGHHUE, REDILOWSATURATION, REDIHIGHSATURATION, REDILOWVALUE, REDIHIGHVALUE)));
//    Tracker track("bottom_camera/image_raw", "bottom_camera", REDILOWHUE, REDIHIGHHUE, REDILOWSATURATION, REDIHIGHSATURATION, REDILOWVALUE, REDIHIGHVALUE);

//	Tracker redTargetTracker[2];
//	redTargetTracker[0] = *( Tracker(CAMERA_TOPIC_5, CAMERA_FRAME_5, REDILOWHUE, REDIHIGHHUE, REDILOWSATURATION, REDIHIGHSATURATION, REDILOWVALUE, REDIHIGHVALUE));
//	redTargetTracker[1] = *( Tracker("back_camera/image_raw", "back_camera", REDILOWHUE, REDIHIGHHUE, REDILOWSATURATION, REDIHIGHSATURATION, REDILOWVALUE, REDIHIGHVALUE));

//	while(ros::ok())
//    {
////    	ROS_INFO_STREAM("Size:" << (*(redTargetTracker[0])).getPoses().size());
//    	for(auto& e : redTargetTracker)
//    	{
//        	std::vector<tf::Vector3> val = (*e).getPoses();
//        	if(val.size() > 0)
//        		ROS_WARN_STREAM("GOT IT");
////    		ROS_INFO_STREAM("Tracker size:"<< (*e).getPoses().size());
//    	}
//        ros::spinOnce();
//    }


//	GroundVehicle gv;
//	std_msgs::Header ih;
//	ih.stamp.nsec = 0.12*pow10(9);
//	gv.init(ih, Eigen::Matrix<double, 4, 1>(0, 0, 0, 0));
//	ih.stamp.nsec += 0.01*pow10(9);
//	gv.update(Eigen::Matrix<double, 2, 1>(2, 0), ih);
//	ih.stamp.nsec += 0.41*pow10(9);
//	gv.predict(ih);
//	ih.stamp.nsec += 0.18*pow10(9);
//	gv.predict(ih);
//
//	ih.stamp.nsec -= 0.1*pow10(9);
//	gv.update(Eigen::Matrix<double, 2, 1>(4, 0), ih);
//
//	ih.stamp.nsec += 0.16*pow10(9);
//	gv.update(Eigen::Matrix<double, 2, 1>(6, 0), ih);
//
//	ih.stamp.nsec += 0.61*pow10(9);
//	gv.predict(ih);
//
//	gv.update(Eigen::Matrix<double, 2, 1>(3, 0), ih);
//	ih.stamp.nsec += 0.31*pow10(9);
//	gv.predict(ih);

	Tracker track;

//    Controller controlMain;
//    ObstacleDetector lid;
    while(ros::ok())
    {
//    	if(!controlMain.initialized)
//    		controlMain.init();
//    	else if(!controlMain.fullInit)
//    		controlMain.updateAndInit();
//    	else
//    		controlMain.run();
    	ros::spinOnce();
    }
    return 0;
}
