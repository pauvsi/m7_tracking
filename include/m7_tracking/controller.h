#include <stdio.h>
//#include <boost/lexical_cast.hpp>
#include <sensor_msgs/Image.h>
//#include <tf2_ros/message_filter.h>
//#include <tf2_ros/transform_listener.h>
//#include <tf2_ros/transform_broadcaster.h>
//#include <tf/message_filter.h>
//#include <tf/transform_broadcaster.h>
//#include <tf/transform_listener.h>
//#include <ros/ros.h>
//#include <ros/publisher.h>
//#include <vector>
//#include <string.h>
#include <iterator>
#include <std_msgs/Header.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>

#include "tracker.h"
#include "ground_vehicle.h"
#include "rplidar.h"

#define testing true

//#define CAMERA_TOPIC_1 "bottom_camera/image_rect_color"
//#define CAMERA_TOPIC_2 "front_camera/image_rect_color"
//#define CAMERA_TOPIC_3 "right_camera/image_rect_color"
//#define CAMERA_TOPIC_4 "back_camera/image_rect_color"
//#define CAMERA_TOPIC_5 "left_camera/image_rect_color"

#define CAMERA_TOPIC_1 "bottom_camera/image_raw"
#define CAMERA_TOPIC_2 "front_camera/image_raw"
#define CAMERA_TOPIC_3 "right_camera/image_raw"
#define CAMERA_TOPIC_4 "back_camera/image_raw"
#define CAMERA_TOPIC_5 "left_camera/image_raw"

#define CAMERA_FRAME_1 "bottom_camera"
#define CAMERA_FRAME_2 "front_camera"
#define CAMERA_FRAME_3 "right_camera"
#define CAMERA_FRAME_4 "back_camera"
#define CAMERA_FRAME_5 "left_camera"

#define CAMERA_INFO_1 "bottom_camera/camera_info"
#define CAMERA_INFO_2 "front_camera/camera_info"
#define CAMERA_INFO_3 "right_camera/camera_info"
#define CAMERA_INFO_4 "bottom_camera/camera_info"


#define REDILOWHUE 0
#define	REDIHIGHHUE 8
#define REDILOWSATURATION 165
#define REDIHIGHSATURATION 256
#define REDILOWVALUE 105
#define REDIHIGHVALUE 256

#define GREENILOWHUE 50
#define	GREENIHIGHHUE 62
#define GREENILOWSATURATION 185
#define GREENIHIGHSATURATION 240
#define GREENILOWVALUE 215
#define GREENIHIGHVALUE 255

#define SPACE_BETWEEN_ROOMBA 0.2

//typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

class Controller{
public:
	Controller();
	virtual ~Controller();
	void init();
	void updateAndInit();
	void getReadings();
	void removeCopies();
	void mergeCopies(std::vector<tf::Vector3>& uniquePoses);
	void updateTargetPos();
	void updateObsPos();
	void run();
	void publishAll();
	void callback(const sensor_msgs::ImageConstPtr& img1, const sensor_msgs::CameraInfoConstPtr& cam1, const sensor_msgs::ImageConstPtr& img2, const sensor_msgs::CameraInfoConstPtr& cam2, const sensor_msgs::ImageConstPtr& img3, const sensor_msgs::CameraInfoConstPtr& cam3, const sensor_msgs::ImageConstPtr& img4, const sensor_msgs::CameraInfoConstPtr& cam4, const sensor_msgs::ImageConstPtr& img5);

	bool initialized;
	bool fullInit;

private:
	ros::NodeHandle nh;
	ros::Publisher posPub;
	ros::Publisher red1, red2, red3, red4, red5, green1, green2, green3, green4, green5;
	ros::Publisher obs1, obs2, obs3, obs4;
	std::vector<Tracker*> redTargetTracker;
	std::vector<Tracker*> greenTargetTracker;
	std::vector<GroundVehicle*> targets;
	std::vector<tf::Vector3> uniqueRedPoses;
	std::vector<tf::Vector3> uniqueGreenPoses;
	std::vector<tf::Vector3> uniqueObstaclePoses;

//	message_filters::Subscriber<sensor_msgs::Image>* image1_sub;
//	message_filters::Subscriber<sensor_msgs::Image>* image2_sub;
//	message_filters::Subscriber<sensor_msgs::Image>* image3_sub;
//	message_filters::Subscriber<sensor_msgs::Image>* image4_sub;
//	message_filters::Subscriber<sensor_msgs::Image>* image5_sub;
//	message_filters::Synchronizer<MySyncPolicy>* sync;

	ObstacleDetector obsDet;
	std::vector<GroundVehicle*> obstacles;
	std_msgs::Header imageHeader;
	int redTargetCtr, greenTargetCtr;
	int obsCtr;
};



