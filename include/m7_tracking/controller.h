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

#include "tracker.h"
#include "ground_vehicle.h"
#include "rplidar.h"

//#define CAMERA_TOPIC_1 "bottom_camera/image_color_rect"
//#define CAMERA_TOPIC_2 "front_camera/image_color_rect"
//#define CAMERA_TOPIC_3 "right_camera/image_color_rect"
//#define CAMERA_TOPIC_4 "back_camera/image_color_rect"
//#define CAMERA_TOPIC_5 "left_camera/image_color_rect"

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

#define REDILOWHUE 0
#define	REDIHIGHHUE 8
#define REDILOWSATURATION 165
#define REDIHIGHSATURATION 256
#define REDILOWVALUE 105
#define REDIHIGHVALUE 256

#define GREENILOWHUE 0
#define	GREENIHIGHHUE 179
#define GREENILOWSATURATION 0
#define GREENIHIGHSATURATION 255
#define GREENILOWVALUE 0
#define GREENIHIGHVALUE 255

#define SPACE_BETWEEN_ROOMBA 0.2

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
	ObstacleDetector obsDet;
	std::vector<GroundVehicle*> obstacles;
	std_msgs::Header imageHeader;
	int redTargetCtr, greenTargetCtr;
	int obsCtr;
};



