#include <stdio.h>
#include <boost/lexical_cast.hpp>
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

#include "tracker.hpp"
#include "ground_vehicle.hpp"
#include "rplidar.hpp"

#define CAMERA_TOPIC_1 "/cameraBottom/image_color"
#define CAMERA_TOPIC_2 "/cameraFront/image_color"
#define CAMERA_TOPIC_3 "/cameraRight/image_color"
#define CAMERA_TOPIC_4 "/cameraBack/image_color"
#define CAMERA_TOPIC_5 "/cameraLeft/image_color"

#define CAMERA_FRAME_1 "camera_Bottom_frame"
#define CAMERA_FRAME_2 "camera_Front_frame"
#define CAMERA_FRAME_3 "camera_Right_frame"
#define CAMERA_FRAME_4 "camera_Back_frame"
#define CAMERA_FRAME_5 "camera_Left_frame"

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
	void init();
	void getReadings();
	void removeCopies();
	void mergeCopies(std::vector<tf::Vector3> &uniquePoses);
	void updateTargetPos();
	void updateObsPos();
	void run();

private:
	ros::NodeHandle nh;
	ros::Publisher posPub;
	Tracker redTargetTracker[5];
	Tracker greenTargetTracker[5];
	GroundVehicle targets[10];
	std::vector<tf::Vector3> uniqueRedPoses;
	std::vector<tf::Vector3> uniqueGreenPoses;
	std::vector<tf::Vector3> uniqueObstaclePoses;
	ObstacleDetector obsDet;
	GroundVehicle obstacles[4];
	std_msgs::Header imageHeader;
};



