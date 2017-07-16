/*
 * tracker.h
 *
 *  Created on: Nov 4, 2016
 *  Author: Logesh Roshan
 */

#ifndef PAUVSI_TRACKER_INCLUDE_TRACKER_H_
#define PAUVSI_TRACKER_INCLUDE_TRACKER_H_


#include <opencv2/calib3d.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/video.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <stdio.h>
#include <boost/lexical_cast.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <vector>
#include <string.h>
#include <std_msgs/Header.h>
#include <float.h>

#define CANNY_THRESHOLD 100
#define ROOMBA_HEIGHT 0.09
#define DEFAULT_CAMERA_TOPIC "/camera/image_color_rect"
#define DEFAULT_ODOM_FRAME_NAME "odom"
#define DEFAULT_CAMERA_FRAME_NAME "camera_frame"
//#define DEFAULT_COM_FRAME_NAME "base_link"
#define DEFAULT_WORLD_FRAME_NAME "world"
#define FISHEYE_DISTORTION false

class Tracker
{
public:

	Tracker();
	virtual ~Tracker();
	Tracker(std::string cameraTopic, std::string cameraFrame, int Target_LowHue, int Target_HighHue, int Target_LowSat, int Target_HighSat,
			int Target_LowValue, int Target_HighValue);
	void readROSParameters();
	void cameraCallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam);
	cv::Mat get3x3FromVector(boost::array<double, 9> vec);
	bool curImgOlder;

	void setK(cv::Mat _K)
	{
		K = _K;
	}
	void setD(cv::Mat _D)
	{
		D = _D;
	}

	void displayTargets();
	void createTrackBars();
	void run();
	void removeOutofBounds();
	void getWorldPosition();

	std_msgs::Header getHeader()
	{
		return imageHeader;
	}

	std::string getCameraTopic(){
			return cameraTopic;
	}

	std::vector<tf::Vector3> getPoses(){
		return worldRoombaPosition;
	}

protected:

	int target_LowHue;
	int target_HighHue;
	int target_LowSat;
	int target_HighSat;
	int target_LowValue;
	int target_HighValue;
	double currentImgTime;
	double lastImgTime;

	std::string cameraTopic;
	std::string camera_frame;
	std::string world_frame;
	std::string odom_frame;

	ros::NodeHandle nh;
	image_transport::CameraSubscriber cameraSub;
	std_msgs::Header imageHeader;
	cv::Mat inputImg;

	tf::TransformListener* listener;



	//(x,y) positions of Roombas within image
	std::vector<cv::Point2f> imgRoombaPoses;
	std::vector<tf::Vector3> worldRoombaPosition;

	//Camera Parameter (Intrinsic and distortion)
	cv::Mat K;
	cv::Mat D;

};

#endif
