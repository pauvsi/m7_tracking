#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>

#include <tf/transform_listener.h>

#include "../include/m7_tracking/Tracker.h"
#include "../include/m7_tracking/Params.h"


tf::TransformListener* tf_listener;

Tracker tracker;

void camera1_callback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam){
	ROS_DEBUG_STREAM("CAMERA_1_IMG_RECEIVED");
	tracker.run(img);

}

void camera2_callback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam){
	ROS_DEBUG_STREAM("CAMERA_2_IMG_RECEIVED");
}

void camera3_callback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam){
	ROS_DEBUG_STREAM("CAMERA_3_IMG_RECEIVED");
}

void camera4_callback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam){
	ROS_DEBUG_STREAM("CAMERA_4_IMG_RECEIVED");
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "m7_tracking");
	ros::NodeHandle nh;
	ROS_INFO_STREAM("SUBSCSRIBING");
	tf_listener = new tf::TransformListener();

	ros::param::param<std::string>("~image_topic_1", CAMERA_IMAGE_TOPIC_1, D_CAMERA_IMAGE_TOPIC_1);
	ros::param::param<std::string>("~image_topic_2", CAMERA_IMAGE_TOPIC_2, D_CAMERA_IMAGE_TOPIC_2);
	ros::param::param<std::string>("~image_topic_3", CAMERA_IMAGE_TOPIC_3, D_CAMERA_IMAGE_TOPIC_3);
	ros::param::param<std::string>("~image_topic_4", CAMERA_IMAGE_TOPIC_4, D_CAMERA_IMAGE_TOPIC_4);

	ros::param::param<int>("~bounds_red_hue_high_two", RED_HUE_HSV_HIGH_TWO, D_RED_HUE_HSV_HIGH_TWO);
	ros::param::param<int>("~bounds_red_hue_low_two", RED_HUE_HSV_LOW_TWO, D_RED_HUE_HSV_LOW_TWO);
	ros::param::param<int>("~bounds_red_hue_high_one", RED_HUE_HSV_HIGH_ONE, D_RED_HUE_HSV_HIGH_ONE);
	ros::param::param<int>("~bounds_red_hue_low_one", RED_HUE_HSV_LOW_ONE, D_RED_HUE_HSV_LOW_ONE);
	ros::param::param<int>("~bounds_red_saturation_high", RED_SATURATION_HSV_HIGH, D_RED_SATURATION_HSV_HIGH);
	ros::param::param<int>("~bounds_red_saturation_low", RED_SATURATION_HSV_LOW, D_RED_SATURATION_HSV_LOW);
	ros::param::param<int>("~bounds_red_value_high", RED_VALUE_HSV_HIGH, D_RED_VALUE_HSV_HIGH);
	ros::param::param<int>("~bounds_red_value_high", RED_VALUE_HSV_LOW, D_RED_VALUE_HSV_LOW);

	ros::param::param<int>("~bounds_green_hue_high", GREEN_HUE_HSV_HIGH, D_GREEN_HUE_HSV_HIGH);
	ros::param::param<int>("~bounds_green_hue_low", GREEN_HUE_HSV_LOW, D_GREEN_HUE_HSV_LOW);
	ros::param::param<int>("~bounds_green_saturation_high", GREEN_SATURATION_HSV_HIGH, D_GREEN_SATURATION_HSV_HIGH);
	ros::param::param<int>("~bounds_green_saturation_low", GREEN_SATURATION_HSV_LOW, D_GREEN_SATURATION_HSV_LOW);
	ros::param::param<int>("~bounds_green_value_high", GREEN_VALUE_HSV_HIGH, D_GREEN_VALUE_HSV_HIGH);
	ros::param::param<int>("~bounds_green_value_low", GREEN_VALUE_HSV_LOW, D_GREEN_VALUE_HSV_LOW);

	ros::param::param<int>("~bounds_canny", CANNY_THRESHOLD, D_CANNY_THRESHOLD);

	ROS_INFO_STREAM("SUBSCSRIBING");
	// setup image transport
	image_transport::ImageTransport it1(nh);
	image_transport::CameraSubscriber cam1_sub = it1.subscribeCamera(CAMERA_IMAGE_TOPIC_1, 10, camera1_callback);

	image_transport::ImageTransport it2(nh);
	image_transport::CameraSubscriber cam2_sub = it2.subscribeCamera(CAMERA_IMAGE_TOPIC_2, 10, camera2_callback);

	image_transport::ImageTransport it3(nh);
	image_transport::CameraSubscriber cam3_sub = it3.subscribeCamera(CAMERA_IMAGE_TOPIC_3, 10, camera3_callback);

	image_transport::ImageTransport it4(nh);
	image_transport::CameraSubscriber cam4_sub = it4.subscribeCamera(CAMERA_IMAGE_TOPIC_4, 10, camera4_callback);

	ROS_INFO_STREAM("SUBSCRIBED");

	ros::spin();

	return 0;
}
