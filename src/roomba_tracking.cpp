#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>


#include "../include/m7_tracking/Tracker.h"
#include "../include/m7_tracking/Params.h"


void callback(const sensor_msgs::ImageConstPtr& img1, const sensor_msgs::CameraInfoConstPtr& cam1, const sensor_msgs::ImageConstPtr& img2, const sensor_msgs::CameraInfoConstPtr& cam2, const sensor_msgs::ImageConstPtr& img3, const sensor_msgs::CameraInfoConstPtr& cam3, const sensor_msgs::ImageConstPtr& img4, const sensor_msgs::CameraInfoConstPtr& cam4)
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "m7_tracking", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    ros::param::param<std::string>("~image_topic_1", CAMERA_IMAGE_TOPIC_1, D_CAMERA_IMAGE_TOPIC_1);
    ros::param::param<std::string>("~image_topic_2", CAMERA_IMAGE_TOPIC_2, D_CAMERA_IMAGE_TOPIC_2);
    ros::param::param<std::string>("~image_topic_3", CAMERA_IMAGE_TOPIC_3, D_CAMERA_IMAGE_TOPIC_3);
    ros::param::param<std::string>("~image_topic_4", CAMERA_IMAGE_TOPIC_4, D_CAMERA_IMAGE_TOPIC_4);

    ros::param::param<std::string>("~camera_topic_1", CAMERA_INFO_TOPIC_1, D_CAMERA_INFO_TOPIC_1);
    ros::param::param<std::string>("~camera_topic_2", CAMERA_INFO_TOPIC_2, D_CAMERA_INFO_TOPIC_2);
    ros::param::param<std::string>("~camera_topic_3", CAMERA_INFO_TOPIC_3, D_CAMERA_INFO_TOPIC_3);
    ros::param::param<std::string>("~camera_topic_4", CAMERA_INFO_TOPIC_4, D_CAMERA_INFO_TOPIC_4);

    message_filters::Subscriber<sensor_msgs::Image> image1_sub(nh, CAMERA_IMAGE_TOPIC_1, 20);
    message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo1_sub(nh, CAMERA_INFO_TOPIC_1, 20);
    message_filters::Subscriber<sensor_msgs::Image> image2_sub(nh, CAMERA_IMAGE_TOPIC_2, 20);
    message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo2_sub(nh, CAMERA_INFO_TOPIC_2, 20);
    message_filters::Subscriber<sensor_msgs::Image> image3_sub(nh, CAMERA_IMAGE_TOPIC_3, 20);
    message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo3_sub(nh, CAMERA_INFO_TOPIC_3, 20);
    message_filters::Subscriber<sensor_msgs::Image> image4_sub(nh, CAMERA_IMAGE_TOPIC_4, 20);
    message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo4_sub(nh, CAMERA_INFO_TOPIC_4, 20);


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image> MySyncPolicy;

	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(50), image1_sub, cinfo1_sub, image2_sub, cinfo2_sub, image3_sub, cinfo3_sub, image4_sub, cinfo4_sub, image5_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5, _6, _7, _8));


	ros::spin();

    return 0;
}
